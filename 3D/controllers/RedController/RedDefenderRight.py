"""
RED_DEF_R - SIMPLE + ROBUST BALL FOLLOW (CameraTop + CameraBottom)

What it does:
1) If fallen -> stand up using your EXACT sonar rule:
      (left==2.55 and right==2.55) => StandUpFromBack else StandUpFromFront
   Then "settle" in StandInit for a short time.
2) If not fallen:
   - Try detect the BALL from camera images (black+white pattern, not green field).
   - If ball seen: smooth head track + body turns toward ball + walk using Forwards50.
   - If ball near feet: stop (StandInit) based on Bottom camera.
   - If ball not seen: do 360 search (head sweep + body turning), re-check every step.

NOTE about your “body turns opposite to head” issue:
- Set TURN_SIGN = +1 or -1 below.
  If head looks RIGHT but body turns LEFT, set TURN_SIGN = -1 (default here).
  If it becomes correct, keep it. If still wrong, flip it to +1.
"""

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils.Consts import TIME_STEP


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class DefenderRight(SoccerRobot):
    # ----------------------------
    # IMPORTANT: turn mapping
    # ----------------------------
    # -1 fixes: "head right but body turns left"
    TURN_SIGN = -1

    def __init__(self, robot):
        super().__init__(robot)
        print("[DEF_R_BALL] started:", self.robot.getName())

        # --------------------
        # Cameras
        # --------------------
        self.camTop = self._get_camera_any(["CameraTop", "cameraTop", "TopCamera", "camera_top"])
        self.camBottom = self._get_camera_any(["CameraBottom", "cameraBottom", "BottomCamera", "camera_bottom"])

        if self.camTop is None:
            print("[DEF_R_BALL] WARNING: CameraTop not found.")
        if self.camBottom is None:
            print("[DEF_R_BALL] WARNING: CameraBottom not found (near-feet stop weaker).")

        # --------------------
        # Head motors
        # --------------------
        self.headYaw = self._get_motor_any(["HeadYaw", "headYaw", "head_yaw"])
        self.headPitch = self._get_motor_any(["HeadPitch", "headPitch", "head_pitch"])

        self.headYawTarget = 0.0
        self.headPitchTarget = -0.25

        # --------------------
        # Smooth head control (PD + rate limit)
        # --------------------
        self._headYawCmd = 0.0
        self._prevErr = 0.0
        self._errDeadbandPx = 6
        self._yawRateLimit = 0.05

        self._Kp = 0.0040
        self._Kd = 0.0012

        # --------------------
        # Motion thresholds
        # --------------------
        self.TURN_STRONG = 0.70
        self.TURN_START = 0.25
        self.FORWARD_OK = 0.10

        # --------------------
        # "Near feet" stop (BOTTOM camera)
        # --------------------
        self.NEAR_CENTER_PX = 22
        self.NEAR_BOTTOM_FRAC = 0.68

        # --------------------
        # Ball detector settings
        # --------------------
        self.SAMPLE_STEP = 2
        self.MIN_BALL_PIXELS_TOP = 30
        self.MIN_BALL_PIXELS_BOTTOM = 20

        # bounding box constraints to reject posters/field border
        self.MAX_BBOX_FRAC_W = 0.55  # if bbox too wide -> not ball
        self.MAX_BBOX_FRAC_H = 0.55
        self.MIN_BBOX_FRAC_W = 0.02  # too tiny -> noise
        self.MIN_BBOX_FRAC_H = 0.02

        # require both white + black pixels (soccer pattern)
        self.MIN_WHITE = 10
        self.MIN_BLACK = 6

        # --------------------
        # Search (360): HEAD sweep then BODY turn
        # --------------------
        self.search_dir = 1
        self.search_yaw_max = 1.55
        self.search_step = 0.04

        self.search_phase = "HEAD"
        self.head_flip_count = 0
        self.head_flip_limit = 4  # ~2 full sweeps

        self.last_seen_side = 1  # +1 right side of image, -1 left side

        self.search_turn_steps = 0
        self.search_turn_steps_max = int(10.0 * 1000 / TIME_STEP)  # ~10s turning

        # --------------------
        # Standup lock + settle
        # --------------------
        self._standing_up = False
        self._settle_steps = 0
        self._settle_steps_after_standup = int(1.0 * 1000 / TIME_STEP)  # 1.0s settle

        # Debug
        self._dbg = 0

    # ==========================================================
    # MAIN LOOP
    # ==========================================================
    def run(self):
        while self.robot.step(TIME_STEP) != -1:

            # 1) fall/standup lock
            if self._handle_fall_and_standup():
                continue

            # 2) must have at least one camera
            if self.camTop is None and self.camBottom is None:
                self._apply_head()
                self._play(self.motions.standInit)
                continue

            # 3) detect ball (prefer TOP for steering, BOTTOM for near-feet stop)
            top = self._detect_ball(self.camTop, self.MIN_BALL_PIXELS_TOP) if self.camTop else (False, 0, 0, 0, 0, "no_top")
            bot = self._detect_ball(self.camBottom, self.MIN_BALL_PIXELS_BOTTOM) if self.camBottom else (False, 0, 0, 0, 0, "no_bot")

            top_seen, top_cx, top_cy, tw, th, top_info = top
            bot_seen, bot_cx, bot_cy, bw, bh, bot_info = bot

            # Debug print every ~0.5s
            self._dbg += 1
            if self._dbg % max(1, int(500 / TIME_STEP)) == 0:
                print(f"[DEF_R_BALL] top_seen={top_seen} info={top_info} bot_seen={bot_seen} infoB={bot_info}")

            # 4) stop condition: bottom camera says ball at feet
            if self._ball_near_feet(bot_seen, bot_cx, bot_cy, bw, bh):
                self._apply_head()
                self._play(self.motions.standInit)
                continue

            # 5) choose steering source
            if top_seen:
                used_cx, used_w = top_cx, tw
                # remember last seen side for turning search direction
                err_px = used_cx - (used_w / 2.0)
                if abs(err_px) > 2:
                    self.last_seen_side = 1 if err_px > 0 else -1

                # track head smoothly
                self._track_head_to_ball(used_cx, used_w)

                # body follows head + forwards50
                motion = self._body_from_headyaw(self.headYawTarget)
                self._play(motion)
                continue

            # If top not seen but bottom sees (close ball): still move/align using bottom
            if bot_seen:
                used_cx, used_w = bot_cx, bw
                err_px = used_cx - (used_w / 2.0)
                if abs(err_px) > 2:
                    self.last_seen_side = 1 if err_px > 0 else -1

                self._track_head_to_ball(used_cx, used_w)
                motion = self._body_from_headyaw(self.headYawTarget)
                self._play(motion)
                continue

            # 6) ball not seen -> search 360
            self._search_360()

    # ==========================================================
    # STAND UP: your exact sonar rule + settle delay
    # ==========================================================
    def _handle_fall_and_standup(self):
        z = self.getSelfCoordinate()[2]
        fallen = (z < 0.20)

        if fallen and not self._standing_up:
            self._standing_up = True
            self._settle_steps = 0

            try:
                self.interruptMotion()
            except Exception:
                pass
            self.clearMotionQueue()

            # YOUR EXACT RULE
            if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
                self.addMotionToQueue(self.motions.standUpFromBack)
            else:
                self.addMotionToQueue(self.motions.standUpFromFront)

            self.startMotion()
            return True

        if self._standing_up:
            # keep playing until over
            if self.currentlyMoving and not self.currentlyMoving.isOver():
                self.startMotion()
                return True

            # settle
            if self._settle_steps == 0:
                self._settle_steps = self._settle_steps_after_standup

            if self._settle_steps > 0:
                self._settle_steps -= 1
                self._apply_head()
                self._play(self.motions.standInit)
                return True

            self._standing_up = False
            return False

        return False

    # ==========================================================
    # BALL DETECTOR (black + white pattern, not green field)
    # returns: seen, cx, cy, w, h, info
    # ==========================================================
    def _detect_ball(self, cam, min_pixels):
        if cam is None:
            return False, 0.0, 0.0, 0, 0, "no_cam"

        w = cam.getWidth()
        h = cam.getHeight()
        img = cam.getImage()

        step = self.SAMPLE_STEP

        # candidate mask based on color:
        # - white-ish OR black-ish
        # - NOT field green
        white_cnt = 0
        black_cnt = 0
        total = 0

        minx, miny = 10**9, 10**9
        maxx, maxy = -1, -1

        sum_x = 0
        sum_y = 0

        for y in range(0, h, step):
            for x in range(0, w, step):
                r = cam.imageGetRed(img, w, x, y)
                g = cam.imageGetGreen(img, w, x, y)
                b = cam.imageGetBlue(img, w, x, y)

                # reject green field (very green compared to red/blue)
                is_green = (g > 100 and g > r + 30 and g > b + 30)

                # white / black tests (soccer ball has both)
                is_white = (r > 190 and g > 190 and b > 190)
                is_black = (r < 70 and g < 70 and b < 70)

                if is_green:
                    continue

                if is_white or is_black:
                    total += 1
                    sum_x += x
                    sum_y += y
                    if is_white:
                        white_cnt += 1
                    if is_black:
                        black_cnt += 1
                    if x < minx: minx = x
                    if y < miny: miny = y
                    if x > maxx: maxx = x
                    if y > maxy: maxy = y

        if total < min_pixels:
            return False, 0.0, 0.0, w, h, "too_few"

        if white_cnt < self.MIN_WHITE or black_cnt < self.MIN_BLACK:
            return False, 0.0, 0.0, w, h, "no_pattern"

        bw = maxx - minx + 1
        bh = maxy - miny + 1

        # reject giant blobs (posters/borders)
        if bw > self.MAX_BBOX_FRAC_W * w or bh > self.MAX_BBOX_FRAC_H * h:
            return False, 0.0, 0.0, w, h, f"too_big {bw}x{bh}"

        # reject tiny blobs
        if bw < self.MIN_BBOX_FRAC_W * w or bh < self.MIN_BBOX_FRAC_H * h:
            return False, 0.0, 0.0, w, h, "too_small"

        cx = sum_x / float(total)
        cy = sum_y / float(total)
        return True, cx, cy, w, h, f"ok W={white_cnt} B={black_cnt} box={bw}x{bh}"

    # ==========================================================
    # NEAR FEET CHECK (BOTTOM camera preferred)
    # ==========================================================
    def _ball_near_feet(self, bot_seen, bot_cx, bot_cy, bw, bh):
        if not bot_seen or bw == 0 or bh == 0:
            return False

        err_px = bot_cx - (bw / 2.0)
        centered = abs(err_px) <= self.NEAR_CENTER_PX
        low_enough = (bot_cy / float(bh)) >= self.NEAR_BOTTOM_FRAC
        return centered and low_enough

    # ==========================================================
    # SMOOTH HEAD TRACK (PD + deadband + rate limit)
    # ==========================================================
    def _track_head_to_ball(self, cx, w):
        err_px = cx - (w / 2.0)

        if abs(err_px) < self._errDeadbandPx:
            err_px = 0.0

        derr = err_px - self._prevErr
        self._prevErr = err_px

        desired = self._headYawCmd + (self._Kp * err_px) + (self._Kd * derr)
        desired = clamp(desired, -1.6, 1.6)

        delta = desired - self._headYawCmd
        delta = clamp(delta, -self._yawRateLimit, self._yawRateLimit)
        self._headYawCmd += delta

        self.headYawTarget = self._headYawCmd
        self.headPitchTarget = -0.25
        self._apply_head()

    # ==========================================================
    # BODY FROM HEAD YAW (Forwards50 + turn motions)
    # ==========================================================
    def _body_from_headyaw(self, yaw):
        # Fix direction mismatch via TURN_SIGN
        y = self.TURN_SIGN * yaw
        ay = abs(y)

        # When turning body, gently pull head back toward center
        if ay > self.TURN_START:
            self._headYawCmd *= 0.85
            self.headYawTarget = self._headYawCmd

        if ay > self.TURN_STRONG:
            return self.motions.turnRight60 if y > 0 else self.motions.turnLeft60

        if ay > self.TURN_START:
            return self.motions.turnRight40 if y > 0 else self.motions.turnLeft40

        if ay <= self.FORWARD_OK:
            return self.motions.forwards50

        return self.motions.turnRight40 if y > 0 else self.motions.turnLeft40

    # ==========================================================
    # 360 SEARCH: HEAD sweep then BODY turn (re-check every step)
    # ==========================================================
    def _head_search_step(self):
        prev_dir = self.search_dir

        self.headYawTarget += self.search_dir * self.search_step
        if self.headYawTarget > self.search_yaw_max:
            self.headYawTarget = self.search_yaw_max
            self.search_dir = -1
        elif self.headYawTarget < -self.search_yaw_max:
            self.headYawTarget = -self.search_yaw_max
            self.search_dir = 1

        self.headPitchTarget = -0.25
        self._apply_head()

        # reset PD so it doesn't jerk when ball appears
        self._headYawCmd = self.headYawTarget
        self._prevErr = 0.0

        if self.search_dir != prev_dir:
            self.head_flip_count += 1

    def _search_360(self):
        # stop walking during search
        if self.search_phase == "HEAD":
            self._head_search_step()
            self._play(self.motions.standInit)

            if self.head_flip_count >= self.head_flip_limit:
                self.search_phase = "BODY"
                self.search_turn_steps = 0
            return

        # BODY phase: keep turning (full 360 over time)
        self._apply_head()

        # turn toward last seen side (if last seen right, rotate right)
        if self.last_seen_side > 0:
            turn_motion = self.motions.turnRight60
        else:
            turn_motion = self.motions.turnLeft60

        self._play(turn_motion)

        self.search_turn_steps += 1
        if self.search_turn_steps >= self.search_turn_steps_max:
            self.search_phase = "HEAD"
            self.head_flip_count = 0
            self.search_turn_steps = 0

    # ==========================================================
    # HELPERS
    # ==========================================================
    def _apply_head(self):
        try:
            if self.headYaw:
                self.headYaw.setPosition(self.headYawTarget)
            if self.headPitch:
                self.headPitch.setPosition(self.headPitchTarget)
        except Exception:
            pass

    def _play(self, motion):
        if motion is None:
            return
        if self.isNewMotionValid(motion):
            self.clearMotionQueue()
            self.addMotionToQueue(motion)
        self.startMotion()

    def _get_camera_any(self, names):
        for name in names:
            try:
                d = self.robot.getDevice(name)
                if d:
                    try:
                        d.enable(TIME_STEP)
                    except Exception:
                        pass
                    return d
            except Exception:
                pass
        return None

    def _get_motor_any(self, names):
        for name in names:
            try:
                d = self.robot.getDevice(name)
                if d:
                    return d
            except Exception:
                pass
        return None
