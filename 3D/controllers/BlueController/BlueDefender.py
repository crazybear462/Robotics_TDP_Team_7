"""
Blue Team Defender (Zone-based + Collaborative standby/pass-back, ONE-BY-ONE motions)

ADDED (only):
1) STRICT one-by-one: never interrupt any motion until current motion isOver()
2) ENTER_STRIP_MODE (your new rule):
   - If (ball_zone < 10) AND (both strikers are also opponent side: zone < 10)
   - Defender must ENTER zones 10..12 by reaching the LOWEST X boundary of zones 10..12 (x_min = PLAY_ZONE[10][0][0] = 0.00)
   - IMPORTANT: Use forwardsSprint (NOT backwards). So we first FACE the entry target, then forwardsSprint toward it.
   - Once entered (sx <= x_min + margin), switch back to your previous behavior: track ball Y in the strip.
3) If at least one striker is NOT on opponent side, we do NOT force strip entry; base logic runs.

NOTE: Your original decide logic is kept inside _decideMotion_base (same as what you pasted).
"""

import os, sys, math
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils import Functions
from Utils.Consts import TIME_STEP
import BlueTeamStrategies


class Defender(SoccerRobot):

  def _norm180(self, a):
    # normalize angle to [-180, 180)
    while a >= 180: a -= 360
    while a < -180: a += 360
    return a

  def _log(self, msg):
    print(f"[BLUE_DEF] {msg}")

  def _ball3(self, ball):
    if len(ball) == 3:
      return ball[0], ball[1], ball[2]
    return ball[0], ball[1], 0.0

  def _pose3(self, p):
    if len(p) == 3:
      return p[0], p[1], p[2]
    return p[0], p[1], 0.0

  def _dist2d(self, ax, ay, bx, by):
    dx = ax - bx
    dy = ay - by
    return (dx*dx + dy*dy) ** 0.5

  def _is_stable_now(self, roll_th=0.60, pitch_th=0.60, z_min=0.27):
    roll, pitch, yaw = self.getRollPitchYaw()
    z = self.getSelfCoordinate()[2]
    roll_ok = abs(math.sin(roll)) <= math.sin(roll_th)
    pitch_ok = abs(math.sin(pitch)) <= math.sin(pitch_th)
    stable = roll_ok and pitch_ok and (z >= z_min)
    return stable, roll, pitch, z

  # ============================
  # STRICT one-by-one gating
  # ============================
  def _can_switch_motion(self):
    # STRICT: never switch until current motion is finished
    if not self.currentlyMoving:
      return True
    return self.currentlyMoving.isOver()

  def _start(self, motion):
    if motion is None:
      motion = self.motions.standInit

    # STRICT: do not start anything if a motion is still running
    if self.currentlyMoving and (not self.currentlyMoving.isOver()):
      return

    # Avoid re-starting the same motion again and again
    if self.currentlyMoving and self.currentlyMoving.name == motion.name and (not self.currentlyMoving.isOver()):
      return

    if self.isNewMotionValid(motion):
      self.clearMotionQueue()
      self.addMotionToQueue(motion)

    self._log(f"START_MOTION {motion.name}")
    self.startMotion()

  # ===== helpers for collab override =====
  def _turn_motion_60(self, turningAngle, th=30):
    if turningAngle > th:
      return self.motions.turnLeft60
    if turningAngle < -th:
      return self.motions.turnRight60
    return None

  def _face_target(self, target_xy, self_xy, face_th=15):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(
      target_xy, self_xy, robotHeadingAngle
    )

    turningAngle = self._norm180(turningAngle)

    # IMPORTANT: in your project, ~180 means "already facing"
    if abs(turningAngle) <= face_th:
      return None
    if abs(abs(turningAngle) - 180) <= face_th:
      return None

    # avoid left/right flip near +-180 by mapping to a stable direction
    # if we are closer to 180 than to 0, force a consistent sign
    if abs(turningAngle) > 120:
      # treat as "almost aligned" so don't waste time turning forever
      return None

    return self._turn_motion_60(turningAngle, th=30)

  def _pass_to_nearest_striker(self, self_xy, fwL_xy, fwR_xy):
    dL = self._dist2d(self_xy[0], self_xy[1], fwL_xy[0], fwL_xy[1])
    dR = self._dist2d(self_xy[0], self_xy[1], fwR_xy[0], fwR_xy[1])
    target = fwL_xy if dL <= dR else fwR_xy

    robotHeadingAngle = self.getRollPitchYaw()[2]
    ang = Functions.calculateTurningAngleAccordingToRobotHeading(target, self_xy, robotHeadingAngle)

    if ang > 20 and hasattr(self.motions, "rightSidePass"):
      return self.motions.rightSidePass
    if ang < -20 and hasattr(self.motions, "leftSidePass"):
      return self.motions.leftSidePass

    return self.motions.longShoot if hasattr(self.motions, "longShoot") else self.motions.shoot

  # -------------------------------------------------
  # YOUR ORIGINAL decideMotion (DON'T CHANGE IT)
  # kept here as base
  # -------------------------------------------------
  def _decideMotion_base(self, ballCoordinate, selfCoordinate):
    bx, by, _ = self._ball3(ballCoordinate)
    sx, sy, sz = self._pose3(selfCoordinate)

    data = getattr(self, "supervisorData", None) or []
    fwL = [data[30], data[31]] if len(data) > 31 else [sx, sy]
    fwR = [data[33], data[34]] if len(data) > 34 else [sx, sy]

    ball_zone = BlueTeamStrategies.getZone([bx, by])
    self_zone = BlueTeamStrategies.getZone([sx, sy])
    fwL_zone  = BlueTeamStrategies.getZone(fwL)
    fwR_zone  = BlueTeamStrategies.getZone(fwR)

    # ---- tune ----
    DEF_ZONE_MIN = 13
    Y_NEAR = 1.0
    FACE_TH = 15

    STANDBY_Z_MIN = 10
    STANDBY_Z_MAX = 12
    STANDBY_CENTER_ZONE = 11

    FRONT_X_OFFSET = 0.25
    TARGET_X_MIN = 2.54
    TARGET_X_MAX = 5.20
    Y_CLAMP = 1.50

    KICK_DIST = 0.28
    STABLE_NEED = 3
    SETTLE_TICKS = 2
    # --------------

    if not hasattr(self, "_stable_streak"):
      self._stable_streak = 0
    if not hasattr(self, "_kick_state"):
      self._kick_state = {"mode": "IDLE", "settle": 0, "kick_name": None}

    stable, roll, pitch, z = self._is_stable_now()
    self._stable_streak = self._stable_streak + 1 if stable else 0

    self_xy = [sx, sy]

    # ----- existing logic -----
    if ball_zone < 10:
      z11 = BlueTeamStrategies.PLAY_ZONE[STANDBY_CENTER_ZONE]
      standby_x = (z11[0][0] + z11[1][0]) / 2.0
      standby_y = max(-Y_CLAMP, min(Y_CLAMP, by))
      y_diff = standby_y - sy

      self._log(f"STANDBY_MODE ballZone={ball_zone} -> standbyZone=10..12 aim=({standby_x:.2f},{standby_y:.2f}) "
                f"selfZone={self_zone} fwLZ={fwL_zone} fwRZ={fwR_zone}")

      tm = self._face_target([standby_x, standby_y], self_xy, face_th=FACE_TH)
      if tm is not None:
        return tm

      if abs(y_diff) <= Y_NEAR:
        return self.motions.sideStepRight if y_diff > 0 else self.motions.sideStepLeft
      return self.motions.forwardsSprint

    in_standby_strip = (STANDBY_Z_MIN <= self_zone <= STANDBY_Z_MAX)
    strikers_on_opponent = (fwL_zone < 10 and fwR_zone < 10)

    if ball_zone >= 12 and in_standby_strip and strikers_on_opponent:
      d_ball = self._dist2d(sx, sy, bx, by)
      self._log(f"RECYCLE_MODE ballZone={ball_zone} selfZone={self_zone} fwLZ={fwL_zone} fwRZ={fwR_zone} dBall={d_ball:.3f}")

      if d_ball > KICK_DIST:
        tm = self._face_target([bx, by], self_xy, face_th=FACE_TH)
        if tm is not None:
          return tm

        y_diff = by - sy
        if abs(y_diff) <= Y_NEAR:
          return self.motions.sideStepRight if y_diff > 0 else self.motions.sideStepLeft
        return self.motions.forwardsSprint

      tm = self._face_target([bx, by], self_xy, face_th=FACE_TH)
      if tm is not None:
        self._kick_state["mode"] = "IDLE"
        return tm

      if self._stable_streak < STABLE_NEED:
        self._log(f"RECYCLE_BLOCK stability {self._stable_streak}/{STABLE_NEED} roll={roll:.3f} pitch={pitch:.3f} z={z:.3f}")
        self._kick_state["mode"] = "IDLE"
        return self.motions.standInit

      passMotion = self._pass_to_nearest_striker(self_xy, fwL, fwR)

      if self._kick_state["mode"] == "IDLE":
        self._kick_state["mode"] = "SETTLE"
        self._kick_state["settle"] = SETTLE_TICKS
        self._kick_state["kick_name"] = passMotion.name
        self._log(f"RECYCLE_SETTLE {SETTLE_TICKS} -> {passMotion.name}")
        return self.motions.standInit

      if self._kick_state["mode"] == "SETTLE":
        self._kick_state["settle"] -= 1
        self._log(f"RECYCLE_SETTLING remaining={self._kick_state['settle']}")
        if self._kick_state["settle"] > 0:
          return self.motions.standInit
        self._kick_state["mode"] = "KICK"

      if self._kick_state["mode"] == "KICK":
        kname = self._kick_state["kick_name"]
        self._kick_state["mode"] = "IDLE"
        self._kick_state["kick_name"] = None
        self._log(f"RECYCLE_PASS_FIRE {kname}")
        return getattr(self.motions, kname) if hasattr(self.motions, kname) else passMotion

      return self.motions.standInit

    if ball_zone < DEF_ZONE_MIN:
      z13 = BlueTeamStrategies.PLAY_ZONE[13]
      guard_x = (z13[0][0] + z13[1][0]) / 2.0
      guard_y = max(-Y_CLAMP, min(Y_CLAMP, by))
      y_diff = guard_y - sy

      self._log(f"GUARD_MODE ballZone={ball_zone} (<13) guard=({guard_x:.2f},{guard_y:.2f}) ydiff={y_diff:+.2f}")

      tm = self._face_target([guard_x, guard_y], self_xy, face_th=FACE_TH)
      if tm is not None:
        return tm

      if abs(y_diff) <= Y_NEAR:
        return self.motions.sideStepRight if y_diff > 0 else self.motions.sideStepLeft
      return self.motions.forwardsSprint

    target_x = max(TARGET_X_MIN, min(TARGET_X_MAX, bx + FRONT_X_OFFSET))
    target_y = max(-Y_CLAMP, min(Y_CLAMP, by))
    d_target = self._dist2d(sx, sy, target_x, target_y)
    y_diff = target_y - sy

    self._log(f"ENGAGE_MODE ballZone={ball_zone} target=({target_x:.2f},{target_y:.2f}) dTarget={d_target:.3f}")

    if d_target <= KICK_DIST:
      tm = self._face_target([bx, by], self_xy, face_th=FACE_TH)
      if tm is not None:
        self._kick_state["mode"] = "IDLE"
        return tm

      clearMotion = self.motions.longShoot if hasattr(self.motions, "longShoot") else self.motions.shoot
      if hasattr(self.motions, "rightSidePass") and by >= 0:
        clearMotion = self.motions.rightSidePass
      if hasattr(self.motions, "leftSidePass") and by < 0:
        clearMotion = self.motions.leftSidePass

      if self._stable_streak < STABLE_NEED:
        self._log(f"CLEAR_BLOCK stability {self._stable_streak}/{STABLE_NEED} roll={roll:.3f} pitch={pitch:.3f} z={z:.3f}")
        self._kick_state["mode"] = "IDLE"
        return self.motions.standInit

      if self._kick_state["mode"] == "IDLE":
        self._kick_state["mode"] = "SETTLE"
        self._kick_state["settle"] = SETTLE_TICKS
        self._kick_state["kick_name"] = clearMotion.name
        self._log(f"CLEAR_SETTLE {SETTLE_TICKS} -> {clearMotion.name}")
        return self.motions.standInit

      if self._kick_state["mode"] == "SETTLE":
        self._kick_state["settle"] -= 1
        self._log(f"CLEAR_SETTLING remaining={self._kick_state['settle']}")
        if self._kick_state["settle"] > 0:
          return self.motions.standInit
        self._kick_state["mode"] = "KICK"

      if self._kick_state["mode"] == "KICK":
        kname = self._kick_state["kick_name"]
        self._kick_state["mode"] = "IDLE"
        self._kick_state["kick_name"] = None
        self._log(f"CLEAR_FIRE {kname}")
        return getattr(self.motions, kname) if hasattr(self.motions, kname) else clearMotion

      return self.motions.standInit

    self._kick_state["mode"] = "IDLE"
    tm = self._face_target([target_x, target_y], self_xy, face_th=FACE_TH)
    if tm is not None:
      return tm

    if abs(y_diff) <= Y_NEAR:
      return self.motions.sideStepRight if y_diff > 0 else self.motions.sideStepLeft
    return self.motions.forwardsSprint

  # -------------------------------------------------
  # NEW WRAPPER decideMotion: ONLY ADD (do not change base)
  # -------------------------------------------------
  def decideMotion(self, ballCoordinate, selfCoordinate):
    bx, by, _ = self._ball3(ballCoordinate)
    sx, sy, _ = self._pose3(selfCoordinate)

    data = getattr(self, "supervisorData", None) or []
    fwL = [data[30], data[31]] if len(data) > 31 else [sx, sy]
    fwR = [data[33], data[34]] if len(data) > 34 else [sx, sy]

    ball_zone = BlueTeamStrategies.getZone([bx, by])
    self_zone = BlueTeamStrategies.getZone([sx, sy])
    fwL_zone  = BlueTeamStrategies.getZone(fwL)
    fwR_zone  = BlueTeamStrategies.getZone(fwR)

    both_strikers_opponent = (fwL_zone < 10 and fwR_zone < 10)

    # ==========================================================
    # ENTER_STRIP_MODE (ONLY when BOTH strikers + ball are opponent side)
    # Your request:
    # - "keep moving forwardsprint (with correct direction) until x reaches the lowest x of zones 10–12"
    # - lowest x boundary for zones 10..12 is PLAY_ZONE[10][0][0] == 0.00
    # - After reaching x_min, keep following ball Y like previous code again.
    # ==========================================================
    if both_strikers_opponent and ball_zone < 10:
      x_min_strip = BlueTeamStrategies.PLAY_ZONE[10][0][0]  # 0.00
      enter_margin = 0.05  # small inside margin
      enter_x = x_min_strip + enter_margin
      enter_y = max(-1.5, min(1.5, by))

      self_xy = [sx, sy]

      if hasattr(self, "_enter_strip_lock"):
        self._enter_strip_lock = False

      # If not yet reached x_min boundary, FORCE travel toward (enter_x, enter_y)
      if sx > enter_x:
        self._log(f"ENTER_STRIP_MODE: ballZone={ball_zone} fwLZ={fwL_zone} fwRZ={fwR_zone} selfX={sx:.2f} -> enter x<=0.00")

        # 1) turn to face the entry target (direction fix)
        # remember we are in "enter strip" mode
        if not hasattr(self, "_enter_strip_lock"):
            self._enter_strip_lock = True

        tm = self._face_target([enter_x, enter_y], self_xy, face_th=15)
        if tm is not None:
            return tm

        return self.motions.forwardsSprint

      # Once x is reached (sx <= enter_x), go back to your unchanged base logic
      return self._decideMotion_base(ballCoordinate, selfCoordinate)

    # If at least one striker is not opponent side -> do NOT force strip
    # (base logic already covers going back to zone>=13 and defending)
    return self._decideMotion_base(ballCoordinate, selfCoordinate)

  # -------------------------------------------------
  def run(self):
    while self.robot.step(TIME_STEP) != -1:

      if not self.isNewBallDataAvailable():
        continue

      self.getSupervisorData()

      ballCoordinate = self.getBallData()
      selfCoordinate = self.getSelfCoordinate()

      # goal events
      if self.checkGoal() == 1:
        if self._can_switch_motion():
          self._start(self.motions.handWave)
        continue
      if self.checkGoal() == -1:
        if self._can_switch_motion():
          self._start(self.motions.standInit)
        continue

      # fall handling
      if selfCoordinate[2] < 0.2:
        if self._can_switch_motion():
          if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
            self._start(self.motions.standUpFromBack)
          else:
            self._start(self.motions.standUpFromFront)
        continue

      # opponent priority
      if self.getBallPriority() == "R":
        if self._can_switch_motion():
          self._start(self.motions.standInit)
        continue

      # STRICT one-by-one gate
      if not self._can_switch_motion():
        continue

      decided = self.decideMotion(ballCoordinate, selfCoordinate)
      self._start(decided)
