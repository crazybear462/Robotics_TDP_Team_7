"""
Blue Defender (ONE-BY-ONE motions, no spam switching)

Fix for your log:
- You were deciding a new motion every step, so it keeps queueing standInit in between.
- This version runs a "motion gate": if a motion is running and NOT over, we DO NOTHING.
  (Exception: standInit can be replaced anytime.)

What you want (confirmed):
- Hold around x = 2.95
- Danger zone: ball.x in [2.6 .. 5.0]
- Outside danger: ONLY track ball Y (do NOT approach ball in X)
- Inside danger: align Y first, then approach fast and clear/pass/shoot.
- Y tracking:
  - if near -> sideStep
  - if far  -> turn30 -> forwardsSprint -> turnBack30
- Standup must finish first. No shaking: require stable streak after standup.
"""

import os, sys, math
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils import Functions
from Utils.Consts import TIME_STEP


class Defender(SoccerRobot):

  # -------------------------
  # logs
  # -------------------------
  def _log(self, msg):
    print(f"[BLUE_DEF] {msg}")

  # -------------------------
  # wrap-safe stability
  # -------------------------
  def _is_stable(self, roll_thresh=0.60, pitch_thresh=0.60, z_min=0.25):
    roll, pitch, yaw = self.getRollPitchYaw()
    z = self.getSelfCoordinate()[2]
    roll_ok = abs(math.sin(roll)) <= math.sin(roll_thresh)
    pitch_ok = abs(math.sin(pitch)) <= math.sin(pitch_thresh)
    stable = roll_ok and pitch_ok and (z >= z_min)
    return stable, roll, pitch, z

  def _dist2d(self, a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (dx * dx + dy * dy) ** 0.5

  # -------------------------
  # motion gate: do one motion fully
  # -------------------------
  def _can_decide_new(self):
    """
    Return True only if we are allowed to pick a new motion now.

    - If currentlyMoving is None -> True
    - If current motion is standInit -> True (interruptible)
    - Else: only True if current motion isOver()
    """
    if not self.currentlyMoving:
      return True
    if self.currentlyMoving.name == self.motions.standInit.name:
      return True
    return self.currentlyMoving.isOver()

  def _start(self, motion):
      if not motion:
          motion = self.motions.standInit

      # ✅ if already doing this motion and it’s not over, don’t restart it
      if self.currentlyMoving and self.currentlyMoving.name == motion.name and (not self.currentlyMoving.isOver()):
          return

      if self.isNewMotionValid(motion):
          self.clearMotionQueue()
          self.addMotionToQueue(motion)

      self._log(f"START_MOTION {motion.name}")
      self.startMotion()

  # -------------------------
  # Y follow: near sidestep, far turn+ sprint + turnback
  # -------------------------
  def y_follow_motion(self, target_y, self_y, Y_DB, FAR_Y):
      """
      FAR:  turn60 -> (forwardsSprint/forwards50 repeated) until aligned -> turnBack60
      NEAR: sidestep (optional; you can keep it or make it also angled-forward)
      Convention: +Y = RIGHT, -Y = LEFT
      """

      if not hasattr(self, "_yseq"):
          self._yseq = {"active": False, "phase": 0, "dir": None}

      y_err = target_y - self_y
      abs_err = abs(y_err)

      # aligned -> if we were in a sequence, finish by turning back; else do nothing
      if abs_err <= Y_DB:
          if self._yseq["active"] and self._yseq["phase"] == 1:
              # We were moving angled; now we should turn back to face the field/ball
              self._yseq["phase"] = 2
          else:
              # fully done
              self._yseq["active"] = False
              self._yseq["phase"] = 0
              self._yseq["dir"] = None
              return None

      want_dir = "POS" if y_err > 0 else "NEG"  # POS=RIGHT, NEG=LEFT
      is_far = abs_err >= FAR_Y

      # Required motions (you asked for 60)
      turnLeft = self.motions.turnLeft60
      turnRight = self.motions.turnRight60

      # +Y is RIGHT, -Y is LEFT
      def turn_toward(d):
          return turnRight if d == "POS" else turnLeft

      def turn_back(d):
          return turnLeft if d == "POS" else turnRight

      # Choose forward speed while angled
      forward_move = self.motions.forwardsSprint if is_far else self.motions.forwards50

      # --- NEAR: you can keep sidestep for precision ---
      if not is_far and abs_err > Y_DB:
          if y_err > 0:
              self._log(f"MOVE_Y NEAR: sideStepRight (err={y_err:+.2f})")
              return self.motions.sideStepRight
          else:
              self._log(f"MOVE_Y NEAR: sideStepLeft (err={y_err:+.2f})")
              return self.motions.sideStepLeft

      # Start/restart the angled-forward sequence if needed
      if (not self._yseq["active"]) or (self._yseq["dir"] != want_dir):
          self._yseq["active"] = True
          self._yseq["phase"] = 0
          self._yseq["dir"] = want_dir

      # PHASE 0: turn toward once
      if self._yseq["phase"] == 0:
          self._yseq["phase"] = 1
          self._log(f"MOVE_Y FAR: TURN {'RIGHT' if want_dir == 'POS' else 'LEFT'}60 (err={y_err:+.2f})")
          return turn_toward(want_dir)

      # PHASE 1: keep moving forward UNTIL aligned (this is the key change)
      if self._yseq["phase"] == 1:
          # Still not aligned -> keep pushing forward again and again
          if abs_err > Y_DB:
              self._log(f"MOVE_Y FAR: FORWARD {'SPRINT' if is_far else 'WALK'} (err={y_err:+.2f})")
              return forward_move

          # aligned -> next call will turn back
          self._yseq["phase"] = 2

      # PHASE 2: turn back once, then reset sequence
      self._yseq["active"] = False
      self._yseq["phase"] = 0
      self._log(f"MOVE_Y FAR: TURN BACK {'LEFT' if want_dir == 'POS' else 'RIGHT'}60")
      return turn_back(want_dir)

  # -------------------------
  # main decision
  # -------------------------
  def decideMotion(self, ballCoordinate, selfCoordinate,
                   HOLD_X,
                   DANGER_X_MIN, DANGER_X_MAX,
                   X_DB, Y_DB, FAR_Y,
                   CLOSE_D2D):

    bx, by, _ = ballCoordinate
    sx, sy, _ = selfCoordinate

    in_danger = (DANGER_X_MIN <= bx <= DANGER_X_MAX)

    # HOLD_X small correction only (keep at x=2.95)
    x_err = sx - HOLD_X
    if abs(x_err) > X_DB * 2:
      if x_err > 0:
        self._log(f"HOLD_X: too far +x -> backwards (sx={sx:.2f} hold={HOLD_X:.2f})")
        return self.motions.backwards
      else:
        self._log(f"HOLD_X: too far -x -> forwards50 (sx={sx:.2f} hold={HOLD_X:.2f})")
        return self.motions.forwards50

    # Always follow ball y (even if ball moved manually)
    target_y = by
    self._log(f"FOLLOW_Y target_y={target_y:.2f} (ball y={by:.2f}) danger={in_danger}")

    # OUTSIDE danger: ONLY Y tracking, NEVER chase ball
    if not in_danger:
      y_m = self.y_follow_motion(target_y, sy, Y_DB, FAR_Y)
      return y_m if y_m is not None else self.motions.standInit

    # INSIDE danger:
    # 1) align Y first
    y_m = self.y_follow_motion(target_y, sy, Y_DB, FAR_Y)
    if y_m is not None:
      self._log("DANGER: Y-ALIGN first")
      return y_m

    # 2) aligned -> rush/clear
    d2d = self._dist2d(ballCoordinate, selfCoordinate)
    if d2d <= CLOSE_D2D:
      # clear sideways away from opponent (fallback longShoot)
      if by >= 0:
        self._log(f"DANGER: close d2d={d2d:.3f} -> CLEAR rightSidePass/longShoot")
        return self.motions.rightSidePass if hasattr(self.motions, "rightSidePass") else self.motions.longShoot
      else:
        self._log(f"DANGER: close d2d={d2d:.3f} -> CLEAR leftSidePass/longShoot")
        return self.motions.leftSidePass if hasattr(self.motions, "leftSidePass") else self.motions.longShoot

    self._log(f"DANGER: aligned -> RUSH forwardsSprint (d2d={d2d:.3f})")
    return self.motions.forwardsSprint

  # -------------------------
  # run loop (one-by-one)
  # -------------------------
  def run(self):

    HOLD_X       = 2.95
    DANGER_X_MIN = 2.60
    DANGER_X_MAX = 5.00

    X_DB      = 0.10
    Y_DB      = 0.08
    FAR_Y     = 0.70
    CLOSE_D2D = 0.28

    STANDUP_MOTIONS = {self.motions.standUpFromBack.name, self.motions.standUpFromFront.name}
    KICK_MOTIONS = {
      self.motions.longShoot.name if hasattr(self.motions, "longShoot") else "longShoot",
      self.motions.rightShoot.name if hasattr(self.motions, "rightShoot") else "rightShoot",
      self.motions.leftSidePass.name if hasattr(self.motions, "leftSidePass") else "leftSidePass",
      self.motions.rightSidePass.name if hasattr(self.motions, "rightSidePass") else "rightSidePass",
      self.motions.shoot.name if hasattr(self.motions, "shoot") else "shoot",
    }

    # post-standup stability
    if not hasattr(self, "_stable_count"):
      self._stable_count = 0
    STABLE_NEED = 1

    while self.robot.step(TIME_STEP) != -1:

      if not self.isNewBallDataAvailable():
        continue

      self.getSupervisorData()

      ballCoordinate = self.getBallData()
      selfCoordinate = self.getSelfCoordinate()

      # -----------------------------
      # 1) HARD LOCK: standup must finish (prevents shaking)
      # -----------------------------
      if self.currentlyMoving and (self.currentlyMoving.name in STANDUP_MOTIONS) and (not self.currentlyMoving.isOver()):
        self._log(f"LOCK standup (keep {self.currentlyMoving.name})")
        # do NOT requeue anything
        continue

      # update stable streak
      stable, roll, pitch, z = self._is_stable()
      self._stable_count = (self._stable_count + 1) if stable else 0

      # -----------------------------
      # 2) HARD LOCK: kick/pass must finish
      # -----------------------------
      if self.currentlyMoving and (self.currentlyMoving.name in KICK_MOTIONS) and (not self.currentlyMoving.isOver()):
        self._log(f"LOCK kick/pass (keep {self.currentlyMoving.name})")
        continue

      # -----------------------------
      # 3) ONE-BY-ONE gate:
      # if a motion is running and not over, don't switch.
      # -----------------------------
      if not self._can_decide_new():
        continue

      # -----------------------------
      # 4) gates (goal, fall, priority)
      # -----------------------------
      if self.checkGoal() == 1:
        self._start(self.motions.handWave)
        continue
      elif self.checkGoal() == -1:
        self._start(self.motions.standInit)
        continue

      # fall detection
      if selfCoordinate[2] < 0.2:
        if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
          self._start(self.motions.standUpFromBack)
        else:
          self._start(self.motions.standUpFromFront)
        # reset stable streak after standup starts
        self._stable_count = 0
        continue

      # opponent priority
      if self.getBallPriority() == "R":
        self._start(self.motions.standInit)
        continue

      # -----------------------------
      # 5) post-standup stabilize before actions
      # -----------------------------
      if self._stable_count < STABLE_NEED:
          self._log(f"STABILIZE {self._stable_count}/{STABLE_NEED} (light)")
          # don't force standInit forever; just avoid risky moves this tick



      # -----------------------------
      # 6) decide next motion (single) and start it
      # -----------------------------
      decided = self.decideMotion(
        ballCoordinate, selfCoordinate,
        HOLD_X,
        DANGER_X_MIN, DANGER_X_MAX,
        X_DB, Y_DB, FAR_Y,
        CLOSE_D2D
      )

      # final stability guard (block only risky motions)
      stable, roll, pitch, z = self._is_stable()

      HARD_RISKY = {
          self.motions.longShoot.name if hasattr(self.motions, "longShoot") else "longShoot",
          self.motions.shoot.name if hasattr(self.motions, "shoot") else "shoot",
          self.motions.rightShoot.name if hasattr(self.motions, "rightShoot") else "rightShoot",
          self.motions.leftSidePass.name if hasattr(self.motions, "leftSidePass") else "leftSidePass",
          self.motions.rightSidePass.name if hasattr(self.motions, "rightSidePass") else "rightSidePass",
      }

      # ✅ Sprint is "medium risk": if unstable, downgrade to forwards50 (still moves)
      if decided and decided.name == self.motions.forwardsSprint.name and (not stable):
          self._log("GUARD: unstable -> downgrade forwardsSprint -> forwards50")
          decided = self.motions.forwards50

      # ✅ Only kicks/power moves are blocked when unstable
      if decided and (decided.name in HARD_RISKY) and (not stable):
          self._log(f"GUARD: unstable -> block {decided.name} -> standInit")
          decided = self.motions.standInit

      self._start(decided)
