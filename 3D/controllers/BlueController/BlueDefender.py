import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils.Consts import TIME_STEP
from Utils import Functions
import BlueTeamStrategies


class Defender(SoccerRobot):
  # ----------------------------
  # helper: crowd detection around ball (collision avoidance)
  # ----------------------------
  def _count_near_ball(self, ball_xy, robots_xy, r=0.55):
    bx, by = ball_xy[0], ball_xy[1]
    cnt = 0
    for p in robots_xy:
      dx = p[0] - bx
      dy = p[1] - by
      if (dx*dx + dy*dy) ** 0.5 <= r:
        cnt += 1
    return cnt

  def _ball_crowded_lock(self, ball_xy, robots_xy,
                         enter_r=0.55, exit_r=0.70,
                         threshold=3):
    if not hasattr(self, "_crowd_locked"):
      self._crowd_locked = False

    if self._crowd_locked:
      cnt = self._count_near_ball(ball_xy, robots_xy, r=exit_r)
      if cnt < threshold:
        self._crowd_locked = False
      return self._crowd_locked, cnt

    cnt = self._count_near_ball(ball_xy, robots_xy, r=enter_r)
    if cnt >= threshold:
      self._crowd_locked = True
    return self._crowd_locked, cnt

  def _log(self, msg):
    print(f"[BLUE_DEF] {msg}")

  def _zone(self, xy_or_xyz):
    return BlueTeamStrategies.getZone(xy_or_xyz)

  def _loop_forwards_sprint(self):
    if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint" and self.currentlyMoving.getTime() == 1360:
      self.currentlyMoving.setTime(360)

  # ==========================================================
  # Stand-up helpers (FIX: avoid double-trigger of standUpFromFront)
  # ==========================================================
  def _is_standup_motion(self, m):
    return (m is not False) and (m is not None) and hasattr(m, "name") and ("standUp" in m.name or "StandUp" in m.name)

  def _standup_tick(self):
    if not hasattr(self, "_standup_cooldown"):
      self._standup_cooldown = 0
    if self._standup_cooldown > 0:
      self._standup_cooldown -= 1

  # ==========================================================
  # ADDED: Face-ball gate (run this before any other action)
  # ==========================================================
  def _face_ball_first(self, ballCoordinate, selfCoordinate,
                       face_th=12,
                       hard_th=90):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(
      ballCoordinate, selfCoordinate, robotHeadingAngle
    )

    if abs(turningAngle) >= hard_th:
      return self.getTurningMotion(turningAngle)

    if abs(turningAngle) <= face_th:
      return None

    return self.getTurningMotion(turningAngle)

  # ----------------------------------------------------------
  # TRACK BALL Y (now ALSO faces ball first, per your request)
  # ----------------------------------------------------------
  def _track_ball_y(self, target_x, ball_y, ballCoordinate, selfCoordinate):
    sx, sy = selfCoordinate[0], selfCoordinate[1]

    tm_ball = self._face_ball_first(ballCoordinate, selfCoordinate, face_th=12, hard_th=90)
    if tm_ball is not None:
      return tm_ball

    y_clamp = 1.5
    y_target = max(-y_clamp, min(y_clamp, ball_y))
    y_diff = y_target - sy

    if abs(y_diff) <= 1.0:
      return self.motions.sideStepRight if y_diff > 0 else self.motions.sideStepLeft

    self._loop_forwards_sprint()
    return self.motions.forwardsSprint

  # ----------------------------
  # helper: choose clear/shoot like RedForward (mirrored to RED goal)
  # ----------------------------
  def _clear_like_forward(self, ballCoordinate, selfCoordinate):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    bodyDistanceFromBall = Functions.calculateDistance(ballCoordinate, selfCoordinate)

    if bodyDistanceFromBall < 0.25:
      turningAngleForGoalLeft  = Functions.calculateTurningAngleAccordingToRobotHeading(
        BlueTeamStrategies.RED_GOAL["Left"], selfCoordinate, robotHeadingAngle
      )
      turningAngleForGoalRight = Functions.calculateTurningAngleAccordingToRobotHeading(
        BlueTeamStrategies.RED_GOAL["Right"], selfCoordinate, robotHeadingAngle
      )

      if (turningAngleForGoalLeft > 0 and turningAngleForGoalRight > 0):
        if turningAngleForGoalLeft < 76 or turningAngleForGoalRight < 76:
          return self.motions.rightSidePass if hasattr(self.motions, "rightSidePass") else self.motions.shoot
        return self.motions.sideStepRight

      elif (turningAngleForGoalLeft < 0 and turningAngleForGoalRight < 0):
        if turningAngleForGoalLeft > -76 or turningAngleForGoalRight > -76:
          return self.motions.leftSidePass if hasattr(self.motions, "leftSidePass") else self.motions.shoot
        return self.motions.sideStepLeft

      elif (abs(turningAngleForGoalLeft) > 90 and abs(turningAngleForGoalRight) > 90):
        if abs(turningAngleForGoalLeft) > abs(turningAngleForGoalRight):
          return self.motions.sideStepLeft
        else:
          return self.motions.sideStepRight

      else:
        if bodyDistanceFromBall < 0.2:
          if hasattr(self.motions, "rightShoot"):
            return self.motions.rightShoot
          return self.motions.shoot
        else:
          self._loop_forwards_sprint()
          return self.motions.forwardsSprint

    self._loop_forwards_sprint()
    return self.motions.forwardsSprint

  # -------------------------------------------------
  def decideMotion(self, ballCoordinate, selfCoordinate):

    if self.checkGoal() == 1:
      return self.motions.handWave
    elif self.checkGoal() == -1:
      return self.motions.standInit

    # ----------------------------
    # STAND-UP (fixed: prevent double-trigger)
    # ----------------------------
    # If a stand-up motion is currently playing, let it finish.
    if self._is_standup_motion(self.currentlyMoving) and not self.currentlyMoving.isOver():
      return self.currentlyMoving

    # After stand-up, wait a little so GPS/pose settles.
    if getattr(self, "_standup_cooldown", 0) > 0:
      return self.motions.standInit

    # Detect fallen (keep your z check, but require IMU tilt too)
    roll, pitch, _ = self.getRollPitchYaw()
    fallen = (selfCoordinate[2] < 0.2) and (abs(roll) > 0.6 or abs(pitch) > 0.6)

    if fallen:
      # Lock stand-up choice so it doesn't flip between front/back
      if not hasattr(self, "_standup_choice"):
        if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
          self._standup_choice = self.motions.standUpFromBack
        else:
          self._standup_choice = self.motions.standUpFromFront

      # Start stand-up; then set cooldown to prevent immediate re-trigger
      self._standup_cooldown = int(1200 / TIME_STEP)
      return self._standup_choice

    # If upright, clear locked standup choice (so next fall re-evaluates)
    if hasattr(self, "_standup_choice"):
      delattr(self, "_standup_choice")

    if self.getBallPriority() == "R":
      return self.motions.standInit

    bx, by = ballCoordinate[0], ballCoordinate[1]
    sx, sy = selfCoordinate[0], selfCoordinate[1]

    data = getattr(self, "supervisorData", None) or []
    fwL = [data[30], data[31]] if len(data) > 31 else [sx, sy]
    fwR = [data[33], data[34]] if len(data) > 34 else [sx, sy]

    redDefL = [data[15], data[16]] if len(data) > 16 else [sx, sy]
    redDefR = [data[18], data[19]] if len(data) > 19 else [sx, sy]
    redFw   = [data[21], data[22]] if len(data) > 22 else [sx, sy]

    ball_zone = self._zone([bx, by])
    self_zone = self._zone([sx, sy])
    fwL_zone  = self._zone(fwL)
    fwR_zone  = self._zone(fwR)

    # ==========================================================
    # ALWAYS face ball first before any other logic
    # ==========================================================
    tm0 = self._face_ball_first(ballCoordinate, selfCoordinate, face_th=12, hard_th=90)
    if tm0 is not None:
      return tm0

    # ----------------------------
    # COLLISION AVOIDANCE
    # ----------------------------
    near_robots = [fwL, fwR, redDefL, redDefR, redFw]
    crowded, crowd_cnt = self._ball_crowded_lock(
      ball_xy=[bx, by],
      robots_xy=near_robots,
      enter_r=0.55,
      exit_r=0.70,
      threshold=3
    )

    if crowded:
      self._log(f"CROWD_BLOCK: count={crowd_cnt} near ball -> hold position (no join)")
      if 10 <= self_zone <= 12:
        z11 = BlueTeamStrategies.PLAY_ZONE[11]
        standby_x = (z11[0][0] + z11[1][0]) / 2.0
        return self._track_ball_y(standby_x, by, ballCoordinate, selfCoordinate)

      z13 = BlueTeamStrategies.PLAY_ZONE[13]
      guard_x = (z13[0][0] + z13[1][0]) / 2.0
      return self._track_ball_y(guard_x, by, ballCoordinate, selfCoordinate)

    # ======================================================
    # ballZone >= 13 : chase + clear
    # ======================================================
    if ball_zone >= 13:
      dist = Functions.calculateDistance(ballCoordinate, selfCoordinate)

      if dist < 0.25:
        return self._clear_like_forward(ballCoordinate, selfCoordinate)

      self._loop_forwards_sprint()
      return self.motions.forwardsSprint

    # ----------------------------
    # ballZone < 13 -> striker/strip rules
    # ----------------------------
    both_strikers_opponent = (fwL_zone < 10 and fwR_zone < 10)
    both_strikers_ge12     = (fwL_zone >= 10 and fwR_zone >= 10)
    at_least_one_ge10      = (fwL_zone >= 10 or fwR_zone >= 10)

    if both_strikers_opponent and ball_zone < 13:
      x_min_strip = BlueTeamStrategies.PLAY_ZONE[10][0][0]  # 0.00
      enter_margin = 0.05
      enter_x = x_min_strip + enter_margin

      self._log(f"ENTER_STRIP: ballZ={ball_zone} fwLZ={fwL_zone} fwRZ={fwR_zone} selfX={sx:.2f} -> x<=0.00")

      if sx > enter_x:
        self._loop_forwards_sprint()
        return self.motions.forwardsSprint

      z11 = BlueTeamStrategies.PLAY_ZONE[11]
      standby_x = (z11[0][0] + z11[1][0]) / 2.0
      return self._track_ball_y(standby_x, by, ballCoordinate, selfCoordinate)

    if both_strikers_ge12:
      if self_zone < 13:
        self._log(f"BACK_TO_13: strikers>=10 selfZone={self_zone} -> retreat")
        return self.motions.backwards if hasattr(self.motions, "backwards") else self.motions.standInit

      z13 = BlueTeamStrategies.PLAY_ZONE[13]
      guard_x = (z13[0][0] + z13[1][0]) / 2.0
      return self._track_ball_y(guard_x, by, ballCoordinate, selfCoordinate)

    if at_least_one_ge10:
      if 10 <= self_zone <= 12:
        self._log(f"MIXED_RETREAT: fwLZ={fwL_zone} fwRZ={fwR_zone} selfZone={self_zone} -> retreat")
        return self.motions.backwards if hasattr(self.motions, "backwards") else self.motions.standInit

      z13 = BlueTeamStrategies.PLAY_ZONE[13]
      guard_x = (z13[0][0] + z13[1][0]) / 2.0
      return self._track_ball_y(guard_x, by, ballCoordinate, selfCoordinate)

    if self_zone < 13:
      return self.motions.backwards if hasattr(self.motions, "backwards") else self.motions.standInit

    z13 = BlueTeamStrategies.PLAY_ZONE[13]
    guard_x = (z13[0][0] + z13[1][0]) / 2.0
    return self._track_ball_y(guard_x, by, ballCoordinate, selfCoordinate)

  # -------------------------------------------------
  def run(self):
    while self.robot.step(TIME_STEP) != -1:
      # cooldown tick every step
      self._standup_tick()

      if self.isNewBallDataAvailable():
        self.getSupervisorData()

        if hasattr(self, "runCollisionAvoidanceStep") and self.runCollisionAvoidanceStep():
          print(f"[{self.name}] AVOID_ACTIVE cooldown={getattr(self, 'avoidanceCooldown', -1)}")
          continue

        ballCoordinate = self.getBallData()
        selfCoordinate = self.getSelfCoordinate()

        decidedMotion = self.decideMotion(ballCoordinate, selfCoordinate)

        if self.isNewMotionValid(decidedMotion):
          self.clearMotionQueue()
          self.addMotionToQueue(decidedMotion)

        self.startMotion()
