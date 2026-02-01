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

  # ✅ FIXED: Sidestep FIRST when |y_diff| <= 1 (no turning)
  def _track_ball_y(self, target_x, ball_y, selfCoordinate):
    sx, sy = selfCoordinate[0], selfCoordinate[1]

    y_clamp = 1.5
    y_target = max(-y_clamp, min(y_clamp, ball_y))
    y_diff = y_target - sy

    # IMPORTANT: If |y_diff| <= 1, do NOT turn. Just sidestep.
    if abs(y_diff) <= 1.0:
      return self.motions.sideStepRight if y_diff > 0 else self.motions.sideStepLeft

    # Only when far in Y do we consider turning + sprint
    robotHeadingAngle = self.getRollPitchYaw()[2]
    target = [target_x, y_target]

    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(target, selfCoordinate, robotHeadingAngle)
    turningMotion = self.getTurningMotion(turningAngle)
    if turningMotion is not None:
      return turningMotion

    self._loop_forwards_sprint()
    return self.motions.forwardsSprint

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
        # Kick if very close
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

    if selfCoordinate[2] < 0.2:
      if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
        return self.motions.standUpFromBack
      else:
        return self.motions.standUpFromFront

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
        return self._track_ball_y(standby_x, by, selfCoordinate)

      z13 = BlueTeamStrategies.PLAY_ZONE[13]
      guard_x = (z13[0][0] + z13[1][0]) / 2.0
      return self._track_ball_y(guard_x, by, selfCoordinate)

    # ======================================================
    # ✅ FIXED ENGAGE (ballZone >= 13)
    # - If close -> clear immediately (do NOT keep turning)
    # - If far  -> turn (only if needed) then sprint
    # ======================================================
    if ball_zone >= 13:
      dist = Functions.calculateDistance(ballCoordinate, selfCoordinate)

      # CLOSE: kick/clear NOW (don’t get stuck in turning)
      if dist < 0.25:
        return self._clear_like_forward(ballCoordinate, selfCoordinate)

      # FAR: orient + go
      robotHeadingAngle = self.getRollPitchYaw()[2]
      turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
      turningMotion = self.getTurningMotion(turningAngle)
      if turningMotion is not None:
        return turningMotion

      self._loop_forwards_sprint()
      return self.motions.forwardsSprint

    # ----------------------------
    # (2) ballZone < 13 -> your striker/strip rules
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
        robotHeadingAngle = self.getRollPitchYaw()[2]
        enter_y = max(-1.5, min(1.5, by))
        turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading([enter_x, enter_y], selfCoordinate, robotHeadingAngle)
        turningMotion = self.getTurningMotion(turningAngle)
        if turningMotion is not None:
          return turningMotion

        self._loop_forwards_sprint()
        return self.motions.forwardsSprint

      z11 = BlueTeamStrategies.PLAY_ZONE[11]
      standby_x = (z11[0][0] + z11[1][0]) / 2.0
      return self._track_ball_y(standby_x, by, selfCoordinate)

    if both_strikers_ge12:
      if self_zone < 13:
        self._log(f"BACK_TO_13: strikers>=10 selfZone={self_zone} -> retreat")
        return self.motions.backwards if hasattr(self.motions, "backwards") else self.motions.standInit

      z13 = BlueTeamStrategies.PLAY_ZONE[13]
      guard_x = (z13[0][0] + z13[1][0]) / 2.0
      return self._track_ball_y(guard_x, by, selfCoordinate)

    if at_least_one_ge10:
      if 10 <= self_zone <= 12:
        self._log(f"MIXED_RETREAT: fwLZ={fwL_zone} fwRZ={fwR_zone} selfZone={self_zone} -> retreat")
        return self.motions.backwards if hasattr(self.motions, "backwards") else self.motions.standInit

      z13 = BlueTeamStrategies.PLAY_ZONE[13]
      guard_x = (z13[0][0] + z13[1][0]) / 2.0
      return self._track_ball_y(guard_x, by, selfCoordinate)

    if self_zone < 13:
      return self.motions.backwards if hasattr(self.motions, "backwards") else self.motions.standInit

    z13 = BlueTeamStrategies.PLAY_ZONE[13]
    guard_x = (z13[0][0] + z13[1][0]) / 2.0
    return self._track_ball_y(guard_x, by, selfCoordinate)

  # -------------------------------------------------
  def run(self):
    while self.robot.step(TIME_STEP) != -1:

      if self.isNewBallDataAvailable():
        self.getSupervisorData()

        ballCoordinate = self.getBallData()
        selfCoordinate = self.getSelfCoordinate()

        decidedMotion = self.decideMotion(ballCoordinate, selfCoordinate)

        if self.isNewMotionValid(decidedMotion):
          self.clearMotionQueue()
          self.addMotionToQueue(decidedMotion)

        self.startMotion()
