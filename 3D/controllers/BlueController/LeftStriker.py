
import os, sys
import math

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils.Consts import (TIME_STEP, Motions)
from Utils import Functions
import BlueTeamStrategies


DEBUG_STRIKER = True

#  Striker FSM state

STRIKER_FSM = {"state": "SEEK"}  # SEEK / GET_BEHIND / DRIBBLE / SHOOT / RECOVER


def _clamp(v, lo, hi):
  return max(lo, min(hi, v))


def _unit_vec(ax, ay, bx, by):
  dx, dy = (bx - ax), (by - ay)
  d = math.hypot(dx, dy) + 1e-9
  return dx / d, dy / d, d


def _field_bounds_from_play_zone():
  """
  Derive pitch bounds from BlueTeamStrategies.PLAY_ZONE to avoid hardcoding.
  PLAY_ZONE entries look like: {zone_id: [(x0,y0), (x1,y1)], ...}
  """
  xs = []
  ys = []
  for _, corners in BlueTeamStrategies.PLAY_ZONE.items():
    (x0, y0), (x1, y1) = corners[0], corners[1]
    xs.extend([x0, x1])
    ys.extend([y0, y1])
  return min(xs), max(xs), min(ys), max(ys)


class Forward (SoccerRobot):
  def run(self):

    while self.robot.step(TIME_STEP) != -1:

      if self.isNewBallDataAvailable():

        self.getSupervisorData()


        ballCoordinate = self.getBallData()
        selfCoordinate = self.getSelfCoordinate()

        decidedMotion = self.decideMotion(ballCoordinate, selfCoordinate)

        if self.isNewMotionValid(decidedMotion):

          forwardsSprintInterrupt = self.currentlyMoving and (
            self.currentlyMoving.name == self.motions.forwardsSprint.name and
            decidedMotion.name != self.motions.forwardsSprint.name
          )

          leftShootCheck = (
            self.currentlyMoving and
            self.currentlyMoving.name == self.motions.rightShoot.name and
            self.currentlyMoving.isOver() and
            decidedMotion.name == self.motions.rightShoot.name
          )

          self.clearMotionQueue()

          if leftShootCheck:
            self.addMotionToQueue(self.motions.shoot)
          else:
            self.addMotionToQueue(decidedMotion)

        self.startMotion()
      else:
        pass

  # Override decideMotion
  def decideMotion(self, ballCoordinate, selfCoordinate):
    """
    Port your 2D striker FSM to Webots 3D execution style:
      - keep original safety skeleton (goal / fall / ballPriority)
      - compute a target point based on FSM
      - execute with: turningMotion first, then walking motion, and kick when close & aligned
    """

    if self.checkGoal() == 1:
      return self.motions.handWave
    elif self.checkGoal() == -1:
      return self.motions.standInit

    # Fall Detection
    robotHeightFromGround = self.getSelfCoordinate()[2]
    if robotHeightFromGround < 0.2:
      if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
        return self.motions.standUpFromBack
      else:
        return self.motions.standUpFromFront

    # Check the opponent has ball priority.
    if self.getBallPriority() == "B":
      return self.motions.standInit

    # sonar 
    leftDistance = self.getLeftSonarValue()
    rightDistance = self.getRightSonarValue()

    robotHeadingAngle = self.getRollPitchYaw()[2]



    if not hasattr(self, "_debug_step"):
      self._debug_step = 0
    self._debug_step += 1


    # Read state and compute features
 
    global STRIKER_FSM

    bx, by = ballCoordinate[0], ballCoordinate[1]
    sx, sy = selfCoordinate[0], selfCoordinate[1]

    goal_mid = BlueTeamStrategies.BLUE_GOAL["Middle"]
    gx, gy = goal_mid[0], goal_mid[1]

    state = STRIKER_FSM["state"]

    # Field bounds derived from PLAY_ZONE (no hardcoding)
    x_min, x_max, y_min, y_max = _field_bounds_from_play_zone()

    y_margin = 0.55

    near_ball = 0.45
    contact_ball = 0.28
    far_ball = 1.20

    # corridor / geometry
    lateral_bad = 0.40
    behind_offset = 0.40

    # forward targets
    push_ahead = 0.90
    shoot_ahead = 1.30

    # shooting zone: ball within last X meters to opponent goal line (along x direction)
    shoot_zone_dist = 2.2

    # --- geometry: ball -> goal direction ---
    ux, uy, _ = _unit_vec(bx, by, gx, gy)

    # --- striker-ball distance ---
    dxs, dys = (bx - sx), (by - sy)
    ds = math.hypot(dxs, dys)

    # --- lateral error: perp = (-uy, ux) ---
    lateral_err = dxs * (-uy) + dys * (ux)

    in_shoot_zone = (gx - bx) < shoot_zone_dist


    ball_in_our_half = bx < 0.0

    # 
    # FSM transitions

    if state == "SHOOT":
      if (not in_shoot_zone) or ds > near_ball:
        state = "GET_BEHIND"

    elif state == "DRIBBLE":
      if ds > near_ball or abs(lateral_err) > lateral_bad:
        state = "GET_BEHIND"
      elif in_shoot_zone and ds < near_ball:
        state = "SHOOT"

    elif state == "GET_BEHIND":
      if ds < near_ball and abs(lateral_err) < (lateral_bad * 0.8):
        state = "DRIBBLE"
      elif ds > far_ball:
        state = "RECOVER"

    elif state == "RECOVER":
      if ds < far_ball:
        state = "GET_BEHIND"

    else:  # SEEK
      if ds > far_ball:
        state = "RECOVER"
      else:
        state = "GET_BEHIND"

    # bias: if ball is in our half and we are not close -> recover
    if ball_in_our_half and ds > near_ball:
      state = "RECOVER"

    STRIKER_FSM["state"] = state

    # debug typing
    if DEBUG_STRIKER and self._debug_step % 20 == 0:
      print(
        f"[STRIKER] "
        f"state={state:10s} | "
        f"ds={ds:4.2f} | "
        f"lat_err={lateral_err:5.2f} | "
        f"shoot_zone={int(in_shoot_zone)} | "
        f"ball=({bx:4.2f},{by:4.2f}) | "
        f"self=({sx:4.2f},{sy:4.2f})"
      )
 
    # Targets
    behind_x = bx - ux * behind_offset
    behind_y = by - uy * behind_offset

    dribble_x = bx + ux * push_ahead
    dribble_y = by + uy * push_ahead

    shoot_x = bx + ux * shoot_ahead
    shoot_y = gy  # aim towards goal center line

    # Clamp to field with margins to avoid edge-locking
    def clamp_xy(tx, ty):
      tx = _clamp(tx, x_min + 0.4, x_max - 0.4)
      ty = _clamp(ty, y_min + y_margin, y_max - y_margin)
      return [tx, ty]

    if state == "RECOVER":
      target = clamp_xy(bx, by)
      walk_mode = "sprint"
    elif state == "GET_BEHIND":
      target = clamp_xy(behind_x, behind_y)
      walk_mode = "sprint"
    elif state == "DRIBBLE":
      target = clamp_xy(dribble_x, dribble_y)
      walk_mode = "dribble"
    else:  # SHOOT
      target = clamp_xy(shoot_x, shoot_y)
      walk_mode = "sprint"


    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(
      target, selfCoordinate, robotHeadingAngle
    )
    turningMotion = self.getTurningMotion(turningAngle)
    if turningMotion is not None:
      return turningMotion

    # Close-range kick hook
    # Only when in SHOOT state, very close, and roughly aligned with goal center
    if state == "SHOOT" and ds < 0.30:
      goalAngle = Functions.calculateTurningAngleAccordingToRobotHeading(
        goal_mid, selfCoordinate, robotHeadingAngle
      )
      if abs(goalAngle) < 20:
        return self.motions.rightShoot

    # Obstacle avoidance 
    if self.obstacleAvoidance:
      # When close to the ball, avoid over-aggressive sidestep; otherwise keep same thresholds
      if leftDistance < 0.5:
        return self.motions.sideStepRight
      elif rightDistance < 0.5:
        return self.motions.sideStepLeft

    # Loop forwardsSprint motion if neede
    if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint" and self.currentlyMoving.getTime() == 1360:
      self.currentlyMoving.setTime(360)

    # Choose walking speed: dribble slower; contact -> slower still
    if walk_mode == "dribble":
      if ds < contact_ball:
        return self.motions.forwards50
      return self.motions.forwards

    return self.motions.forwardsSprint
