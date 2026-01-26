"""
Red Team Forward robot behaviours.
(FULL VERSION)
- Keeps your original decideMotion logic mostly intact
- Adds:
  1) motion logs (decision + start)
  2) NO-INTERRUPT locks for standup + kick/pass motions until finished
  3) short RECOVERY after kick/pass/turn so feet settle (standInit for N steps)
  4) stability check uses ONLY self z-height (reliable)  ✅ (NO roll/pitch guard)
  5) prevents forwardsSprint stop-start (keeps looping)
"""

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils.Consts import (TIME_STEP, Motions)
from Utils import Functions
import RedTeamStrategies


class Forward(SoccerRobot):

  # -------------------------
  # logging helpers
  # -------------------------
  def _mname(self, m):
    try:
      return m.name
    except Exception:
      return str(m)

  def _log(self, msg):
    print(f"[{self.robotName}] {msg}")

  # -------------------------
  # stability (use only height)
  # -------------------------
  def _is_stable_on_ground(self, selfCoord):
    # Tune this if needed: print your standing z and set threshold slightly below it
    z = float(selfCoord[2])
    if z < 0.28:
      return False, f"z_low z={z:.3f} (<0.28)"
    return True, f"OK z={z:.3f}"

  # -------------------------
  # main loop
  # -------------------------
  def run(self):

    # Motions that must NEVER be interrupted until they finish
    STANDUP_MOTIONS = {
      self.motions.standUpFromBack.name,
      self.motions.standUpFromFront.name,
    }

    # Motions that should not be interrupted (kicks / passes)
    KICK_MOTIONS = {
      self.motions.longShoot.name,
      self.motions.shoot.name,
      self.motions.rightSidePass.name,
      self.motions.leftSidePass.name,
      self.motions.shoot.name,   # you used this in original forward
    }

    # (Optional) treat turns as "actions" that should settle before next action
    TURN_MOTIONS = {
      self.motions.turnLeft40.name,
      self.motions.turnLeft60.name,
      self.motions.turnLeft180.name,
      self.motions.turnRight40.name,
      self.motions.turnRight60.name,
      # (your base maps right180 to left180 sometimes; keep safe)
    }

    # Recovery: after any action motion, do standInit for a few steps
    RECOVERY_AFTER_ACTION = 8  # tune 6–15
    recovery_steps = 0

    # Only log when decision changes (less spam)
    last_logged_decision = None

    while self.robot.step(TIME_STEP) != -1:

      if not self.isNewBallDataAvailable():
        continue

      # Required
      self.getSupervisorData()

      ballCoordinate = self.getBallData()
      selfCoordinate = self.getSelfCoordinate()
      ballDist = Functions.calculateDistance(ballCoordinate, selfCoordinate)

      # -----------------------------------
      # 0) If recovering, just stand still
      # -----------------------------------
      if recovery_steps > 0:
        recovery_steps -= 1
        self._log(f"RECOVERY standInit ({recovery_steps} left) dist={ballDist:.3f}")
        self.clearMotionQueue()
        self.addMotionToQueue(self.motions.standInit)
        self.startMotion()
        continue

      # -----------------------------------
      # 1) HARD LOCK stand-up until finished
      # -----------------------------------
      if self.currentlyMoving and (self.currentlyMoving.name in STANDUP_MOTIONS) and (not self.currentlyMoving.isOver()):
        self._log(f"LOCK standup (keep {self.currentlyMoving.name})")
        self.clearMotionQueue()
        self.addMotionToQueue(self.currentlyMoving)
        self.startMotion()
        continue

      # -----------------------------------
      # 2) HARD LOCK kick/pass until finished
      # -----------------------------------
      if self.currentlyMoving and (self.currentlyMoving.name in KICK_MOTIONS) and (not self.currentlyMoving.isOver()):
        self._log(f"LOCK kick (keep {self.currentlyMoving.name})")
        self.clearMotionQueue()
        self.addMotionToQueue(self.currentlyMoving)
        self.startMotion()
        continue

      # -----------------------------------
      # 3) Normal decision
      # -----------------------------------
      decidedMotion = self.decideMotion(ballCoordinate, selfCoordinate)

      if decidedMotion and decidedMotion.name != last_logged_decision:
        self._log(
          f"DECIDE {decidedMotion.name} "
          f"dist={ballDist:.3f} owner={self.getBallOwner()} prio={self.getBallPriority()} "
          f"current={(self.currentlyMoving.name if self.currentlyMoving else None)}"
        )
        last_logged_decision = decidedMotion.name

      # -----------------------------------
      # 4) After ANY motion except forwardsSprint:
      #    make sure "stable" before starting an action motion
      #    (uses only z-height; NOT roll/pitch)
      # -----------------------------------
      if decidedMotion:
        is_action = decidedMotion.name in (KICK_MOTIONS | TURN_MOTIONS)
        if is_action:
          stable, reason = self._is_stable_on_ground(selfCoordinate)
          self._log(f"GUARD_CHECK stable={stable} {reason}")
          if not stable:
            self._log("GUARD: not stable -> standInit")
            decidedMotion = self.motions.standInit

      # -----------------------------------
      # 5) Prevent forwardsSprint stop-start:
      #    If already sprinting and still want sprint -> keep it looping
      # -----------------------------------
      if (
        self.currentlyMoving
        and self.currentlyMoving.name == self.motions.forwardsSprint.name
        and decidedMotion
        and decidedMotion.name == self.motions.forwardsSprint.name
      ):
        # loop forward.motion
        try:
          if self.currentlyMoving.getTime() == 1360:
            self.currentlyMoving.setTime(360)
        except Exception:
          pass

        self.clearMotionQueue()
        self.addMotionToQueue(self.currentlyMoving)
        self.startMotion()
        continue

      # -----------------------------------
      # 6) Apply motion decision (your original style)
      # -----------------------------------
      if decidedMotion and self.isNewMotionValid(decidedMotion):

        # your old "leftShootCheck" idea, kept safe
        leftShootCheck = (
          self.currentlyMoving
          and self.currentlyMoving.name == self.motions.shoot.name
          and self.currentlyMoving.isOver()
          and decidedMotion.name == self.motions.shoot.name
        )

        self.clearMotionQueue()

        if leftShootCheck:
          self._log("QUEUE shoot (after shoot-over repeat)")
          self.addMotionToQueue(self.motions.shoot)
        else:
          self._log(f"QUEUE {decidedMotion.name}")
          self.addMotionToQueue(decidedMotion)

      # Start motion
      self.startMotion()
      if self.currentlyMoving:
        self._log(f"START_MOTION {self.currentlyMoving.name}")

      # -----------------------------------
      # 7) If we just STARTED an action motion,
      #    schedule recovery AFTER it finishes (not immediately).
      #
      # We do this by detecting: motion name is action AND it's not over now.
      # Next loop, it will be locked until over; after it becomes over,
      # we will set recovery_steps once (see below).
      # -----------------------------------

      # When an action motion finishes, trigger recovery
      if self.currentlyMoving and self.currentlyMoving.isOver():
        if self.currentlyMoving.name in (KICK_MOTIONS | TURN_MOTIONS):
          recovery_steps = RECOVERY_AFTER_ACTION
          self._log(f"ACTION_DONE {self.currentlyMoving.name} -> start RECOVERY {RECOVERY_AFTER_ACTION}")

  # ----------------------------------------------------
  # decision logic: ORIGINAL (UNCHANGED)
  # ----------------------------------------------------
  def decideMotion(self, ballCoordinate, selfCoordinate):

    # Check the goal scored to balance itself.
    if self.checkGoal() == 1:
      return self.motions.handWave
    elif self.checkGoal() == -1:
      return self.motions.standInit

    # Fall Detection
    robotHeightFromGround = selfCoordinate[2]
    if robotHeightFromGround < 0.2:
      if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
        return self.motions.standUpFromBack
      else:
        return self.motions.standUpFromFront

    # Check the opponent has ball priority.
    if self.getBallPriority() == "B":
      return self.motions.standInit

    # We are going to use these values to check if there is an obstacle in front of the robot.
    leftDistance = self.getLeftSonarValue()
    rightDistance = self.getRightSonarValue()

    robotHeadingAngle = self.getRollPitchYaw()[2]

    # If the ball on the opponent field.
    if RedTeamStrategies.getZone(ballCoordinate) > 9:

      # The ball on team member.
      if self.getBallOwner()[0] == "R" and self.getBallOwner() != "RED_FW":

        # Go to zone 14.
        if RedTeamStrategies.getZone(selfCoordinate) != 14:
          # Center of zone 14.
          zoneCenterX = (RedTeamStrategies.PLAY_ZONE[14][0][0] + RedTeamStrategies.PLAY_ZONE[14][1][0]) / 2
          zoneCenterY = (RedTeamStrategies.PLAY_ZONE[14][0][1] + RedTeamStrategies.PLAY_ZONE[14][1][1]) / 2
          # Find the angle between the target zone and robot heading.
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading([zoneCenterX, zoneCenterY], selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion

          # Check if there is an obstacle in front of the robot.
          if self.obstacleAvoidance:
            if leftDistance < 0.75:
              return self.motions.sideStepRight
            elif rightDistance < 0.75:
              return self.motions.sideStepLeft

          # loop forward.motion
          if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint":
            try:
              if self.currentlyMoving.getTime() == 1360:
                self.currentlyMoving.setTime(360)
            except Exception:
              pass

          return self.motions.forwardsSprint

        # Head to ball.
        else:
          turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
          turningMotion = self.getTurningMotion(turningAngle)
          if turningMotion is not None:
            return turningMotion

      # The ball on the opponent or on the robot itself.
      else:
        turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
        turningMotion = self.getTurningMotion(turningAngle)
        if turningMotion is not None:
          return turningMotion

        bodyDistanceFromBall = Functions.calculateDistance(ballCoordinate, selfCoordinate)

        # Decide where to shoot or pass.
        if bodyDistanceFromBall < 0.25:

          # If the robot at the 16th or 18th zones, the goal is 17th zone.
          if RedTeamStrategies.getZone(ballCoordinate) == 16 or RedTeamStrategies.getZone(ballCoordinate) == 18:
            turningAngleForGoalLeft = Functions.calculateTurningAngleAccordingToRobotHeading([RedTeamStrategies.PLAY_ZONE[17][0][0], 0], selfCoordinate, robotHeadingAngle)
            turningAngleForGoalRight = Functions.calculateTurningAngleAccordingToRobotHeading([RedTeamStrategies.PLAY_ZONE[17][1][0], 0], selfCoordinate, robotHeadingAngle)
          else:
            turningAngleForGoalLeft = Functions.calculateTurningAngleAccordingToRobotHeading(RedTeamStrategies.BLUE_GOAL["Left"], selfCoordinate, robotHeadingAngle)
            turningAngleForGoalRight = Functions.calculateTurningAngleAccordingToRobotHeading(RedTeamStrategies.BLUE_GOAL["Right"], selfCoordinate, robotHeadingAngle)

          if (turningAngleForGoalLeft > 0 and turningAngleForGoalRight > 0):
            if turningAngleForGoalLeft < 76 or turningAngleForGoalRight < 76:
              return self.motions.rightSidePass
            return self.motions.sideStepRight
          elif (turningAngleForGoalLeft < 0 and turningAngleForGoalRight < 0):
            if turningAngleForGoalLeft > -76 or turningAngleForGoalRight > -76:
              return self.motions.leftSidePass
            return self.motions.sideStepLeft
          elif (abs(turningAngleForGoalLeft) > 90 and abs(turningAngleForGoalRight) > 90):
            if abs(turningAngleForGoalLeft) > abs(turningAngleForGoalRight):
              return self.motions.sideStepLeft
            else:
              return self.motions.sideStepRight
          else:
            if bodyDistanceFromBall < 0.2:
              return self.motions.shoot
            else:
              return self.motions.forwardsSprint

        # Check if there is an obstacle in front of the robot.
        elif self.obstacleAvoidance and leftDistance < bodyDistanceFromBall:
          if leftDistance < 0.5:
            return self.motions.sideStepRight
          elif rightDistance < 0.5:
            return self.motions.sideStepLeft

        # loop forward.motion
        if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint":
          try:
            if self.currentlyMoving.getTime() == 1360:
              self.currentlyMoving.setTime(360)
          except Exception:
            pass

        return self.motions.forwardsSprint

    # The ball on team field.
    else:

      # Go to zone 11.
      if RedTeamStrategies.getZone(selfCoordinate) != 11:
        # Center of zone 11.
        zoneCenterX = (RedTeamStrategies.PLAY_ZONE[11][0][0] + RedTeamStrategies.PLAY_ZONE[11][1][0]) / 2
        zoneCenterY = (RedTeamStrategies.PLAY_ZONE[11][0][1] + RedTeamStrategies.PLAY_ZONE[11][1][1]) / 2
        turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading([zoneCenterX, zoneCenterY], selfCoordinate, robotHeadingAngle)
        turningMotion = self.getTurningMotion(turningAngle)
        if turningMotion is not None:
          return turningMotion

        if self.obstacleAvoidance:
          if leftDistance < 0.75:
            return self.motions.sideStepRight
          elif rightDistance < 0.75:
            return self.motions.sideStepLeft

        # loop forward.motion
        if self.currentlyMoving and self.currentlyMoving.name == "forwardsSprint":
          try:
            if self.currentlyMoving.getTime() == 1360:
              self.currentlyMoving.setTime(360)
          except Exception:
            pass

        return self.motions.forwardsSprint

      # Head to ball.
      else:
        turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
        turningMotion = self.getTurningMotion(turningAngle)
        if turningMotion is not None:
          return turningMotion

    # Stand by.
    return self.motions.standInit
