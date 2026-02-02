"""
Blue Team Left Forward robot behaviours.
"""

import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils import Functions
from Utils.Consts import TIME_STEP


class ForwardLeft(SoccerRobot):

  def run(self):

    post_coordinate = [-4.86, -0.717, 0.0799]
    our_post = [4.86, 0.0]
    their_post = [-4.86, 0.0]

    flag1 = 0
    flag2 = 0
    goto_Coordinate = [0.0, 0.0, 0.0]
    useless_flag = 0
    count_0 = 0
    count_1 = 0

    while self.robot.step(TIME_STEP) != -1:

      if self.isNewBallDataAvailable():
        self.getSupervisorData()

        data = self.supervisorData
        ballOwner = self.getBallOwner()
        ballCoordinate = self.getBallData()          # [x,y]
        selfCoordinate = self.getSelfCoordinate()    # [x,y,z]

        # ============================================================
        # robots start at index 12 (24 doubles = 8 robots * 3)
        # order: RED_GK, RED_DEF_L, RED_DEF_R, RED_FW, BLUE_GK, BLUE_DEF, BLUE_FW_L, BLUE_FW_R
        # ============================================================
        base = 12

        def robot_xyz(i):
          j = base + i * 3
          return [float(data[j]), float(data[j + 1]), float(data[j + 2])]

        # BLUE TEAM
        blueGoalie = robot_xyz(4)
        blueDef = robot_xyz(5)
        blueForwardRight = robot_xyz(7)

        # RED TEAM
        redGoalie = robot_xyz(0)
        redDefLeft = robot_xyz(1)
        redDefRight = robot_xyz(2)
        redForward = robot_xyz(3)

        rightForward = blueForwardRight  # alias

        robots_positions = [
          blueGoalie[:2], blueDef[:2], selfCoordinate[:2], blueForwardRight[:2],
          redGoalie[:2], redDefLeft[:2], redDefRight[:2], redForward[:2]
        ]
        robot_names = ["BLUE_GK", "BLUE_DEF", "BLUE_FW_L", "BLUE_FW_R",
                       "RED_GK", "RED_DEF_L", "RED_DEF_R", "RED_FW"]

        team_blue = robots_positions[:4]
        red_team = robots_positions[4:]

        # ============================================================
        # ✅ FIX: distance calculation must use each robot (not selfCoordinate)
        # ============================================================
        robot_dist_from_ball = []
        for robot in robots_positions:
          distance = Functions.calculateDistance(ballCoordinate[:2], robot)
          robot_dist_from_ball.append(distance)

        team_blue_dist_from_ball = robot_dist_from_ball[:4]

        # Sort red by x descending (closest to our goal, since our goal x is +4.86)
        red_team_sorted = sorted(red_team, key=lambda r: r[0], reverse=True)

        # Find the closest team member to the ball (0..3 within team_blue)
        closest_rob = team_blue_dist_from_ball.index(min(team_blue_dist_from_ball))

        # Find ball owner position safely
        try:
          rob_number = robot_names.index(ballOwner)
          ball_owner_pos = robots_positions[rob_number]  # already 2D
          rob_dist_ball = robot_dist_from_ball[rob_number]
        except ValueError:
          rob_number = -1
          ball_owner_pos = [0.0, 0.0]
          rob_dist_ball = 999.0

        # ------------------------------------------------------------
        # Goal check
        # ------------------------------------------------------------
        if self.checkGoal() == 1:
          decidedMotion = self.motions.handWave
          if self.isNewMotionValid(decidedMotion):
            boolean = self.currentlyMoving and (
              self.currentlyMoving.name == self.motions.forwards50.name and
              decidedMotion.name != self.motions.forwards50.name
            )
            if boolean:
              self.interruptMotion()
            self.clearMotionQueue()
            if boolean:
              self.addMotionToQueue(self.motions.standInit)
            self.addMotionToQueue(decidedMotion)
          self.startMotion()
          continue

        elif self.checkGoal() == -1:
          decidedMotion = self.motions.standInit
          if self.isNewMotionValid(decidedMotion):
            boolean = self.currentlyMoving and (
              self.currentlyMoving.name == self.motions.forwards50.name and
              decidedMotion.name != self.motions.forwards50.name
            )
            if boolean:
              self.interruptMotion()
            self.clearMotionQueue()
            if boolean:
              self.addMotionToQueue(self.motions.standInit)
            self.addMotionToQueue(decidedMotion)
          self.startMotion()
          continue

        # ------------------------------------------------------------
        # Fall detection
        # ------------------------------------------------------------
        robotHeightFromGround = selfCoordinate[2]
        if robotHeightFromGround < 0.27:
          if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
            decidedMotion = self.motions.standUpFromBack
          else:
            decidedMotion = self.motions.standUpFromFront

          if self.isNewMotionValid(decidedMotion):
            if self.currentlyMoving and self.currentlyMoving.name != decidedMotion.name:
              boolean = self.currentlyMoving and (
                self.currentlyMoving.name == self.motions.forwards50.name and
                decidedMotion.name != self.motions.forwards50.name
              )
              if boolean:
                self.interruptMotion()
              self.clearMotionQueue()
              if boolean:
                self.addMotionToQueue(self.motions.standInit)
              self.addMotionToQueue(decidedMotion)
          self.startMotion()
          continue

        # ------------------------------------------------------------
        # Priority wait
        # ------------------------------------------------------------
        if self.getBallPriority() == "R":
          decidedMotion = self.motions.standInit
          if self.isNewMotionValid(decidedMotion):
            boolean = self.currentlyMoving and (
              self.currentlyMoving.name == self.motions.forwards50.name and
              decidedMotion.name != self.motions.forwards50.name
            )
            if boolean:
              self.interruptMotion()
            self.clearMotionQueue()
            if boolean:
              self.addMotionToQueue(self.motions.standInit)
            self.addMotionToQueue(decidedMotion)
          self.startMotion()
          continue

        # ============================================================
        # MAIN BEHAVIOUR
        # ============================================================
        if flag1 == 0:
          # opponent has the ball
          if ballOwner and ballOwner[0] == "R":

            if closest_rob == 2:
              # I am the closest blue robot -> chase ball
              decidedMotion, flag1 = self.decideMotion(ballCoordinate, selfCoordinate, post_coordinate)

              if self.isNewMotionValid(decidedMotion):
                boolean = self.currentlyMoving and (
                  self.currentlyMoving.name == self.motions.forwards50.name and
                  decidedMotion.name != self.motions.forwards50.name
                )
                if boolean:
                  self.interruptMotion()
                self.clearMotionQueue()
                if boolean:
                  self.addMotionToQueue(self.motions.standInit)
                self.addMotionToQueue(decidedMotion)
              self.startMotion()

            else:
              # Not closest -> mark an opponent (collaborative defense)
              if len(red_team_sorted) >= 2 and red_team_sorted[0] == ball_owner_pos:
                target_opponent = red_team_sorted[1]

                if target_opponent[0] < 0:
                  # Target inactive -> mark the ball owner instead, side-mark
                  target_opponent = red_team_sorted[0]
                  marking_distance = 0.5

                  # 90 deg clockwise mark vector (as in your code)
                  marking_vector_y = (our_post[0] - target_opponent[0]) * -1
                  marking_vector_x = (our_post[1] - target_opponent[1])

                  vector_dist = Functions.calculateDistance(our_post, target_opponent) + 1e-9
                  marking_point_x = (marking_vector_x / vector_dist) * marking_distance
                  marking_point_y = (marking_vector_y / vector_dist) * marking_distance

                  target_point = [target_opponent[0] + marking_point_x, target_opponent[1] + marking_point_y]

                  decidedMotion, flag1 = self.decideMotion(
                    target_point, selfCoordinate, post_coordinate, motion=self.motions.standInit
                  )

                else:
                  # Target active -> mark between opponent and our goal
                  marking_distance = 0.5
                  marking_vector_x = our_post[0] - target_opponent[0]
                  marking_vector_y = our_post[1] - target_opponent[1]

                  vector_dist = Functions.calculateDistance(our_post, target_opponent) + 1e-9
                  marking_point_x = (marking_vector_x / vector_dist) * marking_distance
                  marking_point_y = (marking_vector_y / vector_dist) * marking_distance

                  target_point = [target_opponent[0] + marking_point_x, target_opponent[1] + marking_point_y]

                  decidedMotion, flag1 = self.decideMotion(
                    target_point, selfCoordinate, post_coordinate,
                    motion=self.motions.standInit,
                    turn_location=target_opponent
                  )

                if self.isNewMotionValid(decidedMotion):
                  boolean = self.currentlyMoving and (
                    self.currentlyMoving.name == self.motions.forwards50.name and
                    decidedMotion.name != self.motions.forwards50.name
                  )
                  if boolean:
                    self.interruptMotion()
                  self.clearMotionQueue()
                  if boolean:
                    self.addMotionToQueue(self.motions.standInit)
                  self.addMotionToQueue(decidedMotion)
                self.startMotion()

          # teammate BLUE_FW_R has the ball -> support run / lane avoid
          elif ballOwner == "BLUE_FW_R":
            if rightForward[0] > -4.47 and rightForward[0] < 4.44 and rightForward[1] > 0 and rightForward[1] < 2.2:
              goto_Coordinate[0] = rightForward[0] - 0.5

              avoid_distance = 0.3
              for robot in red_team[1:]:  # ignore red GK
                # NOTE: this requires Functions.getDistanceFromLine to exist in your Utils/Functions.py
                obstruct = Functions.getDistanceFromLine(selfCoordinate[:2], their_post, robot)
                if obstruct < avoid_distance:
                  goto_Coordinate[1] = selfCoordinate[1]
                  break
                else:
                  goto_Coordinate[1] = their_post[1]

              goto_Coordinate[2] = 0.343

              decidedMotion, useless_flag = self.decideMotion(goto_Coordinate, selfCoordinate, post_coordinate)
              if self.isNewMotionValid(decidedMotion):
                boolean = self.currentlyMoving and (
                  self.currentlyMoving.name == self.motions.forwards50.name and
                  decidedMotion.name != self.motions.forwards50.name
                )
                if boolean:
                  self.interruptMotion()
                self.clearMotionQueue()
                if boolean:
                  self.addMotionToQueue(self.motions.standInit)
                self.addMotionToQueue(decidedMotion)
              self.startMotion()

          # defender or goalkeeper has ball -> sometimes chase
          elif (ballOwner == "BLUE_DEF" or ballOwner == "BLUE_GK"):
            if ballCoordinate[0] <= 2.52 or (-2.98 < ballCoordinate[1] < -2.4):
              decidedMotion, flag1 = self.decideMotion(ballCoordinate, selfCoordinate, post_coordinate)
              if self.isNewMotionValid(decidedMotion):
                boolean = self.currentlyMoving and (
                  self.currentlyMoving.name == self.motions.forwards50.name and
                  decidedMotion.name != self.motions.forwards50.name
                )
                if boolean:
                  self.interruptMotion()
                self.clearMotionQueue()
                if boolean:
                  self.addMotionToQueue(self.motions.standInit)
                self.addMotionToQueue(decidedMotion)
              self.startMotion()

            elif redForward[0] > 2.51 and redForward[1] < 0 and redForward[1] >= -2.5:
              goto_Coordinate[0] = 3.38
              goto_Coordinate[1] = -0.636
              goto_Coordinate[2] = 0.315

              decidedMotion, useless_flag = self.decideMotion(goto_Coordinate, selfCoordinate, post_coordinate)
              if self.isNewMotionValid(decidedMotion):
                boolean = self.currentlyMoving and (
                  self.currentlyMoving.name == self.motions.forwards50.name and
                  decidedMotion.name != self.motions.forwards50.name
                )
                if boolean:
                  self.interruptMotion()
                self.clearMotionQueue()
                if boolean:
                  self.addMotionToQueue(self.motions.standInit)
                self.addMotionToQueue(decidedMotion)
              self.startMotion()

            else:
              decidedMotion = self.turnMotion(blueDef, selfCoordinate)
              if self.isNewMotionValid(decidedMotion):
                boolean = self.currentlyMoving and (
                  self.currentlyMoving.name == self.motions.forwards50.name and
                  decidedMotion.name != self.motions.forwards50.name
                )
                if boolean:
                  self.interruptMotion()
                self.clearMotionQueue()
                if boolean:
                  self.addMotionToQueue(self.motions.standInit)
                self.addMotionToQueue(decidedMotion)
              self.startMotion()

          # nobody / self / other -> face goal post and pass/shoot
          else:
            decidedMotion, flag1 = self.turn_to_goal_post(post_coordinate, selfCoordinate, rightForward, redForward)

            if count_0 >= 2:
              decidedMotion = self.motions.rightShoot
              count_0 = 0
            if decidedMotion == self.motions.longShoot:
              count_0 += 1

            if self.isNewMotionValid(decidedMotion):
              boolean = self.currentlyMoving and (
                self.currentlyMoving.name == self.motions.forwards50.name and
                decidedMotion.name != self.motions.forwards50.name
              )
              if boolean:
                self.interruptMotion()
              self.clearMotionQueue()
              if boolean:
                self.addMotionToQueue(self.motions.standInit)
              self.addMotionToQueue(decidedMotion)
            self.startMotion()

      else:
        print("NO BALL DATA!!!")

  # ------------------------------------------------------------
  # Override decideMotion (simple chase/turn)
  # ------------------------------------------------------------
  def decideMotion(self, ballCoordinate, selfCoordinate, post_coordinate, motion=None, turn_location=None):

    robotHeadingAngle = self.getRollPitchYaw()[2]
    distanceFromBall = Functions.calculateDistance(ballCoordinate[:2], selfCoordinate[:2])

    if distanceFromBall < 0.22:
      if motion is None:
        return self.motions.handWave, 1
      else:
        return motion, 1

    if turn_location is None:
      turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate[:2], selfCoordinate[:2], robotHeadingAngle)
    else:
      turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(turn_location[:2], selfCoordinate[:2], robotHeadingAngle)

    if turningAngle > 50:
      return self.motions.turnLeft60, 0
    elif turningAngle > 30:
      return self.motions.turnLeft40, 0
    elif turningAngle < -50:
      return self.motions.turnRight60, 0
    elif turningAngle < -30:
      return self.motions.turnRight40, 0

    return self.motions.forwards50, 0

  # ------------------------------------------------------------
  def turn_to_goal_post(self, post_coordinate, selfCoordinate, rightForward, redForward):

    self.clearMotionQueue()
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(post_coordinate[:2], selfCoordinate[:2], robotHeadingAngle)

    # if red forward is very close -> pass to right
    if (
      ((redForward[0] >= (selfCoordinate[0] - 0.5) and redForward[0] < selfCoordinate[0]) or
       (redForward[1] >= (selfCoordinate[1] - 0.45) and redForward[1] < selfCoordinate[1]) or
       (redForward[0] <= (selfCoordinate[0] + 0.5) and redForward[0] > selfCoordinate[0]) or
       (redForward[1] <= (selfCoordinate[1] + 0.45) and redForward[1] > selfCoordinate[1]))
    ):
      return self.pass_to_right(selfCoordinate, rightForward)

    # otherwise align and shoot/pass
    if turningAngle > 30:
      return self.motions.rightSidePass, 0
    elif turningAngle < -30:
      return self.motions.leftSidePass, 0
    else:
      return self.motions.longShoot, 0

  def pass_to_right(self, selfCoordinate, rightForward):

    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(rightForward[:2], selfCoordinate[:2], robotHeadingAngle)

    if turningAngle > 50:
      return self.motions.rightSidePass, 0
    elif turningAngle < -50:
      return self.motions.leftSidePass, 0
    else:
      return self.motions.longShoot, 0

  def turnMotion(self, ballCoordinate, selfCoordinate):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate[:2], selfCoordinate[:2], robotHeadingAngle)

    if turningAngle > 90:
      return self.motions.turnLeft180
    elif turningAngle > 50:
      return self.motions.turnLeft60
    elif turningAngle > 30:
      return self.motions.turnLeft40
    elif turningAngle < -50:
      return self.motions.turnRight60
    elif turningAngle < -30:
      return self.motions.turnRight40

    return self.motions.standInit
