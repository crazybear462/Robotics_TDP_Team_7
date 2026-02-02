"""
Blue Team Left Forward robot behaviours.
"""

import os, sys

from Utils.Functions import calculateDistance

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils import Functions
from Utils.Consts import (TIME_STEP, Motions)


class ForwardLeft (SoccerRobot):
  
  def run(self):

    post_coordinate = [-4.86,-0.717,0.0799]
    our_post = [4.86, 0]
    their_post = [-4.86, 0]
    flag1=0
    flag2=0
    goto_Coordinate=[0,0,0]
    useless_flag=0
    count_0=0
    count_1=0
    while self.robot.step(TIME_STEP) != -1:
      print(f"--- THE BALL OWNER IS ---\n", flush=True)
      if self.isNewBallDataAvailable():
        self.getSupervisorData()
        # Use the ballData (location) to do something.
        data = self.supervisorData
        ballOwner = self.getBallOwner()
        ballCoordinate = self.getBallData()
        # print("RedForward - ballCoordinate: ", ballCoordinate)
        selfCoordinate = self.getSelfCoordinate()
        # rightForward = [data[33],data[34],data[35]]
        # redForward = [data[21],data[22],data[23]]
        # blueDef = [data[27],data[28],data[29]]
        # 2. DEFINE BLUE TEAM
        # Note: You are BLUE_FW_L (data 22-24)
        blueGoalie = [data[16], data[17], data[18]]
        blueDef = [data[19], data[20], data[21]]
        blueForwardRight = [data[25], data[26], data[27]]

        # 3. DEFINE RED TEAM
        redGoalie = [data[4], data[5], data[6]]
        redDefLeft = [data[7], data[8], data[9]]
        redDefRight = [data[10], data[11], data[12]]
        redForward = [data[13], data[14], data[15]]

        rightForward = [data[25], data[26], data[27]]
        blueDef = [data[19], data[20], data[21]]

        robots_positions = [blueGoalie[:2], blueDef[:2], selfCoordinate[:2], blueForwardRight[:2],
                            redGoalie[:2], redDefLeft[:2], redDefRight[:2], redForward[:2]]
        robot_names = ["BLUE_GK", "BLUE_DEF", "BLUE_FW_L","BLUE_FW_R", "RED_GK", "RED_DEF_L", "RED_DEF_R", "RED_FW"]

        team_blue = [blueGoalie[:2],blueDef[:2], selfCoordinate[:2], blueForwardRight[:2]]
        red_team = robots_positions[4:]

        # All Robots distances from the ball
        robot_dist_from_ball = []
        for robot in robots_positions:
            distance = Functions.calculateDistance(ballCoordinate[:2], robot)
            robot_dist_from_ball.append(distance)
        # Team Blue distances from the ball
        team_blue_dist_from_ball = robot_dist_from_ball[:4]


        # Sorts by index 0 (x) in descending order (highest to lowest)
        # This shows the closest robot to our goal
        red_team_sorted = sorted(red_team, key=lambda robot: robot[0], reverse=True)

        # Find the closest team member to the ball
        closest_rob = team_blue_dist_from_ball.index(min(team_blue_dist_from_ball)) # index of closest robot
                                                                                    # (in team blue list)
        try: # This is for Both Red and Blue
            rob_number = robot_names.index(ballOwner)  # Return the index of the robot with the ball

            ball_owner_pos = robots_positions[rob_number]  # Returns the position of the robot on the field
            rob_dist_ball = robot_dist_from_ball[rob_number]  # That robots distance from the ball
        except ValueError:
            # If ballOwner is not in our list (e.g. "NONE" or glitch), set defaults
            rob_number = -1
            ball_owner_pos = [0, 0]
            rob_dist_ball = 999


        print(ballOwner)

        # Check the goal scored to balance itself.
        if self.checkGoal() == 1:
          decidedMotion =  self.motions.handWave

          if self.isNewMotionValid(decidedMotion):
              boolean = self.currentlyMoving and\
                  (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
              if boolean:
                  self.interruptMotion()
              self.clearMotionQueue()
              if boolean:
                  self.addMotionToQueue(self.motions.standInit)
              self.addMotionToQueue(decidedMotion)

          self.startMotion()
          
        elif self.checkGoal() == -1:
          decidedMotion =  self.motions.standInit

          if self.isNewMotionValid(decidedMotion):
              boolean = self.currentlyMoving and\
                  (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
              if boolean:
                  self.interruptMotion()
              self.clearMotionQueue()
              if boolean:
                  self.addMotionToQueue(self.motions.standInit)
              self.addMotionToQueue(decidedMotion)

          self.startMotion()

        # Check whether the robot falls down.
        robotHeightFromGround = selfCoordinate[2]

        if robotHeightFromGround < 0.27:
            if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
                decidedMotion = self.motions.standUpFromBack
            else:
                decidedMotion = self.motions.standUpFromFront
            if self.isNewMotionValid(decidedMotion):
                if self.currentlyMoving.name != decidedMotion.name:
                    boolean = self.currentlyMoving and \
                          (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
                    if boolean:
                        self.interruptMotion()
                    self.clearMotionQueue()
                    if boolean:
                        self.addMotionToQueue(self.motions.standInit)
                    self.addMotionToQueue(decidedMotion)

            self.startMotion()

        # Check the opponent has ball priority.
        elif self.getBallPriority() == "R":
          decidedMotion = self.motions.standInit
          # Wait for Red team to start the game.

          if self.isNewMotionValid(decidedMotion):
              boolean = self.currentlyMoving and\
                  (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
              if boolean:
                  self.interruptMotion()
              self.clearMotionQueue()
              if boolean:
                  self.addMotionToQueue(self.motions.standInit)
              self.addMotionToQueue(decidedMotion)

          self.startMotion()

        else:

            if flag1 == 0: # flag1 = 0 means I am not within shooting distance from ball
                # Check if the opposition has the ball
                if ballOwner[0] == 'R':  # R means red robot i.e Opposition

                    if closest_rob == 2: # If I am the closest robot to the ball
                        # Chase the ball
                        decidedMotion, flag1 = self.decideMotion(ballCoordinate, selfCoordinate, post_coordinate)

                        if self.isNewMotionValid(decidedMotion):
                            boolean = self.currentlyMoving and \
                              (self.currentlyMoving.name == self.motions.forwards50.name and
                               decidedMotion.name != self.motions.forwards50.name)
                            if boolean:
                                self.interruptMotion()
                            self.clearMotionQueue()
                            if boolean:
                                self.addMotionToQueue(self.motions.standInit)
                            self.addMotionToQueue(decidedMotion)
                        self.startMotion()

                    else: # I am not the closest robot. Let me get in position to stop an attack
                        near_red_to_goal = max(robot[0] for robot in red_team)
                        if red_team_sorted[0] == ball_owner_pos: # Rob_Pos is the opponent here
                            # This means the closest player to the goal (opp) is also closest to the ball
                            # I'll mark someone I am closer to
                            # Slight issue, just because I am not the closest to the ball
                            # Does not mean I am not the closest to the player chasing the ball (I think)
                            # It's not a big issue though, I'm still unlikely to get the ball, so I must be useful

                            target_opponent = red_team_sorted[1] # This is who I will mark
                            print("hello there")
                            if target_opponent[0] < 0:# Opponent is still in his defensive half
                                print("Target is inactive, commence double team")
                                target_opponent = red_team_sorted[0]
                                marking_distance = 0.5  # How close the striker should be to opponent
                                marking_vector_y = (our_post[0] - target_opponent[0]) * -1 # Swiched the vectors to mark by the
                                marking_vector_x = our_post[1] - target_opponent[1] # side of the opponent (90 deg clockwise)
                                vector_dist = Functions.calculateDistance(our_post, target_opponent)
                                marking_point_x = (marking_vector_x / vector_dist) * marking_distance
                                marking_point_y = (marking_vector_y / vector_dist) * marking_distance
                                target_point = [target_opponent[0] + marking_point_x,
                                                target_opponent[1] + marking_point_y]
                                decidedMotion, flag1 = self.decideMotion(target_point, selfCoordinate,
                                                                         post_coordinate, motion=self.motions.standInit)
                                if self.isNewMotionValid(decidedMotion):
                                    boolean = self.currentlyMoving and \
                                              (self.currentlyMoving.name == self.motions.forwards50.name and
                                               decidedMotion.name != self.motions.forwards50.name)
                                    if boolean:
                                        self.interruptMotion()
                                    self.clearMotionQueue()
                                    if boolean:
                                        self.addMotionToQueue(self.motions.standInit)
                                    self.addMotionToQueue(decidedMotion)
                                self.startMotion()


                            else:  # He crossed the halfway line
                                print("Target is active")
                                marking_distance = 0.5 # How close the striker should be to opponent
                                marking_vector_x = our_post[0] - target_opponent[0]
                                marking_vector_y = our_post[1] - target_opponent[1]
                                vector_dist = Functions.calculateDistance(our_post, target_opponent)
                                marking_point_x = (marking_vector_x/vector_dist) * marking_distance
                                marking_point_y = (marking_vector_y/vector_dist) * marking_distance
                                # All these lines plots a vector that is 0.5 meters in length and is pointing towards goal
                                target_point = [target_opponent[0]+marking_point_x, target_opponent[1]+marking_point_y]
                                decidedMotion, flag1 = self.decideMotion(target_point, selfCoordinate,
                                                                         post_coordinate, motion=self.motions.standInit,
                                                                         turn_location=target_opponent)
                                if self.isNewMotionValid(decidedMotion):
                                    boolean = self.currentlyMoving and \
                                              (self.currentlyMoving.name == self.motions.forwards50.name and
                                               decidedMotion.name != self.motions.forwards50.name)
                                    if boolean:
                                        self.interruptMotion()
                                    self.clearMotionQueue()
                                    if boolean:
                                        self.addMotionToQueue(self.motions.standInit)
                                    self.addMotionToQueue(decidedMotion)
                                self.startMotion()


                elif ballOwner=='BLUE_FW_R':
                    if rightForward[0]>-4.47 and rightForward[0]<4.44 and rightForward[1]>0 and rightForward[1]<2.2:
                        goto_Coordinate[0]= rightForward[0] - 0.5
                        # Logic to avoid other players
                        avoid_distance = 0.3  # How far the opponents should be from me

                        obstructor = False
                        obstructor_position = 0
                        for robot in red_team[1:]: # Goalkeeper not included
                            obstruct = Functions.getDistanceFromLine(selfCoordinate[:2],their_post, robot)
                            if obstruct < avoid_distance: # If the robot is within 3m to my path
                                obstructor = True
                                obstructor_position = robot
                                goto_Coordinate[1] = selfCoordinate[1]
                                break # Just worry about one robot at a time
                            else:
                                goto_Coordinate[1] = their_post[1]


                        goto_Coordinate[2] = 0.343
                        decidedMotion, useless_flag= self.decideMotion(goto_Coordinate, selfCoordinate, post_coordinate)
                        if self.isNewMotionValid(decidedMotion):
                            boolean = self.currentlyMoving and\
                                (self.currentlyMoving.name == self.motions.forwards50.name and
                                decidedMotion.name != self.motions.forwards50.name)
                            if boolean:
                                self.interruptMotion()
                            self.clearMotionQueue()
                            if boolean:
                                self.addMotionToQueue(self.motions.standInit)
                            self.addMotionToQueue(decidedMotion)

                        self.startMotion()

                elif rightForward[0]>-4.47 and rightForward[0]<=0.268 and rightForward[1]<-1.5 and rightForward[1]>-2.96:
                    goto_Coordinate[0]= rightForward[0] + 1.5
                    goto_Coordinate[1] = rightForward[0] - 1
                    goto_Coordinate[2] = 0.343
                    decidedMotion, useless_flag= self.decideMotion(goto_Coordinate, selfCoordinate, post_coordinate)
                    if self.isNewMotionValid(decidedMotion):
                        boolean = self.currentlyMoving and\
                          (self.currentlyMoving.name == self.motions.forwards50.name and
                           decidedMotion.name != self.motions.forwards50.name)
                        if boolean:
                            self.interruptMotion()
                        self.clearMotionQueue()
                        if boolean:
                            self.addMotionToQueue(self.motions.standInit)
                        self.addMotionToQueue(decidedMotion)

                    self.startMotion()

                else:
                    goto_Coordinate[0]= rightForward[0] - 1
                    goto_Coordinate[1] = rightForward[0] - 1
                    goto_Coordinate[2] = 0.343
                    decidedMotion, useless_flag= self.decideMotion(goto_Coordinate, selfCoordinate, post_coordinate)
                    if self.isNewMotionValid(decidedMotion):
                        boolean = self.currentlyMoving and\
                          (self.currentlyMoving.name == self.motions.forwards50.name and
                           decidedMotion.name != self.motions.forwards50.name)
                        if boolean:
                            self.interruptMotion()
                        self.clearMotionQueue()
                        if boolean:
                            self.addMotionToQueue(self.motions.standInit)
                        self.addMotionToQueue(decidedMotion)
                    self.startMotion()

            elif (ballOwner=='BLUE_DEF' or ballOwner=='BLUE_GK'):
                if ballCoordinate[0]<=2.52 or (ballCoordinate[1]<-2.4 and ballCoordinate[1]>-2.98):
                    decidedMotion, flag1 = self.decideMotion(ballCoordinate, selfCoordinate, post_coordinate)
                    if self.isNewMotionValid(decidedMotion):
                        boolean = self.currentlyMoving and\
                          (self.currentlyMoving.name == self.motions.forwards50.name and
                           decidedMotion.name != self.motions.forwards50.name)
                        if boolean:
                            self.interruptMotion()
                        self.clearMotionQueue()
                        if boolean:
                            self.addMotionToQueue(self.motions.standInit)
                        self.addMotionToQueue(decidedMotion)

                    self.startMotion()

                elif redForward[0]>2.51 and redForward[1]<0 and redForward[1]>=-2.5:
                    goto_Coordinate[0]=3.38
                    goto_Coordinate[1]=-0.636
                    goto_Coordinate[2]=0.315
                    decidedMotion, useless_flag= self.decideMotion(goto_Coordinate, selfCoordinate, post_coordinate)
                    if self.isNewMotionValid(decidedMotion):
                        boolean = self.currentlyMoving and\
                          (self.currentlyMoving.name == self.motions.forwards50.name and
                           decidedMotion.name != self.motions.forwards50.name)
                        if boolean:
                            self.interruptMotion()
                        self.clearMotionQueue()
                        if boolean:
                            self.addMotionToQueue(self.motions.standInit)
                        self.addMotionToQueue(decidedMotion)

                    self.startMotion()

                else:
                  decidedMotion= self.turnMotion(blueDef,selfCoordinate)
                  if self.isNewMotionValid(decidedMotion):
                      boolean = self.currentlyMoving and\
                        (self.currentlyMoving.name == self.motions.forwards50.name and decidedMotion.name != self.motions.forwards50.name)
                      if boolean:
                          self.interruptMotion()
                      self.clearMotionQueue()
                      if boolean:
                          self.addMotionToQueue(self.motions.standInit)
                      self.addMotionToQueue(decidedMotion)

                  self.startMotion()

            else:
                decidedMotion, flag1 = self.turn_to_goal_post(post_coordinate, selfCoordinate,rightForward,redForward)
                if count_0>=2:
                    decidedMotion=self.motions.rightShoot
                    count_0=0
                if decidedMotion ==  self.motions.longShoot:
                    count_0=count_0+1
                if self.isNewMotionValid(decidedMotion):
                    boolean = self.currentlyMoving and\
                    (self.currentlyMoving.name == self.motions.forwards50.name and
                     decidedMotion.name != self.motions.forwards50.name)
                    if boolean:
                        self.interruptMotion()
                    self.clearMotionQueue()
                    if boolean:
                        self.addMotionToQueue(self.motions.standInit)
                    self.addMotionToQueue(decidedMotion)

              #self.addMotionToQueue(decidedMotion)

                self.startMotion()

      else:
        print("NO BALL DATA!!!")

  # Override decideMotion
  def decideMotion(self, ballCoordinate, selfCoordinate, post_coordinate, motion=None, turn_location=None):
    
    robotHeadingAngle = self.getRollPitchYaw()[2]
    distanceFromBall = Functions.calculateDistance(ballCoordinate, selfCoordinate)

    if distanceFromBall < 0.22:
      if motion is None:
        return self.motions.handWave,1
      else:
        return motion, 1

    if turn_location is None:
        turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
    else:
        turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(turn_location, selfCoordinate,
                                                                              robotHeadingAngle)
    if turningAngle > 50:
      return self.motions.turnLeft60,0
    elif turningAngle > 30:
      return self.motions.turnLeft40,0
    # elif turningAngle >= 10:
    #   return self.motions.turnLeft10,0
    elif turningAngle < -50:
      return self.motions.turnRight60,0
    elif turningAngle < -30:
      return self.motions.turnRight40,0
    # elif turningAngle < -20:
    #   return self.motions.turnRight10,0

    return self.motions.forwards50,0
    
  def turn_to_goal_post(self, post_coordinate, selfCoordinate,rightForward,redForward):
    
    self.clearMotionQueue()
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(post_coordinate, selfCoordinate, robotHeadingAngle)
    if ((redForward[0] >=(selfCoordinate[0]-0.5) and redForward[0] < selfCoordinate[0]) or (redForward[1] >= (selfCoordinate[1]-0.45) and redForward[1] < selfCoordinate[1]) or (redForward[0] <=(selfCoordinate[0]+0.5) and redForward[0] > selfCoordinate[0]) or (redForward[1] <= (selfCoordinate[1]+0.45) and redForward[1] > selfCoordinate[1])):
      return self.pass_to_right(selfCoordinate, rightForward)
      
    else:
     
      if turningAngle > 90:
        return self.motions.rightSidePass,0
      elif turningAngle > 50:
        return self.motions.rightSidePass,0
      elif turningAngle > 30:
        return self.motions.rightSidePass,0    
      elif turningAngle <-50:
        return self.motions.leftSidePass,0
      elif turningAngle <-30:       
        return self.motions.leftSidePass,0
      else:      
        return self.motions.longShoot,0
      
  def check_position(self,selfCoordinate, rightForward):
    
    if (rightForward[0]<=(selfCoordinate[0]) and rightForward[1]>selfCoordinate[1]):
      return True
    else:
      return False 
  
  def pass_to_right(self,selfCoordinate, rightForward):
    
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(rightForward, selfCoordinate, robotHeadingAngle)
    if turningAngle > 90:
      return self.motions.rightSidePass,0
    elif turningAngle > 50:
      return self.motions.rightSidePass,0
    elif turningAngle > 30:
      return self.motions.longShoot,0    
    elif turningAngle <-50:
      return self.motions.leftSidePass,0
    elif turningAngle <-30:       
      return self.motions.longShoot,0
    else:
      return self.motions.longShoot,0
      
  def turnMotion(self, ballCoordinate, selfCoordinate):
    robotHeadingAngle = self.getRollPitchYaw()[2]
    turningAngle = Functions.calculateTurningAngleAccordingToRobotHeading(ballCoordinate, selfCoordinate, robotHeadingAngle)
    
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
    pass
