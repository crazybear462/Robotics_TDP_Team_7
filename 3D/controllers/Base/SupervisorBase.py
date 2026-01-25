"""
The Basic Supervisor class.
All Supervisor classes should be derived from this class.
"""
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from controller import Supervisor
import struct
from Utils.Consts import (TIME_STEP, INITIAL_TRANSLATIONS, INITIAL_ROTATIONS)
from Utils import Functions

class SupervisorBase(Supervisor):
  def __init__(self):
    """Initialize the Supervisor with all necessary devices and entities."""
    super().__init__()
    
    # Get the emitter device for communication with robots
    self.emitter = self.getDevice("emitter")
    
    # Get the ball node from the world
    self.ball = self.getFromDef("SOCCERBALL")
    
    # Get all robot nodes from the world
    self.robots = {
      "RED_GK"    : self.getFromDef("RED_GK"),
      "RED_DEF_L" : self.getFromDef("RED_DEF_L"),
      "RED_DEF_R" : self.getFromDef("RED_DEF_R"),
      "RED_FW"    : self.getFromDef("RED_FW"),
      "BLUE_GK"   : self.getFromDef("BLUE_GK"),
      "BLUE_DEF"  : self.getFromDef("BLUE_DEF"),
      "BLUE_FW_L" : self.getFromDef("BLUE_FW_L"),
      "BLUE_FW_R" : self.getFromDef("BLUE_FW_R")
    }
    
    # Ball priority indicator (R=Red, B=Blue, N=Neutral)
    self.ballPriority = "R"
    
    # Track previous ball location for movement detection
    self.previousBallLocation = [0, 0, 0.0798759]
    
    # Set initial positions at startup
    self.setInitialPositions()
  
  def setInitialPositions(self) -> None:
    """
    Set all robots and ball to their initial positions and rotations. This is called at initialization and when reset is triggered.
    """
    # Reset ball to initial position
    ballTranslation = self.ball.getField("translation")
    ballTranslation.setSFVec3f(INITIAL_TRANSLATIONS["BALL"])
    
    # Reset ball rotation
    ballRotation = self.ball.getField("rotation")
    ballRotation.setSFRotation(INITIAL_ROTATIONS["BALL"])
    
    # Reset ball physics to clear any momentum
    self.ball.resetPhysics()
    
    # Reset all robots to their initial positions and rotations
    for robotName, robotNode in self.robots.items():
      # Set robot translation (position)
      robotTranslation = robotNode.getField("translation")
      robotTranslation.setSFVec3f(INITIAL_TRANSLATIONS[robotName])
      
      # Set robot rotation (orientation)
      robotRotation = robotNode.getField("rotation")
      robotRotation.setSFRotation(INITIAL_ROTATIONS[robotName])
      
      # Reset robot physics to clear any momentum or velocity
      robotNode.resetPhysics()
    
    # Reset ball tracking variables
    self.previousBallLocation = INITIAL_TRANSLATIONS["BALL"].copy()
    self.ballPriority = "R"
  
  def getBallPosition(self) -> list:
    """
    Get the soccer ball coordinate on the field.
    
    Returns:
        list: x, y, z coordinates.
    """
    # Get the translation field of the ball
    ballTranslation = self.ball.getField("translation")
    newBallLocation = ballTranslation.getSFVec3f()
    
    # Check if ball is within field boundaries (4.5m x 3m)
    if abs(newBallLocation[0]) < 4.5 and abs(newBallLocation[1]) < 3:
      # Check if ball has moved significantly (more than 5cm in any direction)
      if (self.previousBallLocation[0] + 0.05 < newBallLocation[0] or self.previousBallLocation[0] - 0.05 > newBallLocation[0] or\
          self.previousBallLocation[1] + 0.05 < newBallLocation[1] or self.previousBallLocation[1] - 0.05 > newBallLocation[1]):
        # Set ball priority to neutral when it moves
        self.ballPriority = "N"
        self.previousBallLocation = newBallLocation

    return newBallLocation
  
  def setBallPosition(self, ballPosition) -> None:
    """
    Set the soccer ball coordinate on the field.
    
    Args:
        ballPosition (list): x, y, z coordinates.
    """
    # Update tracked ball location
    self.previousBallLocation = ballPosition
    
    # Set the ball's translation field
    ballTranslation = self.ball.getField("translation")
    ballTranslation.setSFVec3f(ballPosition)
    
    # Reset physics to apply the new position immediately
    self.ball.resetPhysics()
    
  def getRobotPosition(self, robotName) -> list:
    """
    Get the robot coordinate on the field.

    Returns:
        list: x, y, z coordinates.
    """
    # Get the translation field of the specified robot
    robotTranslation = self.robots[robotName].getField("translation")
    return robotTranslation.getSFVec3f()
    
  def getBallOwner(self) -> str:
    """
    Calculate the ball owner team from the distances from the ball.
    Determines which robot is closest to the ball.
    
    Returns:
        str: Ball owner team first letter.
    """
    # Get current ball position
    ballPosition = self.getBallPosition()
    
    # Initialize with first robot as default
    ballOwnerRobotName = "RED_GK"
    minDistance = Functions.calculateDistance(ballPosition, self.getRobotPosition(ballOwnerRobotName))
    
    # Check all robots to find the closest one
    for i, key in enumerate(self.robots):
      tempDistance = Functions.calculateDistance(ballPosition, self.getRobotPosition(key))
      if tempDistance < minDistance:
        minDistance = tempDistance
        ballOwnerRobotName = key
    
    if len(ballOwnerRobotName) < 9:
      for i in range(len(ballOwnerRobotName), 9):
        ballOwnerRobotName = ballOwnerRobotName + '*'
  
    return ballOwnerRobotName

  def sendSupervisorData(self) -> None:
    """
    Send Data (ballPosition, ballOwner, ballPriority, all robot positions) to Robots.
    Communication channel is '0'.
    """
    # Gather all necessary data
    ballPosition = self.getBallPosition()
    ballOwner = bytes(self.getBallOwner(), 'utf-8')
    ballPriority = bytes(self.ballPriority, 'utf-8')
    
    # Get positions of all robots
    redGk = self.getRobotPosition("RED_GK")
    redDefLeft = self.getRobotPosition("RED_DEF_L")
    redDefRight = self.getRobotPosition("RED_DEF_R")
    redFw = self.getRobotPosition("RED_FW")
    blueGk = self.getRobotPosition("BLUE_GK")
    blueDef = self.getRobotPosition("BLUE_DEF")
    blueFwLeft = self.getRobotPosition("BLUE_FW_L")
    blueFwRight = self.getRobotPosition("BLUE_FW_R")
    
    # Pack all data into binary format for transmission
    # Format: 2 doubles (ball x,y), 9 chars (owner), 1 char (priority), 24 doubles (8 robots * 3 coords)
    data = struct.pack('dd9ss24d', ballPosition[0], ballPosition[1], ballOwner, ballPriority, 
                       redGk[0], redGk[1], redGk[2], 
                       redDefLeft[0], redDefLeft[1], redDefLeft[2], 
                       redDefRight[0], redDefRight[1], redDefRight[2],
                       redFw[0], redFw[1], redFw[2], 
                       blueGk[0], blueGk[1], blueGk[2], 
                       blueDef[0], blueDef[1], blueDef[2], 
                       blueFwLeft[0], blueFwLeft[1], blueFwLeft[2], 
                       blueFwRight[0], blueFwRight[1], blueFwRight[2])
    
    # Send the packed data via emitter
    self.emitter.send(data)

  def setBallPriority(self, priority):
    self.ballPriority = priority
    
  def resetSimulation(self):
    """
    Reset the simulation to initial state.
    Called when the reset button is pressed in Webots.
    """
    # Reset ball tracking variable
    self.previousBallLocation = [0, 0, 0.0798759]
    
    # Reset all entities to initial positions
    self.setInitialPositions()
    
    # Reset the Webots simulation
    self.simulationReset()
  
  def stopSimulation(self):
    self.simulationSetMode(self.SIMULATION_MODE_PAUSE)