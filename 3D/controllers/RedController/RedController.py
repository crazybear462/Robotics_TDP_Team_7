"""
Red Team Main Controller.
Assign roles based on robot name.
"""

from controller import Robot

from RedGoalkeeper import Goalkeeper
from RedDefenderLeft import SimpleCameraFollower
from RedDefenderRight import DefenderRight
from RedForward import Forward

robot = Robot()
robotName = robot.getName()

print("[RED CTRL] robotName =", robotName)

if robotName == "RED_GK":
    robotController = Goalkeeper(robot)
elif robotName == "RED_DEF_L":
    robotController = SimpleCameraFollower(robot)
elif robotName == "RED_DEF_R":
    robotController = DefenderRight(robot)
else:
    robotController = Forward(robot)

print("[RED CTRL] Assigned:", robotController.__class__.__name__)
robotController.run()
