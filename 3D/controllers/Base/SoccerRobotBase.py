"""
SoccerRobotBase (FIXED):
- Receives Supervisor binary packets via Receiver.getData()
- Unpacks struct 'dd9ss24d' exactly as SupervisorBase sends
- Provides motions + queue helpers used by your Defender code
"""

import struct
import math
from controller import Robot

from Utils.Consts import TIME_STEP, Motions


MAX_SONAR = 2.55  # your code checks 2.55 meaning "no obstacle"


class SoccerRobot:
  def __init__(self, robot: Robot):
    self.robot = robot
    self.robotName = self.robot.getName()

    # -----------------------
    # Motions / queue
    # -----------------------
    self.motions = Motions()
    self.motionQueue = []
    self.currentlyMoving = None

    # -----------------------
    # Sensors / devices
    # -----------------------
    self.receiver = self._get_device_any(["receiver", "Receiver", "ball_receiver"])
    self.receiver = self._get_device_any(["receiver", "Receiver", "ball_receiver"])

    if self.receiver:
      self.receiver.enable(TIME_STEP)
      try:
        self.receiver.setChannel(0)
      except Exception:
        pass

    # inertial unit for yaw
    self.imu = self._get_device_any(["inertial unit", "inertialUnit", "imu"])
    if self.imu:
      try:
        self.imu.enable(TIME_STEP)
      except Exception:
        pass

    # Sonars (try common names; if not found we return MAX_SONAR)
    self.sonarLeft = self._get_device_any(["Sonar/Left", "sonar_left", "left sonar", "US/Left", "us_left"])
    self.sonarRight = self._get_device_any(["Sonar/Right", "sonar_right", "right sonar", "US/Right", "us_right"])
    for s in [self.sonarLeft, self.sonarRight]:
      if s:
        try:
          s.enable(TIME_STEP)
        except Exception:
          pass

    self.obstacleAvoidance = True

    # -----------------------
    # Latest supervisor data
    # -----------------------
    self.supervisorData = None
    self.ballX = 0.0
    self.ballY = 0.0
    self.ballOwner = "N"
    self.ballPriority = "N"
    self.lastPacketTime = -1.0

    # cached robot positions from packet
    self.robotPos = {}

  # -------------------------------------------------------------------
  # Device helper
  # -------------------------------------------------------------------
  def _get_device_any(self, names):
    for n in names:
      try:
        d = self.robot.getDevice(n)
        if d:
          return d
      except Exception:
        pass
    return None

  # -------------------------------------------------------------------
  # Supervisor packet handling (CRITICAL FIX)
  # -------------------------------------------------------------------
  def isNewBallDataAvailable(self) -> bool:
    return bool(self.receiver) and self.receiver.getQueueLength() > 0

  def getSupervisorData(self) -> bool:
    if not self.receiver or self.receiver.getQueueLength() == 0:
      return False

    raw = self.receiver.getBytes()

    # MUST match SupervisorBase: struct.pack('dd9ss24d', ...)
    # dd = ballX, ballY
    # 9s = ballOwner (padded with *)
    # s  = ballPriority
    # 24d = 8 robots * 3 coords
    try:
      unpacked = struct.unpack("dd9ss24d", raw)
    except Exception:
      # if packet size mismatched, drop it
      self.receiver.nextPacket()
      return False

    self.ballX = unpacked[0]
    self.ballY = unpacked[1]

    self.ballOwner = unpacked[2].decode("utf-8", errors="ignore").replace("*", "").strip()
    self.ballPriority = unpacked[3].decode("utf-8", errors="ignore").strip()

    doubles = unpacked[4:]  # 24 doubles

    order = [
      "RED_GK", "RED_DEF_L", "RED_DEF_R", "RED_FW",
      "BLUE_GK", "BLUE_DEF", "BLUE_FW_L", "BLUE_FW_R"
    ]

    self.robotPos = {}
    k = 0
    for name in order:
      self.robotPos[name] = [doubles[k], doubles[k + 1], doubles[k + 2]]
      k += 3

    self.supervisorData = list(unpacked)
    self.lastPacketTime = self.robot.getTime()

    self.receiver.nextPacket()
    return True

  # -------------------------------------------------------------------
  # Getters used by your behaviour scripts
  # -------------------------------------------------------------------
  def getBallData(self):
    # z not sent; keep standard ball height
    return [self.ballX, self.ballY, 0.696782]

  def getBallOwner(self) -> str:
    return self.ballOwner

  def getBallPriority(self) -> str:
    return self.ballPriority

  def getSelfCoordinate(self):
    # name mapping: your world uses RED_DEF_R etc
    if self.robotName in self.robotPos:
      return self.robotPos[self.robotName]
    # fallback (if name differs)
    return [0.0, 0.0, 0.33]

  def getLeftSonarValue(self):
    if not self.sonarLeft:
      return MAX_SONAR
    try:
      return self.sonarLeft.getValue()
    except Exception:
      return MAX_SONAR

  def getRightSonarValue(self):
    if not self.sonarRight:
      return MAX_SONAR
    try:
      return self.sonarRight.getValue()
    except Exception:
      return MAX_SONAR

  def getRollPitchYaw(self):
    if not self.imu:
      return (0.0, 0.0, 0.0)
    try:
      rpy = self.imu.getRollPitchYaw()
      return (rpy[0], rpy[1], rpy[2])
    except Exception:
      return (0.0, 0.0, 0.0)

  # -------------------------------------------------------------------
  # Goal check (simple, field centered at 0, x in [-4.5, +4.5])
  # return:  1 => my team scored, -1 => conceded, 0 => none
  # -------------------------------------------------------------------
  def checkGoal(self):
    bx, by = self.ballX, self.ballY
    GOAL_X = 4.5
    GOAL_HALF_Y = 1.3  # goal width 2.6

    is_red = self.robotName.startswith("RED")

    if bx > (GOAL_X - 0.05) and abs(by) <= GOAL_HALF_Y:
      # ball in BLUE goal (right side) => RED scored
      return 1 if is_red else -1

    if bx < (-GOAL_X + 0.05) and abs(by) <= GOAL_HALF_Y:
      # ball in RED goal (left side) => BLUE scored
      return -1 if is_red else 1

    return 0

  # -------------------------------------------------------------------
  # Motion queue helpers (matches your Defender style)
  # -------------------------------------------------------------------
  def clearMotionQueue(self):
    self.motionQueue = []

  def addMotionToQueue(self, motion):
    self.motionQueue.append(motion)

  def interruptMotion(self):
    if self.currentlyMoving:
      try:
        self.currentlyMoving.stop()
      except Exception:
        pass
      self.currentlyMoving = None

  def isNewMotionValid(self, decidedMotion) -> bool:
      if decidedMotion is None:
          return False

      # --- cooldown for turn motions (prevents spam & falling) ---
      if not hasattr(self, "_turn_cooldown"):
          self._turn_cooldown = 0
      if self._turn_cooldown > 0:
          self._turn_cooldown -= 1
          # block repeated turnRight40/turnLeft40 while cooling down
          if decidedMotion.name in ["turnRight40", "turnLeft40"]:
              return False

      if not self.currentlyMoving:
          # start cooldown only when we actually start a turn
          if decidedMotion.name in ["turnRight40", "turnLeft40"]:
              self._turn_cooldown = int(0.8 * 1000 / TIME_STEP)  # ~0.8s
          return True

      try:
          if self.currentlyMoving.name != decidedMotion.name:
              if decidedMotion.name in ["turnRight40", "turnLeft40"]:
                  self._turn_cooldown = int(0.8 * 1000 / TIME_STEP)
              return True
          return self.currentlyMoving.isOver()
      except Exception:
          return True

  def startMotion(self):
    if not self.motionQueue:
      return

    next_motion = self.motionQueue[0]

    # If nothing playing -> play
    if not self.currentlyMoving:
      self.currentlyMoving = next_motion
      try:
        self.currentlyMoving.play()
      except Exception:
        pass
      return

    # If different motion requested -> stop current and play next
    try:
      if self.currentlyMoving.name != next_motion.name:
        try:
          self.currentlyMoving.stop()
        except Exception:
          pass
        self.currentlyMoving = next_motion
        self.currentlyMoving.play()
        return

      # Same motion requested: if over, replay
      if self.currentlyMoving.isOver():
        self.currentlyMoving.play()
    except Exception:
      # if anything weird, try to play next
      self.currentlyMoving = next_motion
      try:
        self.currentlyMoving.play()
      except Exception:
        pass

  # -------------------------------------------------------------------
  # Turning motion helper (used by Defender strategies)
  # -------------------------------------------------------------------
  def getTurningMotion(self, turningAngleDeg):
    a = turningAngleDeg
    if a is None:
      return None
    if abs(a) < 10:
      return None
    if a > 120:
      return self.motions.turnLeft180
    if a > 50:
      return self.motions.turnLeft60
    if a > 25:
      return self.motions.turnLeft40
    if a < -120:
      return self.motions.turnLeft180  # your set doesn’t have Right180, keep left180
    if a < -50:
      return self.motions.turnRight60
    if a < -25:
      return self.motions.turnRight40
    return None
