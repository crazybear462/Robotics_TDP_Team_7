"""
Simple camera follow:
- find white ball in camera image
- turn until centered
- walk forward when centered
- if fall -> stand up and wait until motion finishes
"""

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from Base.SoccerRobotBase import SoccerRobot
from Utils.Consts import TIME_STEP
from controller import Camera


class SimpleCameraFollower(SoccerRobot):
    def __init__(self, robot):
        super().__init__(robot)
        print("[DEF_R_CAM] started:", self.robot.getName())

        # Try NAO camera device names (Webots NAO usually has these)
        self.camera = None
        for name in ["CameraTop", "CameraBottom", "cameraTop", "cameraBottom"]:
            try:
                cam = self.robot.getDevice(name)
                if cam:
                    self.camera = cam
                    self.camera.enable(TIME_STEP)
                    print("[DEF_R_CAM] Using camera:", name)
                    break
            except Exception:
                pass

        if self.camera is None:
            print("[DEF_R_CAM] ERROR: No camera found. Add/enable CameraTop or CameraBottom in your robot.")
            # Don't crash; just stand.
        self.standup_active = False

        # Simple tuning
        self.CENTER_TOL = 18        # pixels
        self.MIN_PIXELS = 40        # ball visibility threshold
        self.SCAN_DIR = 1           # alternate scan direction

    def run(self):
        while self.robot.step(TIME_STEP) != -1:

            # ---------- 1) FALL CHECK ----------
            z = self.getSelfCoordinate()[2]
            if z < 0.2:
                self.standup_active = True
                # your rule: both sonars==2.55 => fallen on back
                if self.getLeftSonarValue() == 2.55 and self.getRightSonarValue() == 2.55:
                    self._play_blocking(self.motions.standUpFromBack)
                else:
                    self._play_blocking(self.motions.standUpFromFront)
                continue

            # After standup, give it time to settle (IMPORTANT)
            if self.standup_active:
                if self.currentlyMoving and not self.currentlyMoving.isOver():
                    self.startMotion()
                    continue
                self.standup_active = False
                self._play_blocking(self.motions.standInit)
                continue

            # ---------- 2) CAMERA FOLLOW ----------
            if self.camera is None:
                # no camera -> just stand
                self._play(self.motions.standInit)
                continue

            motion = self._decide_from_camera()
            self._play(motion)

    # -------------------------
    # camera logic
    # -------------------------
    def _decide_from_camera(self):
        w = self.camera.getWidth()
        h = self.camera.getHeight()
        img = self.camera.getImage()

        # detect "white-ish" pixels, estimate centroid X
        bright = 0
        sum_x = 0

        step = 2  # sample every 2 pixels
        for y in range(0, h, step):
            for x in range(0, w, step):
                r = self.camera.imageGetRed(img, w, x, y)
                g = self.camera.imageGetGreen(img, w, x, y)
                b = self.camera.imageGetBlue(img, w, x, y)
                if r > 190 and g > 190 and b > 190:
                    bright += 1
                    sum_x += x

        # ball not seen -> scan
        if bright < self.MIN_PIXELS:
            # gentle scanning turn (alternate direction)
            if self.currentlyMoving and self.currentlyMoving.isOver():
                self.SCAN_DIR *= -1
            return self.motions.turnLeft40 if self.SCAN_DIR > 0 else self.motions.turnRight40

        cx = sum_x / bright
        err = cx - (w / 2)

        # turn to center
        if abs(err) > self.CENTER_TOL:
            return self.motions.turnRight40 if err > 0 else self.motions.turnLeft40

        # centered -> walk forward (more stable than sprint)
        self._loop_forwards()
        return self.motions.forwards

    # -------------------------
    # motion helpers
    # -------------------------
    def _play(self, motion):
        if self.isNewMotionValid(motion):
            self.clearMotionQueue()
            self.addMotionToQueue(motion)
        self.startMotion()

    def _play_blocking(self, motion):
        # queue motion and let it run (don’t interrupt)
        if self.isNewMotionValid(motion):
            self.clearMotionQueue()
            self.addMotionToQueue(motion)
        self.startMotion()

    def _loop_forwards(self):
        # loop walk cycle so it doesn't stop after one cycle
        if self.currentlyMoving and self.currentlyMoving.name == "forwards":
            try:
                if self.currentlyMoving.getTime() >= 1360:
                    self.currentlyMoving.setTime(360)
            except Exception:
                pass
