from controller import Robot, Camera, Accelerometer, LED, Motor, Motion
import os
import math

class NaoController:
    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        # Devices
        self.camera_top = None
        self.camera_bottom = None
        self.accelerometer = None
        self.leds = []
        self.RShoulderPitch = None
        self.LShoulderPitch = None
        self.HeadYaw = None
        self.HeadPitch = None

        # Motions and sequence
        self.motion_list = {}
        self.currently_playing = None
        self.sequence = None

        # Setup
        self.find_and_enable_devices()
        self.load_motion_list()
        self.sequence = self.read_sequence()

    # -----------------------------------------------------------------------
    def find_and_enable_devices(self):
        """Find and enable Nao's sensors and actuators."""
        self.camera_top = self.robot.getDevice("CameraTop")
        self.camera_bottom = self.robot.getDevice("CameraBottom")
        self.camera_top.enable(self.time_step)
        self.camera_bottom.enable(self.time_step)

        self.accelerometer = self.robot.getDevice("accelerometer")
        self.accelerometer.enable(self.time_step)

        led_names = [
            "ChestBoard/Led", "RFoot/Led", "LFoot/Led",
            "Face/Led/Right", "Face/Led/Left",
            "Ears/Led/Right", "Ears/Led/Left"
        ]
        self.leds = [self.robot.getDevice(name) for name in led_names]

        self.RShoulderPitch = self.robot.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.robot.getDevice("LShoulderPitch")
        self.HeadYaw = self.robot.getDevice("HeadYaw")
        self.HeadPitch = self.robot.getDevice("HeadPitch")

    # -----------------------------------------------------------------------
    def load_motion_list(self):
        """Load motion files from the motions directory."""
        motion_dir = os.path.join(os.path.dirname(__file__), "motions")
        if not os.path.exists(motion_dir):
            print(f"Motion directory not found: {motion_dir}")
            return

        for filename in os.listdir(motion_dir):
            if filename.endswith(".motion"):
                name = os.path.splitext(filename)[0]
                path = os.path.join(motion_dir, filename)
                try:
                    self.motion_list[name] = Motion(path)
                    print(f"Loaded motion: {name}")
                except Exception as e:
                    print(f"Error loading {filename}: {e}")

    # -----------------------------------------------------------------------
    def start_motion(self, name):
        """Stop current motion and start a new one."""
        if name not in self.motion_list:
            print(f"Motion not found: {name}")
            return
        if self.currently_playing:
            self.currently_playing.stop()
        motion = self.motion_list[name]
        motion.play()
        self.currently_playing = motion

    # -----------------------------------------------------------------------
    def read_sequence(self):
        """Read the robot-specific sequence file (e.g., B1.txt)."""
        name = self.robot.getName()
        color = name[4]  # 'B' or 'R'
        number = name[9] if color == 'B' else name[8]
        filename = f"{color}{number}.txt"
        filepath = os.path.join(os.path.dirname(__file__), filename)

        if not os.path.exists(filepath):
            print(f"Sequence file not found: {filepath}")
            return []

        sequence = []
        with open(filepath, "r") as f:
            for line in f:
                line = line.strip()
                if not line or ":" not in line:
                    continue
                t, action = line.split(":", 1)
                try:
                    sequence.append((int(t), action.strip()))
                except ValueError:
                    continue
        return sequence

    # -----------------------------------------------------------------------
    def run_action(self, action):
        """Execute an action line from the sequence."""
        if action.startswith("motion:"):
            self.start_motion(action[7:])

        elif action.startswith("eyes:"):
            try:
                left_hex = action[5:12]
                right_hex = action[12:].strip()
                left_val = int(left_hex, 16)
                right_val = int(right_hex, 16)
                self.leds[3].set(left_val)
                self.leds[4].set(right_val)
                for i in [0, 1, 2, 5, 6]:
                    self.leds[i].set(0)
            except Exception:
                print(f"Invalid eyes action: {action}")

        elif action.startswith("yaw:"):
            yaw = float(action[4:])
            self.HeadYaw.setPosition(yaw)

        elif action.startswith("pitch:"):
            pitch = float(action[6:])
            self.HeadPitch.setPosition(pitch)

        elif action.startswith("left_arm:"):
            val = float(action[9:])
            self.LShoulderPitch.setPosition(val)

        else:
            print(f"Unknown action: {action}")

    # -----------------------------------------------------------------------
    def run_sequence(self):
        """Check and run actions according to current simulation time."""
        time_ms = int(self.robot.getTime() * 1000)
        for t, action in self.sequence:
            if t == time_ms:
                self.run_action(action)

    # -----------------------------------------------------------------------
    def run(self):
        """Main control loop."""
        while self.robot.step(self.time_step) != -1:
            acc = self.accelerometer.getValues()

            if (self.currently_playing is None or self.currently_playing.isOver()) and \
               abs(acc[0]) > abs(acc[1]) and abs(acc[0]) > abs(acc[2]) and acc[0] < -5:
                self.start_motion("StandUpFromFront")
            else:
                self.run_sequence()

        # Cleanup (on simulation reset)
        for motion in self.motion_list.values():
            motion.stop()


# -----------------------------------------------------------------------
if __name__ == "__main__":
    controller = NaoController()
    controller.run()
