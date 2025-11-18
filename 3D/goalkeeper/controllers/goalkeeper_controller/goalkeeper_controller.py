from controller import Robot
import math

robot = Robot()
time_step = int(robot.getBasicTimeStep())

# 头部
yaw = robot.getDevice("HeadYaw")
pitch = robot.getDevice("HeadPitch")

# 手臂
l_arm = robot.getDevice("LShoulderPitch")
r_arm = robot.getDevice("RShoulderPitch")

# Leg joints
l_hip_roll  = robot.getDevice("LHipRoll")
r_hip_roll  = robot.getDevice("RHipRoll")


# 通信（接收球坐标）
receiver = robot.getDevice("nao_receiver")
receiver.enable(time_step)

print("[Goalkeeper] Ready.")

while robot.step(time_step) != -1:
    if receiver.getQueueLength() > 0:
        data = receiver.getString()
        receiver.nextPacket()
        # 解析坐标
        x, y, z = map(float, data.split(","))
        # print("[Goalkeeper] ball =", x, y, z)

        # --- 头部跟踪 ---
        # ------ 限制角度范围 ------
        def clamp(v, lo, hi):
            return max(lo, min(v, hi))

        yaw_angle = math.atan2(y, x)
        pitch_angle = -math.atan2(z - 0.5, math.sqrt(x*x + y*y))

        # NAO 官方限制
        yaw_angle = clamp(yaw_angle, -2.07, 2.07)
        pitch_angle = clamp(pitch_angle, -0.65, 0.50)

        yaw.setPosition(yaw_angle)
        pitch.setPosition(pitch_angle)

        # --- 简单左右移动守门 ---
        # y > 0 球在右侧 → 身体向右倾一点
        # y < 0 球在左侧 → 身体向左倾一点

        shift = clamp(y * 0.5, -0.3, 0.3)

        l_hip_roll.setPosition( shift)
        r_hip_roll.setPosition(-shift)

        # --- 扑救动作 ---
        if 0 < x < 1.2:   # 球快到门前
            if y > 0.1:
                l_arm.setPosition(-1.2)
                r_arm.setPosition(-0.2)
            elif y < -0.1:
                r_arm.setPosition(-1.2)
                l_arm.setPosition(-0.2)
            else:
                l_arm.setPosition(-1.0)
                r_arm.setPosition(-1.0)
        
