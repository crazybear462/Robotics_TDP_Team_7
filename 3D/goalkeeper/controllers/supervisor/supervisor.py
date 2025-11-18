from controller import Supervisor
import random
import math

supervisor = Supervisor()
time_step = int(supervisor.getBasicTimeStep())

# --- 获取足球 ---
ball = supervisor.getFromDef("BALL")
if ball is None:
    print("[Supervisor] ERROR: BALL not found. Check DEF name.")
    exit(1)

ball_pos = ball.getField("translation")

# --- 通信（发给 NAO） ---
emitter = supervisor.getDevice("supervisor_emitter")

# --- 球状态变量 ---
vx = 0.0
vy = 0.0
vz = 0.0

# 球半径（你截图中是 0.07）
BALL_RADIUS = 0.07
GROUND_Z = BALL_RADIUS  # 球中心最低 z

def reset_and_shoot():
    global vx, vy, vz
    
    # 初始位置——让球稍微离地
    ball_pos.setSFVec3f([-3.0, 0.0, GROUND_Z + 0.02])

    # 随机速度
    vx = 4.0
    vy = random.uniform(-1.0, 1.0)
    vz = random.uniform(0.5, 1.2)

    print("[Supervisor] Shoot → vx=%.2f vy=%.2f vz=%.2f" % (vx, vy, vz))


# 初始化
reset_and_shoot()

while supervisor.step(time_step) != -1:

    dt = time_step / 1000.0

    # --- 当前球位置 ---
    x, y, z = ball_pos.getSFVec3f()

    # --- 重力 ---
    vz += -9.81 * dt

    # --- 位置更新 ---
    x += vx * dt
    y += vy * dt
    z += vz * dt

    # --- 地面碰撞 ---
    if z < GROUND_Z:
        z = GROUND_Z
        vz = -vz * 0.3   # 弹起一点（0 = 不弹）

    # --- 位置写回世界 ---
    ball_pos.setSFVec3f([x, y, z])

    # --- 发给 NAO ---
    msg = f"{x},{y},{z}"
    emitter.send(msg.encode())

    # --- 球飞过守门员重新来一球 ---
    if x > 2.5:
        reset_and_shoot()
