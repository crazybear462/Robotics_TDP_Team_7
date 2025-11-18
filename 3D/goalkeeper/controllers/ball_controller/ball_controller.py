from controller import Supervisor, Keyboard
import random

TIME_STEP = 32

supervisor = Supervisor()
keyboard = supervisor.getKeyboard()
keyboard.enable(TIME_STEP)

ball = supervisor.getFromDef("BALL")
if ball is None:
    print("[BallController] ERROR: Cannot find BALL node.")
    exit(1)

trans = ball.getField("translation")
vel = ball.getField("velocity")

KICK_POS = [-2.0, 0.0, 0.05]
GOAL_POS = [0.0, 0.0, 0.05]

mode = 1  # 默认直线模式
last_shot = supervisor.getTime()
SHOT_INTERVAL = 4.0

def reset_ball():
    trans.setSFVec3f(KICK_POS)
    vel.setSFVec3f([0, 0, 0])

def shoot_ball():
    global mode

    if mode == 1:
        print("[BallController] Mode 1: straight")
        target = GOAL_POS
        speed = 1.8

    elif mode == 2:
        print("[BallController] Mode 2: random left/right")
        offset = random.choice([-0.3, 0.3])
        target = [GOAL_POS[0], offset, GOAL_POS[2]]
        speed = 1.8

    elif mode == 3:
        print("[BallController] Mode 3: power shot")
        target = GOAL_POS
        speed = 3.0

    dx = target[0] - KICK_POS[0]
    dy = target[1] - KICK_POS[1]
    mag = (dx*dx + dy*dy)**0.5
    dx /= mag
    dy /= mag

    vel.setSFVec3f([dx * speed, dy * speed, 0])

while supervisor.step(TIME_STEP) != -1:
    key = keyboard.getKey()
    if key == ord('1'):
        mode = 1
        print("[BallController] Switch to mode 1: straight")

    if key == ord('2'):
        mode = 2
        print("[BallController] Switch to mode 2: left/right")

    if key == ord('3'):
        mode = 3
        print("[BallController] Switch to mode 3: power shot")

    now = supervisor.getTime()
    if now - last_shot > SHOT_INTERVAL:
        reset_ball()
        supervisor.step(TIME_STEP * 2)
        shoot_ball()
        last_shot = now
