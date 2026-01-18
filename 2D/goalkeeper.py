import numpy as np
import matplotlib.pyplot as plt

# ----------------------------
# Field parameters
# ----------------------------
fieldLength = 9.0
fieldWidth = 6.0
goalWidth = 2.6
dangerZone = 2.0  # meters from goal

# ----------------------------
# Robot parameters (ALL SAME SPEED)
# ----------------------------
robotSpeed = 0.25
robotRadius = 0.2
kickSpeed = 0.5

# ----------------------------
# Simulation parameters
# ----------------------------
numSteps = 600
dt = 0.1  # seconds per simulation step
maxStep = robotSpeed * dt  # max distance per step for EVERY robot (including GK)

# ----------------------------
# Ball parameters
# ----------------------------
ballRadius = 0.07
ballPos = np.array([fieldLength / 2, 4.0], dtype=float)
ballVel = np.array([0.0, 0.0])
collisionThreshold = 0.12
ballOwner = 0  # 0 = free
ballFriction = 0.95

# ----------------------------
# Goalkeeper parameters (movement uses maxStep)
# ----------------------------
gkLineOffset = 0.5
gkFollowGain = 0.35
gkGoalProximityX = 1.8
gkClearSpeed = 0.9

# ----------------------------
# Fast kick + bypass opponent beside parameters
# ----------------------------
FAST_KICK_PROB = 0.30
FAST_KICK_MULT = 2.2

BYPASS_ONLY_WHEN_FAST = True
BYPASS_TRIGGER_DIST = 0.9
BYPASS_SIDE_GAIN = 0.70
BYPASS_FORWARD_GAIN = 1.00
BYPASS_RANDOM_JITTER = 0.05

# Defender fast clearance (ball velocity)
defClearSpeed = 1.3
defClearUpBias = 0.15

# ----------------------------
# NEW: Side deflection parameters (reflect/deflect attacks)
# ----------------------------
ATTACK_SPEED_TH = 0.35        # if |ballVel| above this, treat as "shot/attack"
DEFLECT_SPEED_GK = 1.1        # ball speed when GK deflects to side
DEFLECT_SPEED_DEF = 1.0       # ball speed when defender deflects to side
SIDE_MARGIN = 0.15            # keep inside field boundaries

# ----------------------------
# Team 1 initial positions
# ----------------------------
T1_gkPos = np.array([gkLineOffset, fieldWidth / 2], dtype=float)
T1_defPos = np.array([1.5, 4.5], dtype=float)
T1_str1Pos = np.array([2.5, 1.5], dtype=float)
T1_str2Pos = np.array([3.5, 5.0], dtype=float)

# ----------------------------
# Team 2 initial positions
# ----------------------------
T2_gkPos = np.array([fieldLength - gkLineOffset, fieldWidth / 2], dtype=float)
T2_defPos = np.array([1.0, 3.0], dtype=float)  # test: far away
T2_str1Pos = np.array([fieldLength - 6.5, 2.0], dtype=float)
T2_str2Pos = np.array([fieldLength - 6.5, 4.0], dtype=float)

# Defender home positions
T1_defHome = np.array([1.5, fieldWidth / 2], dtype=float)
T2_defHome = np.array([fieldLength - 1.5, fieldWidth / 2], dtype=float)

# ----------------------------
# Score
# ----------------------------
score = [0, 0]

# ----------------------------
# Figure setup
# ----------------------------
plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(0, fieldLength)
ax.set_ylim(0, fieldWidth)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title('2D Robotic Soccer: same speed + side deflection for GK/DEF')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')

# Goals
ax.add_patch(plt.Rectangle((0, fieldWidth / 2 - goalWidth / 2), 0.2, goalWidth, color='red'))
ax.add_patch(plt.Rectangle((fieldLength - 0.2, fieldWidth / 2 - goalWidth / 2), 0.2, goalWidth, color='blue'))

# Robot plots (distinct markers)
plots = {}
plots['T1_gk'], = ax.plot([], [], 'ro', markersize=9)
plots['T1_def'], = ax.plot([], [], 'ro', markersize=9)
plots['T1_str1'], = ax.plot([], [], 'ro', markersize=9)
plots['T1_str2'], = ax.plot([], [], 'ro', markersize=9)

plots['T2_gk'], = ax.plot([], [], 'bo', markersize=9)
plots['T2_def'], = ax.plot([], [], 'bo', markersize=9)
plots['T2_str1'], = ax.plot([], [], 'bo', markersize=9)
plots['T2_str2'], = ax.plot([], [], 'bo', markersize=9)

# Ball
ballPlot, = ax.plot([], [], 'k*', markersize=10)

# Direction arrows
arrows = []

# Scoreboard
scoreText = ax.text(fieldLength / 2 - 1, fieldWidth + 0.1,
                    f"Team 1: {score[0]} – Team 2: {score[1]}",
                    fontsize=10, fontweight='bold')

# ----------------------------
# Helper functions
# ----------------------------
def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def move_toward(pos, target, speed, dt):
    """Move toward target with limited step speed*dt."""
    vec = target - pos
    dist = np.linalg.norm(vec)
    if dist > 1e-9:
        step = speed * dt
        if step >= dist:
            return target.copy()
        return pos + step * vec / dist
    return pos

def avoid_overlap(pos, others, radius):
    for o in others:
        diff = pos - o
        dist = np.linalg.norm(diff)
        if dist < 2 * radius and dist > 1e-6:
            pos += (diff / dist) * (2 * radius - dist) / 2
    return pos

def nearest_opponent(kicker_pos, opponents):
    if not opponents:
        return None, np.inf
    dists = [np.linalg.norm(o - kicker_pos) for o in opponents]
    i = int(np.argmin(dists))
    return opponents[i], dists[i]

def is_attack_shot(ball_vel, team_is_left=True):
    """
    Attack means: ball is moving toward your goal AND fast enough.
    Team 1 (left goal): attack if vx < 0
    Team 2 (right goal): attack if vx > 0
    """
    speed = np.linalg.norm(ball_vel)
    if speed < ATTACK_SPEED_TH:
        return False
    vx = ball_vel[0]
    return (vx < -0.02) if team_is_left else (vx > 0.02)

def side_deflect_velocity(robot_pos, ball_pos, ball_vel, team_is_left=True, out_speed=1.0):
    """
    Deflect to left/right side line (like a block -> clearance to corner/side).
    Choose the side that is "safer": push ball toward the nearest touchline away from center.
    """
    # choose top or bottom touchline depending on which side ball is already on
    if ball_pos[1] >= fieldWidth / 2:
        y_target = fieldWidth - SIDE_MARGIN   # top
    else:
        y_target = SIDE_MARGIN                # bottom

    # also push slightly forward (away from own goal)
    x_target = fieldLength * 0.65 if team_is_left else fieldLength * 0.35

    target = np.array([x_target, y_target])
    v = target - robot_pos
    n = np.linalg.norm(v)
    if n < 1e-9:
        return np.zeros(2)
    return out_speed * v / n

def goalkeeper_clear_forward(gk_pos, team_is_left=True):
    """Forward clear (ball velocity)."""
    if team_is_left:
        target = np.array([fieldLength * 0.80, fieldWidth / 2])
    else:
        target = np.array([fieldLength * 0.20, fieldWidth / 2])
    v = target - gk_pos
    n = np.linalg.norm(v)
    if n < 1e-9:
        return np.zeros(2)
    return gkClearSpeed * v / n

gkMaxOut=0.5
def goalkeeper_move_same_speed(gk_pos, def_pos, ball_pos, ball_vel, team_is_left=True):
    """
    GK stays on goal line (x fixed). Movement limited to maxStep = robotSpeed*dt.
    - When ball is near own goal: track ball y (still limited).
    - Otherwise: stay near center with small defender-follow.
    """
    goal_center_y = fieldWidth / 2
    y_low = goal_center_y - goalWidth / 2
    y_high = goal_center_y + goalWidth / 2

    # Allowed x range for GK (come out a little, but not too far)
    if team_is_left:
        x_max = gkLineOffset
        x_min = gkLineOffset + gkMaxOut  # e.g. 0.5 -> 1.0
    else:
        x_max = fieldLength - gkLineOffset  # 8.5
        x_min = (fieldLength - gkLineOffset) - gkMaxOut  # 8.0 if gkMaxOut=0.5

    # ---- Y target: normal follow defender, or track ball if near goal ----
    def_offset = def_pos[1] - goal_center_y
    normal_target_y = clamp(goal_center_y + gkFollowGain * def_offset, y_low, y_high)

    ball_near_goal = (ball_pos[0] <= gkGoalProximityX) if team_is_left else (
                ball_pos[0] >= fieldLength - gkGoalProximityX)
    target_y = clamp(ball_pos[1], y_low, y_high) if ball_near_goal else normal_target_y

    # ---- X target: only come out when ball near goal ----
    if ball_near_goal:
        # move toward ball x but clamp within [x_min, x_max]
        desired_x = ball_pos[0]
        target_x = clamp(desired_x, x_min, x_max) if team_is_left else clamp(desired_x, x_min, x_max)
    else:
        # stay on the line
        target_x = x_max

    # Move toward (target_x, target_y) with SAME speed as others
    target = np.array([target_x, target_y], dtype=float)
    return move_toward(gk_pos, target, robotSpeed, dt)

def defender_clear_forward(def_pos, team_is_left=True):
    signy = np.sign(def_pos[1] - fieldWidth / 2)
    signy = 1.0 if signy == 0 else signy
    y_target = clamp(def_pos[1] + defClearUpBias * signy, 0, fieldWidth)

    if team_is_left:
        target = np.array([fieldLength * 0.85, y_target])
    else:
        target = np.array([fieldLength * 0.15, y_target])

    v = target - def_pos
    n = np.linalg.norm(v)
    if n < 1e-9:
        return np.zeros(2)
    return defClearSpeed * v / n

def kick_toward_goal_with_bypass(kicker_pos, team_is_left=True, base_speed=kickSpeed, opponents=None, fast=False):
    opponents = opponents or []
    goal_target = np.array([fieldLength, fieldWidth / 2]) if team_is_left else np.array([0, fieldWidth / 2])

    fwd = goal_target - kicker_pos
    nf = np.linalg.norm(fwd)
    if nf < 1e-9:
        return np.zeros(2)
    fwd = fwd / nf

    do_bypass = fast if BYPASS_ONLY_WHEN_FAST else True
    if do_bypass:
        opp, d = nearest_opponent(kicker_pos, opponents)
        if opp is not None and d < BYPASS_TRIGGER_DIST:
            right = np.array([fwd[1], -fwd[0]])
            left = -right
            rel = opp - kicker_pos
            side_sign = np.dot(rel, right)
            side = left if side_sign > 0 else right
            v = BYPASS_FORWARD_GAIN * fwd + BYPASS_SIDE_GAIN * side
            v = v + BYPASS_RANDOM_JITTER * np.random.randn(2)
            nv = np.linalg.norm(v)
            if nv > 1e-9:
                v = v / nv
            return base_speed * v

    return base_speed * fwd

def defender_pass_sideways(def_pos, allies):
    if not allies:
        return np.zeros(2)
    distances = [np.linalg.norm(def_pos - a) for a in allies]
    idx = int(np.argmin(distances))
    target = allies[idx]
    dir_vec = target - def_pos
    n = np.linalg.norm(dir_vec)
    if n < 1e-9:
        return np.zeros(2)
    dir_vec /= n
    dir_right = np.array([dir_vec[1], -dir_vec[0]])
    return kickSpeed * dir_right


# ----------------------------
# Simulation loop
# ----------------------------
for _ in range(numSteps):
    # ---- GK movement (SAME SPEED LIMIT) ----
    T1_gkPos = goalkeeper_move_same_speed(T1_gkPos, T1_defPos, ballPos, ballVel, team_is_left=True)
    T2_gkPos = goalkeeper_move_same_speed(T2_gkPos, T2_defPos, ballPos, ballVel, team_is_left=False)

    # ---- Build list for touch detection ----
    allRobots = [T1_gkPos, T1_defPos, T1_str1Pos, T1_str2Pos,
                 T2_gkPos, T2_defPos, T2_str1Pos, T2_str2Pos]

    # ---- Ball touch (any time, even moving) ----
    dists = [np.linalg.norm(r - ballPos) for r in allRobots]
    if ballOwner == 0:
        idx = int(np.argmin(dists))
        if dists[idx] <= collisionThreshold:
            is_fast = (np.random.rand() < FAST_KICK_PROB)
            speed = kickSpeed * (FAST_KICK_MULT if is_fast else 1.0)

            team1_opps = [T2_gkPos, T2_defPos, T2_str1Pos, T2_str2Pos]
            team2_opps = [T1_gkPos, T1_defPos, T1_str1Pos, T1_str2Pos]

            # --- GK / DEF reflect logic: if attack shot incoming -> side deflect ---
            if idx == 0:  # T1 GK
                if is_attack_shot(ballVel, team_is_left=True):
                    ballVel = side_deflect_velocity(T1_gkPos, ballPos, ballVel, team_is_left=True, out_speed=DEFLECT_SPEED_GK)
                else:
                    ballVel = goalkeeper_clear_forward(T1_gkPos, team_is_left=True)

            elif idx == 4:  # T2 GK
                if is_attack_shot(ballVel, team_is_left=False):
                    ballVel = side_deflect_velocity(T2_gkPos, ballPos, ballVel, team_is_left=False, out_speed=DEFLECT_SPEED_GK)
                else:
                    ballVel = goalkeeper_clear_forward(T2_gkPos, team_is_left=False)

            elif idx == 1:  # T1 defender
                if is_attack_shot(ballVel, team_is_left=True) and ballPos[0] <= dangerZone + 0.8:
                    ballVel = side_deflect_velocity(T1_defPos, ballPos, ballVel, team_is_left=True, out_speed=DEFLECT_SPEED_DEF)
                else:
                    ballVel = defender_clear_forward(T1_defPos, team_is_left=True) if ballPos[0] <= dangerZone + 0.5 \
                        else defender_pass_sideways(T1_defPos, [T1_str1Pos, T1_str2Pos])

            elif idx == 5:  # T2 defender
                if is_attack_shot(ballVel, team_is_left=False) and ballPos[0] >= fieldLength - (dangerZone + 0.8):
                    ballVel = side_deflect_velocity(T2_defPos, ballPos, ballVel, team_is_left=False, out_speed=DEFLECT_SPEED_DEF)
                else:
                    ballVel = defender_clear_forward(T2_defPos, team_is_left=False) if ballPos[0] >= fieldLength - (dangerZone + 0.5) \
                        else defender_pass_sideways(T2_defPos, [T2_str1Pos, T2_str2Pos])

            elif idx in [2, 3]:  # Team 1 strikers
                ballVel = kick_toward_goal_with_bypass(ballPos, True, speed, team1_opps, is_fast)

            elif idx in [6, 7]:  # Team 2 strikers
                ballVel = kick_toward_goal_with_bypass(ballPos, False, speed, team2_opps, is_fast)

            else:
                ballVel = np.zeros(2)

            ballOwner = 0

    # ---- Ball motion ----
    ballPos += ballVel * dt
    ballVel *= ballFriction
    if np.linalg.norm(ballVel) < 0.05:
        ballVel[:] = 0
        ballOwner = 0

    # ---------------- Defender behavior (ONE MOVE, SAME SPEED) ----------------
    if ballPos[0] > dangerZone:
        T1_defPos = move_toward(T1_defPos, np.array([T1_defHome[0], ballPos[1]]), robotSpeed, dt)
    else:
        T1_defPos = move_toward(T1_defPos, ballPos, robotSpeed, dt)

    if ballPos[0] < fieldLength - dangerZone:
        T2_defPos = move_toward(T2_defPos, np.array([T2_defHome[0], ballPos[1]]), robotSpeed, dt)
    else:
        T2_defPos = move_toward(T2_defPos, ballPos, robotSpeed, dt)

    # ---------------- Strikers movement (BOTH TEAMS, SAME SPEED) ----------------
    # Team 1
    d1 = np.linalg.norm(T1_str1Pos - ballPos)
    d2 = np.linalg.norm(T1_str2Pos - ballPos)
    if d1 <= d2:
        T1_str1Pos = move_toward(T1_str1Pos, ballPos, robotSpeed, dt)
        T1_str2Pos = move_toward(T1_str2Pos, np.array([fieldLength/2, 2*fieldWidth/3]), robotSpeed, dt)
    else:
        T1_str2Pos = move_toward(T1_str2Pos, ballPos, robotSpeed, dt)
        T1_str1Pos = move_toward(T1_str1Pos, np.array([fieldLength/2, fieldWidth/3]), robotSpeed, dt)

    # Team 2
    d1 = np.linalg.norm(T2_str1Pos - ballPos)
    d2 = np.linalg.norm(T2_str2Pos - ballPos)
    # if d1 <= d2:
    #     T2_str1Pos = move_toward(T2_str1Pos, ballPos, robotSpeed, dt)
    #     T2_str2Pos = move_toward(T2_str2Pos, np.array([fieldLength/2, 2*fieldWidth/3]), robotSpeed, dt)
    # else:
    #     T2_str2Pos = move_toward(T2_str2Pos, ballPos, robotSpeed, dt)
    #     T2_str1Pos = move_toward(T2_str1Pos, np.array([fieldLength/2, fieldWidth/3]), robotSpeed, dt)

    # ---------------- Prevent overlaps ----------------
    allRobots = [T1_gkPos, T1_defPos, T1_str1Pos, T1_str2Pos,
                 T2_gkPos, T2_defPos, T2_str1Pos, T2_str2Pos]
    for i in range(len(allRobots)):
        allRobots[i] = avoid_overlap(allRobots[i], allRobots[:i] + allRobots[i + 1:], robotRadius)

    T1_gkPos, T1_defPos, T1_str1Pos, T1_str2Pos, T2_gkPos, T2_defPos, T2_str1Pos, T2_str2Pos = allRobots

    # ---------------- Goal detection ----------------
    if ballPos[0] <= 0.2 and abs(ballPos[1] - fieldWidth / 2) <= goalWidth / 2:
        score[1] += 1
        ballPos[:] = [fieldLength / 2, fieldWidth / 2]
        ballVel[:] = 0
        ballOwner = 0
    elif ballPos[0] >= fieldLength - 0.2 and abs(ballPos[1] - fieldWidth / 2) <= goalWidth / 2:
        score[0] += 1
        ballPos[:] = [fieldLength / 2, fieldWidth / 2]
        ballVel[:] = 0
        ballOwner = 0

    # ---------------- Update plots ----------------
    robot_positions = [T1_gkPos, T1_defPos, T1_str1Pos, T1_str2Pos,
                       T2_gkPos, T2_defPos, T2_str1Pos, T2_str2Pos]

    for arrow in arrows:
        arrow.remove()
    arrows.clear()

    keys = list(plots.keys())
    for i, pos in enumerate(robot_positions):
        plots[keys[i]].set_data([pos[0]], [pos[1]])
        direction = ballPos - pos
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction /= norm
        arrows.append(ax.arrow(pos[0], pos[1], 0.25 * direction[0], 0.25 * direction[1],
                               head_width=0.08, fc='k', ec='k', alpha=0.35))

    ballPlot.set_data([ballPos[0]], [ballPos[1]])
    scoreText.set_text(f"Team 1: {score[0]} – Team 2: {score[1]}")
    plt.pause(0.05)

plt.ioff()
plt.show()
