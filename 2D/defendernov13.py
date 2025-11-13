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
# Robot parameters
# ----------------------------
robotSpeed = 0.5
robotRadius = 0.2

# ----------------------------
# Ball parameters
# ----------------------------
ballRadius = 0.07
ballPos = np.array([fieldLength / 2, fieldWidth / 2], dtype=float)
ballVel = np.array([0.0, 0.0])
kickSpeed = 1.2
collisionThreshold = 0.1
ballOwner = 0  # 0 = free, 1–8 = robot index

# ----------------------------
# Team 1 initial positions
# ----------------------------
T1_gkPos = np.array([0.5, fieldWidth / 2], dtype=float)
T1_defPos = np.array([1.5, 4.5], dtype=float)
T1_str1Pos = np.array([2.5, 1.5], dtype=float)
T1_str2Pos = np.array([3.5, 3.0], dtype=float)

# ----------------------------
# Team 2 initial positions
# ----------------------------
T2_gkPos = np.array([fieldLength - 0.5, fieldWidth / 2], dtype=float)
T2_defPos = np.array([fieldLength - 1.5, 3.0], dtype=float)
T2_str1Pos = np.array([fieldLength - 6.5, 2.0], dtype=float) #to test defender
T2_str2Pos = np.array([fieldLength - 6.5, 4.0], dtype=float) #to test defender

# ----------------------------
# Score
# ----------------------------
score = [0, 0]

# ----------------------------
# Simulation parameters
# ----------------------------
numSteps = 600
dt = 0.1

# ----------------------------
# Figure setup
# ----------------------------
plt.ion()
fig, ax = plt.subplots()
ax.set_xlim(0, fieldLength)
ax.set_ylim(0, fieldWidth)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title('2D Robotic Soccer: Defender Awareness')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')

# Goals
ax.add_patch(plt.Rectangle((0, fieldWidth / 2 - goalWidth / 2), 0.2, goalWidth, color='red'))
ax.add_patch(plt.Rectangle((fieldLength - 0.2, fieldWidth / 2 - goalWidth / 2), 0.2, goalWidth, color='blue'))

# Robot plots
plots = {}
plots['T1_gk'], = ax.plot([], [], 'go', markersize=8)
plots['T1_def'], = ax.plot([], [], 'ro', markersize=8)
plots['T1_str1'], = ax.plot([], [], 'mo', markersize=8)
plots['T1_str2'], = ax.plot([], [], 'mo', markersize=8)
plots['T2_gk'], = ax.plot([], [], 'co', markersize=8)
plots['T2_def'], = ax.plot([], [], 'bo', markersize=8)
plots['T2_str1'], = ax.plot([], [], 'yo', markersize=8)
plots['T2_str2'], = ax.plot([], [], 'yo', markersize=8)

# Ball
ballPlot, = ax.plot([], [], 'm*', markersize=10)

# Direction arrows
arrows = []

# Scoreboard
scoreText = ax.text(fieldLength / 2 - 1, fieldWidth + 0.1,
                    f"Team 1: {score[0]} – Team 2: {score[1]}",
                    fontsize=10, fontweight='bold')

# ----------------------------
# Helper functions
# ----------------------------
def move_toward(pos, target, speed, dt):
    vec = target - pos
    dist = np.linalg.norm(vec)
    if dist > 1e-6:
        return pos + speed * dt * vec / dist
    return pos

def avoid_overlap(pos, others, radius):
    for o in others:
        diff = pos - o
        dist = np.linalg.norm(diff)
        if dist < 2 * radius and dist > 1e-6:
            pos += (diff / dist) * (2 * radius - dist) / 2
    return pos

def team_chase_ball(team_positions, ball_pos, team_indices):
    dists = [np.linalg.norm(team_positions[i] - ball_pos) for i in team_indices]
    chaser_idx = team_indices[np.argmin(dists)]
    return chaser_idx

def defender_pass(def_pos, allies):
    """Pass ball to nearest striker (never goalkeeper) with slight lateral offset."""
    if not allies:
        return np.zeros(2)
    # distances = [np.linalg.norm(def_pos - a) for a in allies]
    # idx = np.argmin(distances)
    # dir_vec = allies[idx] - def_pos
    # dir_vec[1] += np.sign(dir_vec[1]) * 0.1
    # norm = np.linalg.norm(dir_vec)
    # if norm < 1e-6:
    #     return np.zeros(2)
    # return kickSpeed * dir_vec / norm

        # Choose nearest ally
    distances = [np.linalg.norm(def_pos - a) for a in allies]
    idx = np.argmin(distances)
    target = allies[idx]

    # Direction vector to target
    dir_vec = target - def_pos
    norm = np.linalg.norm(dir_vec)
    if norm < 1e-6:
        return np.zeros(2)
    dir_vec /= norm

    # Rotate direction 90 degrees to the right (clockwise)
    dir_right = np.array([dir_vec[1], -dir_vec[0]])

    # Scale by kick speed
    return kickSpeed * dir_right

# ----------------------------
# Simulation loop
# ----------------------------
for _ in range(numSteps):
    allRobots = [T1_gkPos, T1_defPos, T1_str1Pos, T1_str2Pos,
                 T2_gkPos, T2_defPos, T2_str1Pos, T2_str2Pos]

    # --- Ball possession ---
    dists = [np.linalg.norm(r - ballPos) for r in allRobots]
    if ballOwner == 0:
        idx = np.argmin(dists)
        if dists[idx] <= collisionThreshold:
            ballOwner = idx + 1
            # Ball kick logic
            if idx == 2:  # T1 defender
                allies = [T1_str1Pos, T1_str2Pos]
                ballVel = defender_pass(T1_defPos, allies)
            elif idx == 6:  # T2 defender
                allies = [T2_str1Pos, T2_str2Pos]
                ballVel = defender_pass(T2_defPos, allies)
            elif idx < 4:  # Team1 striker/gk
                dir_vec = np.array([fieldLength, fieldWidth / 2]) - ballPos
                ballVel = kickSpeed * dir_vec / np.linalg.norm(dir_vec)
            else:  # Team2 striker/gk
                dir_vec = np.array([0, fieldWidth / 2]) - ballPos
                ballVel = kickSpeed * dir_vec / np.linalg.norm(dir_vec)

    # Ball motion
    ballPos += ballVel * dt
    ballVel *= 0.95
    if np.linalg.norm(ballVel) < 0.05:
        ballVel[:] = 0
        ballOwner = 0

    # ---------------- Team 1 ----------------
    # Goalkeeper
    # T1_gkPos[1] += robotSpeed * dt * np.sign(ballPos[1] - T1_gkPos[1])
    # T1_gkPos[0] = 0.5

    # Defender
    if ballPos[0] > dangerZone:  # Outside danger zone: follow y-axis
        T1_defPos[1] += robotSpeed * dt * np.sign(ballPos[1] - T1_defPos[1])
        T1_defPos[0] = 1.5
    else:  # Inside danger zone: block and kick to strikers
        dir_vec = ballPos - T1_defPos
        dist = np.linalg.norm(dir_vec)
        if dist > 1e-6:
            T1_defPos = move_toward(T1_defPos, ballPos, robotSpeed, dt)
            # When defender touches ball, just pass to striker
            if dist < collisionThreshold:
                allies = [T1_str1Pos, T1_str2Pos]
                ballVel = defender_pass(T1_defPos, allies)
                ballOwner = 0  # defender does not take ownership

    # Strikers
    team1_strikers = [2, 3]
    chaser_idx = team_chase_ball(allRobots, ballPos, team1_strikers)
    for i, idx in enumerate(team1_strikers):
        if idx == chaser_idx:
            allRobots[idx] = move_toward(allRobots[idx], ballPos, robotSpeed, dt)
        else:
            target = np.array([fieldLength / 2, (i + 1) * fieldWidth / 3])
            allRobots[idx] = move_toward(allRobots[idx], target, robotSpeed * 0.8, dt)

    T1_str1Pos = allRobots[2]
    T1_str2Pos = allRobots[3]

    # ---------------- Team 2 ----------------
    # T2_gkPos[1] += robotSpeed * dt * np.sign(ballPos[1] - T2_gkPos[1])
    # T2_gkPos[0] = fieldLength - 0.5

    # Team 2 defender
    if ballPos[0] < fieldLength - dangerZone:
        T2_defPos[1] += robotSpeed * dt * np.sign(ballPos[1] - T2_defPos[1])
        T2_defPos[0] = fieldLength - 1.5
    else:
        dir_vec = ballPos - T2_defPos
        dist = np.linalg.norm(dir_vec)
        if dist > 1e-6:
            T2_defPos = move_toward(T2_defPos, ballPos, robotSpeed, dt)
            if dist < collisionThreshold:
                allies = [T2_str1Pos, T2_str2Pos]
                ballVel = defender_pass(T2_defPos, allies)
                ballOwner = 0

    # Strikers
    team2_strikers = [6, 7]
    chaser_idx = team_chase_ball(allRobots, ballPos, team2_strikers)
    for i, idx in enumerate(team2_strikers):
        if idx == chaser_idx:
            allRobots[idx] = move_toward(allRobots[idx], ballPos, robotSpeed, dt)
        else:
            target = np.array([fieldLength / 2, (i + 1) * fieldWidth / 3])
            allRobots[idx] = move_toward(allRobots[idx], target, robotSpeed * 0.8, dt)

    T2_str1Pos = allRobots[6]
    T2_str2Pos = allRobots[7]

    # ---------------- Prevent overlaps ----------------
    for i in range(len(allRobots)):
        allRobots[i] = avoid_overlap(allRobots[i], allRobots[:i] + allRobots[i + 1:], robotRadius)

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
    for i, pos in enumerate(robot_positions):
        plots[list(plots.keys())[i]].set_data([pos[0]], [pos[1]])
        direction = ballPos - pos
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction /= norm
        arrows.append(ax.arrow(pos[0], pos[1], 0.3 * direction[0], 0.3 * direction[1],
                               head_width=0.1, fc='k', ec='k', alpha=0.5))

    ballPlot.set_data([ballPos[0]], [ballPos[1]])
    scoreText.set_text(f"Team 1: {score[0]} – Team 2: {score[1]}")
    plt.pause(0.05)

plt.ioff()
plt.show()
