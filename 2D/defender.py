import numpy as np
import matplotlib.pyplot as plt

# ----------------------------
# Field parameters (9 m × 6 m)
# ----------------------------
fieldLength = 9.0   # meters
fieldWidth = 6.0
goalWidth = 2.6

# ----------------------------
# Robot parameters
# ----------------------------
robotSpeed = 0.25    # m per time step
robotRadius = 0.2

# ----------------------------
# Ball parameters
# ----------------------------
ballRadius = 0.07
ballPos = np.array([fieldLength / 2, fieldWidth / 2], dtype=float)
ballVel = np.array([0.0, 0.0])
kickSpeed = 1.2
collisionThreshold = 0.05  # literal touch threshold
ballOwner = 0              # 0 = free, 1–8 = robot index

# ----------------------------
# Team 1 initial positions
# ----------------------------
T1_gkPos = np.array([0.5, fieldWidth / 2])
T1_defPos = np.array([1.5, 4.5])
T1_str1Pos = np.array([2.5, 1.5])
T1_str2Pos = np.array([3.5, 3.0])

# ----------------------------
# Team 2 initial positions
# ----------------------------
T2_gkPos = np.array([fieldLength - 0.5, fieldWidth / 2])
T2_defPos = np.array([fieldLength - 1.5, 3.0])
T2_str1Pos = np.array([fieldLength - 2.5, 2.0])
T2_str2Pos = np.array([fieldLength - 2.5, 4.0])

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
ax.set_title('2D Robotic Soccer: Defender Tactics (KidSize Field)')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')

# Goals
ax.add_patch(plt.Rectangle((0, fieldWidth / 2 - goalWidth / 2), 0.2, goalWidth, color='red'))
ax.add_patch(plt.Rectangle((fieldLength - 0.2, fieldWidth / 2 - goalWidth / 2), 0.2, goalWidth, color='blue'))

# Robots (Team 1 = green/red/magenta, Team 2 = cyan/blue/yellow)
T1_gkPlot, = ax.plot([T1_gkPos[0]], [T1_gkPos[1]], 'go', markersize=8)
T1_defPlot, = ax.plot([T1_defPos[0]], [T1_defPos[1]], 'ro', markersize=8)
T1_str1Plot, = ax.plot([T1_str1Pos[0]], [T1_str1Pos[1]], 'mo', markersize=8)
T1_str2Plot, = ax.plot([T1_str2Pos[0]], [T1_str2Pos[1]], 'mo', markersize=8)

T2_gkPlot, = ax.plot([T2_gkPos[0]], [T2_gkPos[1]], 'co', markersize=8)
T2_defPlot, = ax.plot([T2_defPos[0]], [T2_defPos[1]], 'bo', markersize=8)
T2_str1Plot, = ax.plot([T2_str1Pos[0]], [T2_str1Pos[1]], 'yo', markersize=8)
T2_str2Plot, = ax.plot([T2_str2Pos[0]], [T2_str2Pos[1]], 'yo', markersize=8)

# Ball
ballPlot, = ax.plot([ballPos[0]], [ballPos[1]], 'm*', markersize=10)

# Scoreboard
scoreText = ax.text(fieldLength / 2 - 1.0, fieldWidth + 0.1,
                    f"Team 1: {score[0]}  –  Team 2: {score[1]}",
                    fontsize=10, fontweight='bold')

# Legend
ax.legend([T1_defPlot, T1_gkPlot, T1_str1Plot, T2_defPlot, T2_gkPlot, T2_str1Plot, ballPlot],
          ['T1 Defender', 'T1 GK', 'T1 Striker', 'T2 Defender', 'T2 GK', 'T2 Striker', 'Ball'],
          loc='upper center', ncol=3, fontsize=8)

# ----------------------------
# Simulation loop
# ----------------------------
for _ in range(numSteps):
    # --- Ball touch & random pass ---
    allRobots = np.array([
        T1_gkPos, T1_defPos, T1_str1Pos, T1_str2Pos,
        T2_gkPos, T2_defPos, T2_str1Pos, T2_str2Pos
    ])
    robotColors = [
        'green', 'red', 'magenta', 'magenta',
        'cyan', 'blue', 'yellow', 'yellow'
    ]

    dists = np.linalg.norm(allRobots - ballPos, axis=1)

    # Ball possession logic
    if ballOwner == 0:
        minDist = np.min(dists)
        idx = np.argmin(dists)
        if minDist <= collisionThreshold:
            ballOwner = idx + 1  # 1-indexed
            # random shoulder pass
            if idx < 4:  # Team 1
                angle = np.random.uniform(-np.pi / 6, np.pi / 6)
                dir_vec = np.array([fieldLength, fieldWidth / 2]) - ballPos
            else:        # Team 2
                angle = np.random.uniform(-np.pi / 6, np.pi / 6)
                dir_vec = np.array([0, fieldWidth / 2]) - ballPos

            dir_vec /= np.linalg.norm(dir_vec)
            R = np.array([[np.cos(angle), -np.sin(angle)],
                          [np.sin(angle),  np.cos(angle)]])
            dir_vec = R @ dir_vec
            ballVel = kickSpeed * dir_vec
            ballPlot.set_color(robotColors[idx])

    # Move ball
    ballPos += ballVel * dt
    # Decay velocity
    if np.linalg.norm(ballVel) > 0:
        ballVel *= 0.95
        if np.linalg.norm(ballVel) < 0.05:
            ballVel[:] = 0
            ballOwner = 0

    # ---------------- Team 1 movement ----------------
    if np.linalg.norm(ballPos - T1_gkPos) < 1.5:
        T1_gkPos += robotSpeed * dt * (ballPos - T1_gkPos) / np.linalg.norm(ballPos - T1_gkPos)
    T1_defPos[1] += robotSpeed * dt * np.sign(ballPos[1] - T1_defPos[1])
    T1_defPos[0] = np.clip(T1_defPos[0], 0, fieldLength / 2)

    distStr1 = np.linalg.norm(T1_str1Pos - ballPos)
    distStr2 = np.linalg.norm(T1_str2Pos - ballPos)
    if ballOwner == 0 or ballOwner not in [3, 4]:
        if distStr1 < distStr2:
            T1_str1Pos += robotSpeed * dt * (ballPos - T1_str1Pos) / distStr1
        else:
            T1_str2Pos += robotSpeed * dt * (ballPos - T1_str2Pos) / distStr2

    # ---------------- Team 2 movement ----------------
    if np.linalg.norm(ballPos - T2_gkPos) < 1.5:
        T2_gkPos += robotSpeed * dt * (ballPos - T2_gkPos) / np.linalg.norm(ballPos - T2_gkPos)
    T2_defPos[1] += robotSpeed * dt * np.sign(ballPos[1] - T2_defPos[1])
    T2_defPos[0] = np.clip(T2_defPos[0], fieldLength / 2, fieldLength - 1)

    distStr1 = np.linalg.norm(T2_str1Pos - ballPos)
    distStr2 = np.linalg.norm(T2_str2Pos - ballPos)
    if ballOwner == 0 or ballOwner not in [7, 8]:
        if distStr1 < distStr2:
            T2_str1Pos += robotSpeed * dt * (ballPos - T2_str1Pos) / distStr1
        else:
            T2_str2Pos += robotSpeed * dt * (ballPos - T2_str2Pos) / distStr2

    # ---------------- Prevent overlaps ----------------
    allPositions = [T1_gkPos, T1_defPos, T1_str1Pos, T1_str2Pos,
                    T2_gkPos, T2_defPos, T2_str1Pos, T2_str2Pos]
    for i in range(len(allPositions)):
        for j in range(i + 1, len(allPositions)):
            diff = allPositions[i] - allPositions[j]
            dist = np.linalg.norm(diff)
            if dist < 2 * robotRadius and dist > 0:
                adjust = (2 * robotRadius - dist) / 2
                move = (diff / dist) * adjust
                allPositions[i] += move
                allPositions[j] -= move
    (T1_gkPos, T1_defPos, T1_str1Pos, T1_str2Pos,
     T2_gkPos, T2_defPos, T2_str1Pos, T2_str2Pos) = allPositions

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
    T1_gkPlot.set_data([T1_gkPos[0]], [T1_gkPos[1]])
    T1_defPlot.set_data([T1_defPos[0]], [T1_defPos[1]])
    T1_str1Plot.set_data([T1_str1Pos[0]], [T1_str1Pos[1]])
    T1_str2Plot.set_data([T1_str2Pos[0]], [T1_str2Pos[1]])

    T2_gkPlot.set_data([T2_gkPos[0]], [T2_gkPos[1]])
    T2_defPlot.set_data([T2_defPos[0]], [T2_defPos[1]])
    T2_str1Plot.set_data([T2_str1Pos[0]], [T2_str1Pos[1]])
    T2_str2Plot.set_data([T2_str2Pos[0]], [T2_str2Pos[1]])

    ballPlot.set_data([ballPos[0]], [ballPos[1]])

    scoreText.set_text(f"Team 1: {score[0]}  –  Team 2: {score[1]}")
    plt.pause(0.05)

plt.ioff()
plt.show()