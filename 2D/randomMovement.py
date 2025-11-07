import numpy as np
import matplotlib.pyplot as plt

# ==============================
# KidSize Soccer Simulation
# ==============================

# Field setup (KidSize: 9m x 6m)
fieldLength = 9.0
fieldWidth = 6.0
goalWidth = 1.0

# Simulation params
dt = 0.1
numSteps = 300

# Robot and ball properties
robotRadius = 0.2
ballRadius = 0.1
robotSpeed = 0.5
kickSpeed = 2.0
friction = 0.98

# Initial positions (x, y)
gkPos = np.array([0.5, fieldWidth / 2], dtype=float)
def1Pos = np.array([2.0, 2.0], dtype=float)
def2Pos = np.array([2.0, 4.0], dtype=float)
str1Pos = np.array([7.0, 2.0], dtype=float)
str2Pos = np.array([7.0, 4.0], dtype=float)
ballPos = np.array([4.5, 3.0], dtype=float)
ballVel = np.array([np.random.randn() * 0.1, np.random.randn() * 0.1], dtype=float)

# Plot setup
fig, ax = plt.subplots()
ax.set_xlim(0, fieldLength)
ax.set_ylim(0, fieldWidth)
ax.set_aspect('equal')
ax.set_title("KidSize Humanoid Soccer Simulation")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.grid(True)

# Draw field lines
# Outer boundary
ax.plot([0, fieldLength, fieldLength, 0, 0],
        [0, 0, fieldWidth, fieldWidth, 0], 'k-')
# Center line
ax.plot([fieldLength / 2, fieldLength / 2], [0, fieldWidth], 'k--')
# Goals
ax.plot([0, 0], [fieldWidth/2 - goalWidth/2, fieldWidth/2 + goalWidth/2], 'r-', linewidth=4)
ax.plot([fieldLength, fieldLength],
        [fieldWidth/2 - goalWidth/2, fieldWidth/2 + goalWidth/2], 'b-', linewidth=4)
# Center circle
center = plt.Circle((fieldLength/2, fieldWidth/2), 0.5, color='gray', fill=False)
ax.add_patch(center)

# Create players and ball
gkPlot, = ax.plot([gkPos[0]], [gkPos[1]], 'go', markersize=10, label="Goalkeeper")
def1Plot, = ax.plot([def1Pos[0]], [def1Pos[1]], 'bo', markersize=10, label="Defender 1")
def2Plot, = ax.plot([def2Pos[0]], [def2Pos[1]], 'bo', markersize=10, label="Defender 2")
str1Plot, = ax.plot([str1Pos[0]], [str1Pos[1]], 'ro', markersize=10, label="Striker 1")
str2Plot, = ax.plot([str2Pos[0]], [str2Pos[1]], 'ro', markersize=10, label="Striker 2")
ballPlot, = ax.plot([ballPos[0]], [ballPos[1]], 'm*', markersize=10, label="Ball")

ax.legend(loc="upper right")

# ==============================
# Simulation Loop
# ==============================
for t in range(numSteps):
    # --- Ball physics ---
    ballVel *= friction
    ballPos += ballVel * dt
    # Keep on field
    ballPos = np.clip(ballPos, [0, 0], [fieldLength, fieldWidth])

    # --- Robot positions array ---
    robotPositions = np.array([gkPos, def1Pos, def2Pos, str1Pos, str2Pos])
    dists = np.linalg.norm(robotPositions - ballPos, axis=1)
    closest_idx = np.argmin(dists)
    minDist = dists[closest_idx]

    # --- Kick logic: only if touching ---
    if minDist <= (robotRadius + ballRadius):
        if closest_idx == 0:  # GK: clear to right
            target = np.array([fieldLength/2, fieldWidth/2])
        elif closest_idx <= 2:  # Defenders: pass to striker
            target = np.array([6.0, np.random.uniform(2, 4)])
        else:  # Strikers: shoot to goal
            target = np.array([fieldLength, fieldWidth/2])
        dir_vec = target - ballPos
        if np.linalg.norm(dir_vec) > 0:
            dir_vec = dir_vec / np.linalg.norm(dir_vec)
            ballVel = dir_vec * kickSpeed

    # --- Move robots ---
    # Goalkeeper
    distGK = np.linalg.norm(ballPos - gkPos)
    if distGK < 1.5:
        dirGK = (ballPos - gkPos) / distGK
        gkPos += robotSpeed * dt * dirGK
    else:
        # Return to home near goal
        home = np.array([0.5, fieldWidth/2])
        dirGK = (home - gkPos)
        if np.linalg.norm(dirGK) > 0:
            dirGK /= np.linalg.norm(dirGK)
        gkPos += robotSpeed * dt * dirGK

    # Defenders
    for i, pos in enumerate([def1Pos, def2Pos]):
        dist = np.linalg.norm(ballPos - pos)
        if dist < 3.0:
            dir_ = (ballPos - pos) / dist
        else:
            # Patrol horizontally
            homeX = 2.0 + np.sin(0.1 * t + i) * 0.5
            homeY = 2.0 + i * 2.0
            dir_ = np.array([homeX, homeY]) - pos
            if np.linalg.norm(dir_) > 0:
                dir_ /= np.linalg.norm(dir_)
        pos += robotSpeed * dt * dir_
        if i == 0:
            def1Pos = pos
        else:
            def2Pos = pos

    # Strikers
    for i, pos in enumerate([str1Pos, str2Pos]):
        dist = np.linalg.norm(ballPos - pos)
        if dist < 3.0:
            dir_ = (ballPos - pos) / dist
        else:
            # Attack forward
            dir_ = np.array([1.0, 0.0])
        pos += robotSpeed * dt * dir_
        pos = np.clip(pos, [0, 0], [fieldLength, fieldWidth])
        if i == 0:
            str1Pos = pos
        else:
            str2Pos = pos

    # --- Update graphics ---
    gkPlot.set_data([gkPos[0]], [gkPos[1]])
    def1Plot.set_data([def1Pos[0]], [def1Pos[1]])
    def2Plot.set_data([def2Pos[0]], [def2Pos[1]])
    str1Plot.set_data([str1Pos[0]], [str1Pos[1]])
    str2Plot.set_data([str2Pos[0]], [str2Pos[1]])
    ballPlot.set_data([ballPos[0]], [ballPos[1]])

    plt.pause(0.05)

plt.show()
