from math import pi

# ===== KidSize geometry in meters =====
PITCH_LENGTH_M = 9.0
PITCH_WIDTH_M  = 6.0
BORDER_M       = 1.0

GOAL_WIDTH_M   = 2.6
GOAL_DEPTH_M   = 0.6

CENTER_CIRCLE_D_M = 1.5

# ===== scaling (keep 100 px = 1 m) =====
M2PIX = 100.0
PIX2M = 1.0 / M2PIX

# Screen includes border strip on all sides
SCREEN_WIDTH  = int((PITCH_LENGTH_M + 2 * BORDER_M) * M2PIX)   # 11m -> 1100px
SCREEN_HEIGHT = int((PITCH_WIDTH_M  + 2 * BORDER_M) * M2PIX)   # 8m  -> 800px

# ===== timing =====
FREQUENCY = 60.0
SAMPLE_TIME = 1.0 / FREQUENCY

# ===== physics =====
GRAVITY_ACCLERATION = 9.8

# Ball Parameters (KidSize ball ~14cm diameter => radius 0.07m)
FRICTION_SLOWDOWN = 0.5
REFLECTION = pi
BALL_MASS = 0.450
RADIUS_BALL = 0.07
FACTOR_FRICTION = GRAVITY_ACCLERATION * FRICTION_SLOWDOWN * SAMPLE_TIME / 100

# players Parameters
PLAYER_MASS = 75
BACK_SPEED_COLISION = -0.1

# Vision Parameters
RADIAN_TO_DEGREE = 180 / pi

# colors
RED_COLOR = (255,0,0)
GRAY_COLOR = (50, 50, 50)
YELLOW_COLOR = (238, 203, 0)
BLACK_COLOR = (0, 0, 0)
WHITE_COLOR = (255,255,255)
