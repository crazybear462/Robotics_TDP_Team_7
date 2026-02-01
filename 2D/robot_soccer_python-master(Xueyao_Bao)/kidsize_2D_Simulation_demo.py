import pygame
from math import atan2, pi
from robot_soccer_python.simulation2D import simulation2D
from robot_soccer_python.agents import Player, Environment
from robot_soccer_python.utils import Pose
from robot_soccer_python.constants import *


# =========================
# Low-level helpers
# =========================
def wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while a > pi:
        a -= 2 * pi
    while a < -pi:
        a += 2 * pi
    return a

GK_FSM = {"state": "HOME"}  # HOME / TRACK / CHARGE
STRIKER_FSM = {"state": "SEEK"}  # SEEK / GET_BEHIND / DRIBBLE / SHOOT / RECOVER




def clamp(v: float, lo: float, hi: float) -> float:
    """Clamp value to [lo, hi]."""
    return max(lo, min(hi, v))


def go_to(pose, tx: float, ty: float, vmax: float, wmax: float):
    """
    Simple point-tracking controller:
    - turn toward target
    - move forward more when aligned, less when misaligned
    Returns (v, w).
    """
    dx, dy = tx - pose.position.x, ty - pose.position.y
    ang = atan2(dy, dx)
    err = wrap(ang - pose.rotation)
    w = max(-wmax, min(wmax, 2.5 * err))
    v = vmax * max(0.0, 1.0 - abs(err) / pi)
    return v, w


# =========================
# Field geometry helper
# =========================
def get_field_geometry():
    """Return field geometry in meters."""
    x0, y0 = BORDER_M, BORDER_M
    x1, y1 = BORDER_M + PITCH_LENGTH_M, BORDER_M + PITCH_WIDTH_M
    midx, midy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    return x0, y0, x1, y1, midx, midy


# =========================
# Role policies (return (v, w))
# =========================
def policy_goalkeeper(sim, idx: int, bx: float, by: float, x0: float, y0: float, y1: float):
    """
    GK FSM (more like real robot soccer):
      HOME  : stay near goal center
      TRACK : follow ball laterally along goal line when ball is in our half / defensive zone
      CHARGE: if ball is very close to goal, move out to clear it
    """
    gk = sim.player[idx]
    state = GK_FSM["state"]

    # --- parameters you can tune ---
    gk_x = 0.40          # keeper x position from goal line
    y_margin = 0.60      # keep away from top/bottom boundaries
    danger_enter = x0 + 1.8   # enter danger zone (meters)
    danger_exit  = x0 + 2.2   # exit danger zone (hysteresis)
    charge_dist  = 1.1        # only charge if keeper is close enough
    clear_forward = 1.2       # how far to push ball forward when clearing

    # --- distances ---
    dx = bx - gk.pose.position.x
    dy = by - gk.pose.position.y
    dgk_ball = (dx*dx + dy*dy) ** 0.5

    # --- state transitions (with hysteresis) ---
    if state != "CHARGE":
        # enter CHARGE only in deep danger and keeper near enough
        if bx < danger_enter and dgk_ball < charge_dist:
            state = "CHARGE"
        else:
            # TRACK if ball is somewhat in our half/defensive side
            if bx < (x0 + PITCH_LENGTH_M * 0.55):
                state = "TRACK"
            else:
                state = "HOME"
    else:
        # leave CHARGE only when ball is safely away
        if bx > danger_exit:
            state = "TRACK"

    GK_FSM["state"] = state

    # --- actions ---
    vmax, wmax = gk.max_linear_speed, gk.max_angular_speed

    if state == "HOME":
        # sit at goal center
        tx = x0 + gk_x
        ty = (y0 + y1) / 2.0
        return go_to(gk.pose, tx, ty, vmax, wmax)

    if state == "TRACK":
        # follow ball y along goal line
        tx = x0 + gk_x
        ty = clamp(by, y0 + y_margin, y1 - y_margin)
        return go_to(gk.pose, tx, ty, vmax, wmax)

    # CHARGE: clear the ball
    # Move to a point slightly "through" the ball toward forward direction (+x),
    # so collision tends to push ball away from our goal.
    tx = bx + clear_forward
    ty = clamp(by, y0 + y_margin, y1 - y_margin)
    return go_to(gk.pose, tx, ty, vmax, wmax)



def policy_defender(sim, idx: int, bx: float, by: float, x0: float, y0: float, y1: float, midy: float):
    """DEF blocks line by staying on a defensive x-line and tracking ball y moderately."""
    df = sim.player[idx]
    target_x = x0 + 1.8
    target_y = midy + 0.6 * (by - midy)
    target_y = clamp(target_y, y0 + 0.6, y1 - 0.6)
    return go_to(df.pose, target_x, target_y, df.max_linear_speed, df.max_angular_speed)


def policy_striker(
    sim,
    idx: int,
    bx: float,
    by: float,
    goal_x: float,
    goal_y: float,
    y0: float,
    y1: float,
):
    """
    Striker FSM (more like robot soccer):
      SEEK       : move toward ball area (light chase)
      GET_BEHIND : reposition behind the ball relative to goal direction
      DRIBBLE    : push ball forward while keeping it in front
      SHOOT      : in shooting zone, aggressively push to goal center
      RECOVER    : if ball is in our half / we lost control, chase to regain
    """
    p = sim.player[idx]
    state = STRIKER_FSM["state"]

    # --- tunable parameters (KidSize-friendly) ---
    y_margin = 0.55

    near_ball = 0.45         # close enough to influence ball
    contact_ball = 0.28      # very close -> likely contact/push
    far_ball = 1.20          # too far -> stop "dribble logic"

    # "shooting zone": ball within last X meters before opponent goal line
    shoot_zone_x = 2.2       # meters from goal line

    # alignment thresholds (approach from correct side)
    lateral_bad = 0.40       # meters lateral error -> reposition
    behind_offset = 0.40     # meters behind the ball
    push_ahead = 0.90        # meters ahead of the ball while dribbling
    shoot_ahead = 1.30       # meters ahead when shooting

    # --- geometry: direction from ball to goal (unit vector) ---
    dxg, dyg = (goal_x - bx), (goal_y - by)
    distg = (dxg * dxg + dyg * dyg) ** 0.5 + 1e-9
    ux, uy = dxg / distg, dyg / distg

    # --- striker-ball distance ---
    sx, sy = p.pose.position.x, p.pose.position.y
    dxs, dys = bx - sx, by - sy
    ds = (dxs * dxs + dys * dys) ** 0.5

    # --- lateral error: are we on the "behind ball" corridor? ---
    # perp vector to ball->goal is (-uy, ux)
    lateral_err = dxs * (-uy) + dys * (ux)

    # --- quick state of play features ---
    in_shoot_zone = (goal_x - bx) < shoot_zone_x
    ball_in_our_half = bx < (BORDER_M + PITCH_LENGTH_M * 0.5)

    # --- compute targets ---
    behind_x = bx - ux * behind_offset
    behind_y = clamp(by - uy * behind_offset, y0 + y_margin, y1 - y_margin)

    dribble_x = bx + ux * push_ahead
    dribble_y = clamp(by + uy * push_ahead, y0 + y_margin, y1 - y_margin)

    shoot_x = bx + ux * shoot_ahead
    shoot_y = clamp(goal_y, y0 + y_margin, y1 - y_margin)

    # --- state transitions (with stability) ---
    if state == "SHOOT":
        # leave SHOOT if ball moved away from shooting zone or we lost proximity
        if (not in_shoot_zone) or ds > near_ball:
            state = "GET_BEHIND"

    elif state == "DRIBBLE":
        # if we lost the ball or approach angle got bad -> reposition
        if ds > near_ball or abs(lateral_err) > lateral_bad:
            state = "GET_BEHIND"
        # if we reach shooting zone and are close -> shoot
        elif in_shoot_zone and ds < near_ball:
            state = "SHOOT"

    elif state == "GET_BEHIND":
        # once aligned and close enough, start dribbling
        if ds < near_ball and abs(lateral_err) < (lateral_bad * 0.8):
            state = "DRIBBLE"
        # if ball far away -> recover
        elif ds > far_ball:
            state = "RECOVER"

    elif state == "RECOVER":
        # if we got closer, try to get behind again
        if ds < far_ball:
            state = "GET_BEHIND"

    else:  # SEEK
        # SEEK is a soft default
        if ds > far_ball:
            state = "RECOVER"
        else:
            state = "GET_BEHIND"

    # extra realism: if ball is deep in our half, prioritize recover rather than fancy dribble
    if ball_in_our_half and ds > near_ball:
        state = "RECOVER"

    STRIKER_FSM["state"] = state

    # --- actions ---
    vmax, wmax = p.max_linear_speed, p.max_angular_speed

    if state == "RECOVER":
        # simple chase to regain (do not overthink)
        ty = clamp(by, y0 + y_margin, y1 - y_margin)
        return go_to(p.pose, bx, ty, vmax, wmax)

    if state == "GET_BEHIND":
        return go_to(p.pose, behind_x, behind_y, vmax, wmax)

    if state == "DRIBBLE":
        # keep speed a bit lower near contact to avoid blasting the ball sideways
        v, w = go_to(p.pose, dribble_x, dribble_y, vmax, wmax)
        if ds < contact_ball:
            v *= 0.75
        return v, w

    # SHOOT
    v, w = go_to(p.pose, shoot_x, shoot_y, vmax, wmax)
    return v, w



def policy_wing_support(sim, idx: int, bx: float, by: float):
    """WING: simple support by chasing ball."""
    wing = sim.player[idx]
    return go_to(wing.pose, bx, by, wing.max_linear_speed, wing.max_angular_speed)


def policy_chase(sim, idx: int, bx: float, by: float):
    """Generic chase-the-ball policy (used for Yellow team)."""
    p = sim.player[idx]
    return go_to(p.pose, bx, by, p.max_linear_speed, p.max_angular_speed)


# =========================
# Command builder
# =========================
def build_commands(sim, x0: float, y0: float, x1: float, y1: float, midy: float):
    """
    Build commands list in the SAME order as sim.player:
    Red: 0..3, Yellow: 4..7
    """
    bx, by = sim.ball.pose.position.x, sim.ball.pose.position.y

    cmds = []

    # Red team roles
    cmds.append(policy_goalkeeper(sim, 0, bx, by, x0, y0, y1))
    cmds.append(policy_defender(sim, 1, bx, by, x0, y0, y1, midy))

    # Attack right goal center
    goal_x = x1
    goal_y = (y0 + y1) / 2.0
    cmds.append(policy_striker(sim, 2, bx, by, goal_x, goal_y, y0, y1))

    cmds.append(policy_wing_support(sim, 3, bx, by))

    # Yellow team: minimal policy (all chase ball)
    for i in [4, 5, 6, 7]:
        cmds.append(policy_chase(sim, i, bx, by))

    return cmds


# =========================
# Main
# =========================
def main():
    x0, y0, x1, y1, midx, midy = get_field_geometry()

    # ---- 4v4 spawn (meters) ----
    # Red: 0..3 (left team), Yellow: 4..7 (right team)
    red = [
        Player(Pose(x0 + 0.4, midy, 0), 1.0, 1.8, 0.20),          # GK
        Player(Pose(x0 + 2.0, midy, 0), 1.0, 1.8, 0.20),          # DEF
        Player(Pose(midx - 0.5, midy - 1.0, 0), 1.0, 1.8, 0.20),  # STR
        Player(Pose(midx - 0.5, midy + 1.0, 0), 1.0, 1.8, 0.20),  # WING
    ]
    yellow = [
        Player(Pose(x1 - 0.4, midy, pi), 1.0, 1.8, 0.20),
        Player(Pose(x1 - 2.0, midy, pi), 1.0, 1.8, 0.20),
        Player(Pose(midx + 0.5, midy - 1.0, pi), 1.0, 1.8, 0.20),
        Player(Pose(midx + 0.5, midy + 1.0, pi), 1.0, 1.8, 0.20),
    ]

    sim = simulation2D(red + yellow, shockable=True, full_vision=True)

    pygame.init()
    window = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("KidSize 4v4 - robot_soccer_python base")
    clock = pygame.time.Clock()
    env = Environment(window)

    running = True
    while running:

        print("STR state:", STRIKER_FSM["state"])


        clock.tick(int(FREQUENCY))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Build and apply commands (policy layer)
        cmds = build_commands(sim, x0, y0, x1, y1, midy)

        sim.set_commands(cmds)
        sim.update()
        sim.draw(window, env)
        pygame.display.update()

    pygame.quit()


if __name__ == "__main__":
    main()
