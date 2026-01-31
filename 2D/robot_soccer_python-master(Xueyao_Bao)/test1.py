import time
from robot_soccer_python.simulation2D import simulation2D, init_simulation
from robot_soccer_python.agents import Player
from robot_soccer_python.utils import Pose

sim = simulation2D(
    [Player(Pose(3, 3, 0), 2, 2, 0.20),
     Player(Pose(6, 3, 0), 2, 2, 0.20)],
    shockable=True,
    full_vision=True
)

t0 = time.time()
while time.time() - t0 < 3:
    sim.set_commands([(1.0, 0.0), (1.0, 0.0)])
    init_simulation(sim)
