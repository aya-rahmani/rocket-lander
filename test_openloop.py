import numpy as np
import matplotlib.pyplot as plt
import config as cfg
from sim import simulate

# Start upright at 1000 m, at rest
state0 = [0.0, 1000.0, 0.0, 0.0, 0.0, 0.0]

# Test B: constant thrust, gimbal held at +0.05 rad
ts, states, masses = simulate(
    state0,
    T_func=lambda t: 400000.0,
    delta_func=lambda t: 0.05,
    duration=10.0,
)

fig, ax = plt.subplots(1, 3, figsize=(13, 4))
ax[0].plot(ts, states[:, 1])
ax[0].set_xlabel("t (s)")
ax[0].set_ylabel("z (m)")
ax[0].grid(True)

ax[1].plot(ts, states[:, 0])
ax[1].set_xlabel("t (s)")
ax[1].set_ylabel("x (m)")
ax[1].grid(True)

ax[2].plot(ts, np.degrees(states[:, 2]))
ax[2].set_xlabel("t (s)")
ax[2].set_ylabel("theta (deg)")
ax[2].grid(True)

plt.tight_layout()
plt.show()