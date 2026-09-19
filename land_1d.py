import numpy as np
import matplotlib.pyplot as plt
import config as cfg
from dynamics import derivatives
from integrator import rk4_step
from controller import PID, clamp


def descent_target(z):
    """Desired descent rate at altitude z: slower as the ground nears."""
    return -np.sqrt(2 * 2.0 * max(z, 0.0)) - 1.0


state = np.array([0.0, 1000.0, 0.0, 0.0, -50.0, 0.0])
m = cfg.m0
t = 0.0

pid = PID(kp=120000, ki=20000.0, kd=8000.0)

ts, zs, vzs, Ts = [], [], [], []

for _ in range(6000):
    z = state[1]
    vz = state[4]

    error = descent_target(z) - vz
    T = m * cfg.g + pid.step(error, cfg.dt)
    T = clamp(T, 0.0, cfg.T_max)

    state = rk4_step(derivatives, state, T, 0.0, m, cfg.dt)
    m -= (T / (cfg.Isp * cfg.g0)) * cfg.dt
    t += cfg.dt

    ts.append(t)
    zs.append(state[1])
    vzs.append(state[4])
    Ts.append(T)

    if state[1] <= 0.0:
        break

print("touchdown z:  ", state[1])
print("touchdown vz: ", state[4])
print("time:         ", t)
print("fuel used:    ", cfg.m0 - m)

fig, ax = plt.subplots(1, 3, figsize=(13, 4))
ax[0].plot(ts, zs); ax[0].set_xlabel("t (s)"); ax[0].set_ylabel("z (m)"); ax[0].grid(True)
ax[1].plot(ts, vzs); ax[1].set_xlabel("t (s)"); ax[1].set_ylabel("vz (m/s)"); ax[1].grid(True)
ax[2].plot(ts, Ts); ax[2].set_xlabel("t (s)"); ax[2].set_ylabel("T (N)"); ax[2].grid(True)
plt.tight_layout()
plt.show()