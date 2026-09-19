import numpy as np
import matplotlib.pyplot as plt
import config as cfg
from dynamics import derivatives
from integrator import rk4_step
from controller import PID, clamp

# Start tilted 10 degrees, hovering, asked to return to upright
state = np.array([0.0, 1000.0, np.radians(10.0), 0.0, 0.0, 0.0])
m = cfg.m0
t = 0.0

theta_cmd = 0.0
att_pid = PID(kp=0.5, ki=0.0, kd=1.5)

ts, thetas, deltas = [], [], []

for _ in range(2000):
    theta = state[2]

    error = theta_cmd - theta
    delta = att_pid.step(error, cfg.dt)
    delta = clamp(delta, -cfg.delta_max, cfg.delta_max)

    T = m * cfg.g          # hover thrust, keeps altitude roughly fixed
    state = rk4_step(derivatives, state, T, delta, m, cfg.dt)
    m -= (T / (cfg.Isp * cfg.g0)) * cfg.dt
    t += cfg.dt

    ts.append(t)
    thetas.append(np.degrees(state[2]))
    deltas.append(np.degrees(delta))

print("final theta (deg):", np.degrees(state[2]))
print("final omega:      ", state[5])

fig, ax = plt.subplots(1, 2, figsize=(11, 4))
ax[0].plot(ts, thetas); ax[0].set_xlabel("t (s)"); ax[0].set_ylabel("theta (deg)"); ax[0].grid(True)
ax[1].plot(ts, deltas); ax[1].set_xlabel("t (s)"); ax[1].set_ylabel("delta (deg)"); ax[1].grid(True)
plt.tight_layout()
plt.show()