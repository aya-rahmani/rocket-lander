import numpy as np
import matplotlib.pyplot as plt
import config as cfg
from dynamics import derivatives
from integrator import rk4_step
from controller import PID, clamp


def descent_target(z):
    return -np.sqrt(2 * 2.0 * max(z, 0.0)) - 1.0


# Start 200 m off-target, 1000 m up, tilted 5 degrees, falling
state = np.array([200.0, 1000.0, np.radians(5.0), 0.0, -50.0, 0.0])
m = cfg.m0
t = 0.0

throttle_pid = PID(kp=120000.0, ki=20000.0, kd=8000.0)
pos_pid = PID(kp=0.006, ki=0.0, kd=0.05)
att_pid = PID(kp=1.5, ki=0.0, kd=1.5)
ts, xs, zs, thetas, deltas = [], [], [], [], []

for _ in range(8000):
    x, z, theta, vx, vz, omega = state

    # Outer loop: position error -> desired pitch
    theta_cmd = pos_pid.step(0.0 - x, cfg.dt)
    theta_cmd = clamp(theta_cmd, np.radians(-20.0), np.radians(20.0))

    # Inner loop: pitch error -> gimbal
    delta = att_pid.step(theta_cmd - theta, cfg.dt)
    delta = clamp(delta, -cfg.delta_max, cfg.delta_max)

    # Throttle loop
    T = m * cfg.g + throttle_pid.step(descent_target(z) - vz, cfg.dt)
    T = clamp(T, 0.0, cfg.T_max)

    state = rk4_step(derivatives, state, T, delta, m, cfg.dt)
    m -= (T / (cfg.Isp * cfg.g0)) * cfg.dt
    t += cfg.dt

    ts.append(t); xs.append(state[0]); zs.append(state[1])
    thetas.append(np.degrees(state[2])); deltas.append(np.degrees(delta))

    if state[1] <= 0.0:
        break

print("touchdown x: ", state[0])
print("touchdown z: ", state[1])
print("touchdown vx:", state[3])
print("touchdown vz:", state[4])
print("touchdown theta (deg):", np.degrees(state[2]))
print("fuel used:   ", cfg.m0 - m)

fig, ax = plt.subplots(2, 2, figsize=(11, 7))
ax[0,0].plot(xs, zs); ax[0,0].set_xlabel("x (m)"); ax[0,0].set_ylabel("z (m)"); ax[0,0].grid(True)
ax[0,1].plot(ts, thetas); ax[0,1].set_xlabel("t (s)"); ax[0,1].set_ylabel("theta (deg)"); ax[0,1].grid(True)
ax[1,0].plot(ts, xs); ax[1,0].set_xlabel("t (s)"); ax[1,0].set_ylabel("x (m)"); ax[1,0].grid(True)
ax[1,1].plot(ts, deltas); ax[1,1].set_xlabel("t (s)"); ax[1,1].set_ylabel("delta (deg)"); ax[1,1].grid(True)
plt.tight_layout()
plt.show()