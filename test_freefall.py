import numpy as np
import config as cfg
from dynamics import derivatives
from integrator import rk4_step

state = np.array([0.0, 1000.0, 0.0, 0.0, 0.0, 0.0])
T = 0.0
delta = 0.0
m = cfg.m0

t = 0.0
for _ in range(1000):
    state = rk4_step(derivatives, state, T, delta, m, cfg.dt)
    t += cfg.dt

exact = 1000.0 - 0.5 * cfg.g * t**2
print("simulated z:", state[1])
print("exact z:    ", exact)
print("error:      ", abs(state[1] - exact))