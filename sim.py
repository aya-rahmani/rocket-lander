import numpy as np
import config as cfg
from dynamics import derivatives
from integrator import rk4_step


def simulate(state0, T_func, delta_func, duration):
    """
    steps the rocket forward and records the trajectory.
    T_func and delta_func take time t and return thrust and gimbal.
    """
    n = int(duration / cfg.dt)
    state = np.array(state0, dtype=float)
    m = cfg.m0
    t = 0.0

    ts = np.zeros(n + 1)
    states = np.zeros((n + 1, 6))
    masses = np.zeros(n + 1)

    ts[0] = t
    states[0] = state
    masses[0] = m

    for i in range(n):
        T = T_func(t)
        delta = delta_func(t)

        state = rk4_step(derivatives, state, T, delta, m, cfg.dt)
        m = m - (T / (cfg.Isp * cfg.g0)) * cfg.dt
        t = t + cfg.dt

        ts[i + 1] = t
        states[i + 1] = state
        masses[i + 1] = m

    return ts, states, masses