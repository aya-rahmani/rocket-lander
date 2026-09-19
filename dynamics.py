import numpy as np
import config as cfg


def derivatives(state, T, delta, m):
    x, z, theta, vx, vz, omega = state
    ax = T * np.sin(theta + delta) / m
    az = T * np.cos(theta + delta) / m - cfg.g
    alpha = T * cfg.L * np.sin(delta) / cfg.I

    return np.array([vx, vz, omega, ax, az, alpha])