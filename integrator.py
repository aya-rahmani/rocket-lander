import numpy as np


def rk4_step(f, state, T, delta, m, dt):
    k1 = f(state, T, delta, m)
    k2 = f(state + 0.5 * dt * k1, T, delta, m)
    k3 = f(state + 0.5 * dt * k2, T, delta, m)
    k4 = f(state + dt * k3, T, delta, m)

    return state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)