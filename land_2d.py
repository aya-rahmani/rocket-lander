import numpy as np
import matplotlib.pyplot as plt
import config as cfg
from dynamics import derivatives
from integrator import rk4_step
from controller import Lander


def run(state0, max_steps=8000):
    """Simulate one landing. Returns time, states and mass history."""
    state = np.array(state0, dtype=float)
    lander = Lander()
    m = cfg.m0
    t = 0.0

    ts, states, deltas, Ts = [t], [state.copy()], [0.0], [0.0]

    for _ in range(max_steps):
        T, delta = lander.step(state, m, cfg.dt)

        state = rk4_step(derivatives, state, T, delta, m, cfg.dt)
        m -= (T / (cfg.Isp * cfg.g0)) * cfg.dt
        t += cfg.dt

        ts.append(t)
        states.append(state.copy())
        deltas.append(delta)
        Ts.append(T)

        if state[1] <= 0.0:
            break

    return np.array(ts), np.array(states), np.array(deltas), np.array(Ts), m


if __name__ == "__main__":
    state0 = [200.0, 1000.0, np.radians(5.0), 0.0, -50.0, 0.0]
    ts, states, deltas, Ts, m_final = run(state0)

    print("touchdown x:          ", states[-1, 0])
    print("touchdown z:          ", states[-1, 1])
    print("touchdown vx:         ", states[-1, 3])
    print("touchdown vz:         ", states[-1, 4])
    print("touchdown theta (deg):", np.degrees(states[-1, 2]))
    print("fuel used:            ", cfg.m0 - m_final)

    fig, ax = plt.subplots(2, 2, figsize=(11, 7))
    ax[0,0].plot(states[:, 0], states[:, 1]); ax[0,0].set_xlabel("x (m)"); ax[0,0].set_ylabel("z (m)"); ax[0,0].grid(True)
    ax[0,1].plot(ts, np.degrees(states[:, 2])); ax[0,1].set_xlabel("t (s)"); ax[0,1].set_ylabel("theta (deg)"); ax[0,1].grid(True)
    ax[1,0].plot(ts, states[:, 0]); ax[1,0].set_xlabel("t (s)"); ax[1,0].set_ylabel("x (m)"); ax[1,0].grid(True)
    ax[1,1].plot(ts, np.degrees(deltas)); ax[1,1].set_xlabel("t (s)"); ax[1,1].set_ylabel("delta (deg)"); ax[1,1].grid(True)
    plt.tight_layout()
    plt.show()