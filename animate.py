import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import land_2d

state0 = [200.0, 1000.0, np.radians(5.0), 0.0, -50.0, 0.0]
ts, states, deltas, Ts, m_final = land_2d.run(state0)

# Subsample so the animation runs at a sensible speed, keeping the final frame
step = 5
idx = list(range(0, len(ts), step))
if idx[-1] != len(ts) - 1:
    idx.append(len(ts) - 1)
ts, states, deltas, Ts = ts[idx], states[idx], deltas[idx], Ts[idx]

# Hold the final frame so the result is readable
hold = 20
ts = np.concatenate([ts, np.repeat(ts[-1], hold)])
states = np.concatenate([states, np.repeat(states[-1:], hold, axis=0)])
deltas = np.concatenate([deltas, np.repeat(deltas[-1], hold)])
Ts = np.concatenate([Ts, np.repeat(Ts[-1], hold)])

L_body = 25.0
T_max_draw = Ts.max() if Ts.max() > 0 else 1.0
VIEW = 220.0           # half-width of the visible window, m

fig, ax = plt.subplots(figsize=(7, 7))
ax.set_aspect("equal")
ax.set_xlabel("x (m)")
ax.set_ylabel("z (m)")
ax.grid(True, lw=0.3)
ax.axhline(0, color="black", lw=1.2)
ax.plot(0, 0, marker="x", ms=10, color="black")

trail, = ax.plot([], [], lw=0.8, ls="--", color="grey")
body, = ax.plot([], [], lw=4, color="#333333")
flame, = ax.plot([], [], lw=3, color="#D85A30")
label = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top", fontsize=9)


def frame(i):
    x, z, th = states[i, 0], states[i, 1], states[i, 2]

    ux, uz = np.sin(th), np.cos(th)

    # Drawn standing on its base at (x, z)
    body.set_data([x, x + 2 * L_body * ux],
                  [z, z + 2 * L_body * uz])

    a = th + deltas[i]
    scale = 50.0 * Ts[i] / T_max_draw
    scale = min(scale, max(z, 1.0))
    flame.set_data([x, x - scale * np.sin(a)],
                   [z, z - scale * np.cos(a)])

    trail.set_data(states[:i + 1, 0], states[:i + 1, 1])

    # Camera follows the vehicle, clamped so the ground stays in view
    cz = max(z, VIEW * 0.4)
    ax.set_xlim(x - VIEW, x + VIEW)
    ax.set_ylim(cz - VIEW, cz + VIEW)

    label.set_text(f"t = {ts[i]:5.1f} s\n"
                   f"z = {z:6.1f} m\n"
                   f"vz = {states[i, 4]:5.2f} m/s\n"
                   f"theta = {np.degrees(th):5.1f} deg")
    return body, flame, trail, label


anim = FuncAnimation(fig, frame, frames=len(ts), interval=30, blit=False)
anim.save("figures/landing.gif", writer="pillow", fps=20)
plt.show()