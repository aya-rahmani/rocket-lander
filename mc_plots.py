import numpy as np
import matplotlib.pyplot as plt

d = np.load("mc_results.npz")
ics = d["ics"]
pid_out, pid_ok = d["pid_out"], d["pid_ok"]
lqr_out, lqr_ok = d["lqr_out"], d["lqr_ok"]

fig, ax = plt.subplots(2, 2, figsize=(12, 9))

# Landing scatter: offset vs horizontal speed
for a, out, ok, name in [(ax[0, 0], pid_out, pid_ok, "PID"),
                         (ax[0, 1], lqr_out, lqr_ok, "LQR")]:
    a.scatter(out[ok, 0], out[ok, 1], s=8, alpha=0.5, label="success")
    a.scatter(out[~ok, 0], out[~ok, 1], s=8, alpha=0.7, color="crimson", label="failure")
    a.axvline(-10, ls="--", lw=0.8, color="grey")
    a.axvline(10, ls="--", lw=0.8, color="grey")
    a.set_xlabel("landing offset x (m)")
    a.set_ylabel("horizontal speed vx (m/s)")
    a.set_title(f"{name} - {100 * ok.mean():.1f}% success")
    a.legend(fontsize=8)
    a.grid(True, lw=0.3)

# Touchdown vertical speed
bins = np.linspace(-4, 0, 40)
ax[1, 0].hist(pid_out[:, 2], bins=bins, alpha=0.6, label="PID")
ax[1, 0].hist(lqr_out[:, 2], bins=bins, alpha=0.6, label="LQR")
ax[1, 0].axvline(-3, ls="--", lw=0.8, color="grey")
ax[1, 0].set_xlabel("touchdown vz (m/s)")
ax[1, 0].set_ylabel("runs")
ax[1, 0].legend()
ax[1, 0].grid(True, lw=0.3)

# Failure driver: initial offset vs final offset
ax[1, 1].scatter(np.abs(ics[lqr_ok, 0]), np.abs(lqr_out[lqr_ok, 0]),
                 s=8, alpha=0.5, label="LQR success")
ax[1, 1].scatter(np.abs(ics[~lqr_ok, 0]), np.abs(lqr_out[~lqr_ok, 0]),
                 s=8, alpha=0.7, color="crimson", label="LQR failure")
ax[1, 1].axhline(10, ls="--", lw=0.8, color="grey")
ax[1, 1].set_xlabel("initial offset |x0| (m)")
ax[1, 1].set_ylabel("landing offset |x| (m)")
ax[1, 1].legend(fontsize=8)
ax[1, 1].grid(True, lw=0.3)

plt.tight_layout()
plt.savefig("figures/monte_carlo.png", dpi=150)
plt.show()