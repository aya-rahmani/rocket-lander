import numpy as np
import config as cfg
import land_2d
import land_lqr

N = 500
rng = np.random.default_rng(42)


def sample_initial_conditions(n):
    """Dispersed entry states. Columns: x, z, theta, vx, vz, omega."""
    s = np.zeros((n, 6))
    s[:, 0] = rng.normal(200.0, 50.0, n)            # x, m
    s[:, 1] = rng.normal(1000.0, 50.0, n)           # z, m
    s[:, 2] = np.radians(rng.normal(5.0, 5.0, n))   # theta, rad
    s[:, 3] = rng.normal(0.0, 5.0, n)               # vx, m/s
    s[:, 4] = rng.normal(-50.0, 5.0, n)             # vz, m/s
    s[:, 5] = np.radians(rng.normal(0.0, 1.0, n))   # omega, rad/s
    return s


def success(x, vx, vz, theta):
    return (abs(x) < 10.0 and abs(vx) < 2.0
            and abs(vz) < 3.0 and abs(np.degrees(theta)) < 5.0)


def evaluate(run_fn, ics, label):
    out = np.zeros((len(ics), 6))   # x, vx, vz, theta, fuel, time
    ok = np.zeros(len(ics), dtype=bool)

    for i, s0 in enumerate(ics):
        ts, states, deltas, Ts, m_final = run_fn(s0)
        final = states[-1]
        landed = final[1] <= 0.0

        out[i] = [final[0], final[3], final[4], final[2],
                  cfg.m0 - m_final, ts[-1]]
        ok[i] = landed and success(final[0], final[3], final[4], final[2])

        if (i + 1) % 50 == 0:
            print(f"{label}: {i + 1}/{len(ics)}")

    return out, ok


if __name__ == "__main__":
    ics = sample_initial_conditions(N)

    pid_out, pid_ok = evaluate(land_2d.run, ics, "PID")
    lqr_out, lqr_ok = evaluate(land_lqr.run, ics, "LQR")

    np.savez("mc_results.npz",
             ics=ics,
             pid_out=pid_out, pid_ok=pid_ok,
             lqr_out=lqr_out, lqr_ok=lqr_ok)

    for name, out, ok in [("PID", pid_out, pid_ok), ("LQR", lqr_out, lqr_ok)]:
        print(f"\n--- {name} ---")
        print(f"success rate:      {100 * ok.mean():.1f}%")
        print(f"median |x|:        {np.median(np.abs(out[:, 0])):.2f} m")
        print(f"median vz:         {np.median(out[:, 2]):.2f} m/s")
        print(f"95th pct |x|:      {np.percentile(np.abs(out[:, 0]), 95):.2f} m")
        print(f"mean fuel:         {out[:, 4].mean():.0f} kg")

    print("\n--- LQR failures vs successes ---")
    f = ~lqr_ok
    print(f"failed runs:          {f.sum()}")
    print(f"mean |x0| failed:     {np.abs(ics[f, 0]).mean():.1f} m")
    print(f"mean |x0| succeeded:  {np.abs(ics[~f, 0]).mean():.1f} m")
    print(f"mean |theta0| failed: {np.degrees(np.abs(ics[f, 2])).mean():.2f} deg")
    print(f"mean |theta0| succ:   {np.degrees(np.abs(ics[~f, 2])).mean():.2f} deg")