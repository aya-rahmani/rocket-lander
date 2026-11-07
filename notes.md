# Rocket lander — working notes

## Conventions (fixed)
- World frame: x horizontal, z vertical up. Gravity along −z.
- Deviation from aerospace NED (z down), chosen so altitude stays positive.
- theta: pitch angle from vertical, positive when nose tilts toward +x.
- delta: gimbal angle from body axis, positive produces positive theta-ddot.
- State vector: [x, z, theta, vx, vz, omega]

## Equations of motion
- ax = T·sin(theta + delta) / m
- az = T·cos(theta + delta) / m − g
- alpha = T·L·sin(delta) / I
- mdot = −T / (Isp·g0)

## Day 2 — validation
- Freefall test (T=0, 1000 m, 10 s): error 2.04e-11 at dt = 0.01. Gate passed.

## Day 3 — open-loop tests
- Test A (T = 400 kN, delta = 0): climbs to 1185 m, x and theta stay at zero. Correct.
- Test B (T = 400 kN, delta = +0.05 rad): theta reaches +343° in 10 s, x drifts to 253 m,
  z peaks then falls as thrust rotates past horizontal.
- Sign check: positive delta gives positive theta. Matches the day 1 convention.
  No sign flip needed in dynamics.py.

## Day 4 — vertical control
- Descent target: vz = -sqrt(2*2*z) - 1, so the target rate shrinks near the ground.
- Gain progression: P only -4.64 m/s; +D -4.33; kp 120k -2.33; +ki 20k -1.68 m/s.
- Final: kp=120000, ki=20000, kd=8000. Fuel 3240 kg, 29.4 s.
- I term closed the residual lag; D contributed little. Faster landings cost fuel.

## Day 5 — attitude loop
- Inner loop: theta error -> gimbal command. Gains kp=0.5, ki=0, kd=1.5.
- 10 deg initial tilt settles to 0.005 deg, omega ~3e-5. No oscillation.
- Delta saturates at -15 deg immediately, then overshoots positive to brake the rate.
- Settling ~10 s. May be too slow once the position loop wraps around it.

## Day 6 — 2D cascade
- Three loops: position -> theta_cmd -> delta, plus independent throttle.
- First attempt (pos kp=0.02, kd=0.12): landed 3.2 m off, but delta oscillated
  full-scale +/-15 deg every timestep from t=13. Cascade instability: outer loop
  faster than the inner attitude loop could follow.
- Fix: slowed outer loop to kp=0.006, kd=0.05. Delta smooth, one brief saturation.
- Cost: landing offset 3.2 -> 6.8 m, vx -0.59 -> -1.11 m/s. Accepted: the faster
  tuning was not physically flyable.

## Day 7 — tuning
- Tried att kp=2.0 kd=4.0: better tracking but full-scale gimbal chatter from t=7.
  High D amplifies variation in the theta command from the outer loop.
- Final: att kp=1.5 ki=0 kd=1.5, pos kp=0.006 ki=0 kd=0.05, throttle unchanged.
- Result: 4.75 m offset, vx -0.78, vz -1.66, theta 0.59 deg, fuel 3270 kg.
- theta_cmd saturates at the -20 deg clamp for ~3 s while killing the 200 m offset.

## Day 8 — refactor and README
- Moved all gains into config.py. Controller logic moved into a Lander class in
  controller.py, so land_1d and land_2d no longer duplicate the cascade.
- land_2d.py rewritten around an importable run() returning ts, states, deltas, Ts, m.
  Plotting moved under `if __name__ == "__main__"` so Monte Carlo can call run()
  without triggering a figure.
- Verified the refactor changed nothing: 4.746 m vs 4.751 m, vx -0.7752 vs -0.7760.
  The small difference is the initial state now being recorded before the first step.
- README v1 written: problem, model, conventions, control architecture with the
  tuning rationale, results, validation, file list, limitations.

## Day 9 — linearization about hover

Operating point: theta = 0, delta = 0, T = mg. Small-angle approximations
sin(a) ~ a, cos(a) ~ 1.

- ax = T*sin(theta+delta)/m  ->  g*theta + g*delta
- az = T*cos(theta+delta)/m - g  ->  dT/m   (writing T = mg + dT; the g terms cancel)
- alpha = T*L*sin(delta)/I  ->  (m*g*L/I)*delta

Inputs: u = [dT, delta]. States: s = [x, z, theta, vx, vz, omega].

A (6x6), columns ordered x, z, theta, vx, vz, omega:
  row 1 (xdot):     1 in col 4
  row 2 (zdot):     1 in col 5
  row 3 (thetadot): 1 in col 6
  row 4 (vxdot):    g in col 3
  rows 5, 6:        all zero

B (6x2), columns ordered dT, delta:
  row 4 (vxdot):    g in col 2
  row 5 (vzdot):    1/m in col 1
  row 6 (omegadot): m*g*L/I in col 2
  rows 1-3:         all zero

Note: A is nearly empty. The only state-to-state coupling is g in row 4, tilt
producing horizontal acceleration. Everything else enters through B. That is the
underactuation appearing in the algebra.

## Day 10 — LQR

Built A and B from the day 9 linearization. K from scipy solve_continuous_are.

Structure of K: row 1 (thrust) is nonzero only in the z and vz columns; row 2
(gimbal) only in x, theta, vx, omega. The two channels decoupled without being
told to. The PID cascade had that structure designed in by hand.

Q tuning, all with R = diag([1e-9, 1e2]):
- Q = diag([1, 1, 10, 1, 1, 10]): total failure. theta reached -460 deg, impact at
  -115 m/s. Cause: K[2,1] = 0.1 against x = 200 m gives delta = -20 rad, saturated
  by a factor of 76 from t=0. LQR has no knowledge of actuator limits.
- Q = diag([5e-4, 1, 400, 0.05, 1, 400]): stable but sloppy. 39.9 m off, vx -3.54.
  Overcorrected; x barely weighted at all.
- Q = diag([0.01, 1, 400, 0.5, 1, 400]): 9.5 m, vx -1.23, but vz -3.81. Vertical
  channel tracking too loosely.
- Final Q = diag([0.01, 1, 400, 0.5, 60, 400]): 8.02 m, vx -1.04, vz -1.83,
  theta 0.64 deg, fuel 3259 kg. Gimbal saturates only for the first 2 s.

Comparison at identical initial conditions: PID 4.75 m / vz -1.66; LQR 8.02 m /
vz -1.83. PID wins on every metric. Caveat: the PID had six gains tuned by hand
over two days against this one scenario; the LQR had four Q weights adjusted in
about fifteen minutes. Monte Carlo on dispersed conditions is the fairer test.

Peak tilt reaches 28.5 deg, well outside where sin(a) ~ a holds. The linear
controller is flying a plant it no longer accurately models, and still converges.

## Day 11 — Monte Carlo
- 500 dispersed entry conditions, seeded (rng 42), identical set for both controllers.
  Gaussian dispersions: x 200±50 m, z 1000±50 m, theta 5±5 deg, vx 0±5 m/s,
  vz -50±5 m/s, omega 0±1 deg/s.
- Success criteria: |x| < 10 m, |vx| < 2 m/s, |vz| < 3 m/s, |theta| < 5 deg.
- PID: 95.0% success, median |x| 4.61 m, 95th pct 9.98 m, median vz -1.67, fuel 3264 kg.
- LQR: 82.2% success, median |x| 7.75 m, 95th pct 11.96 m, median vz -1.85, fuel 3259 kg.
- 89 LQR failures. Mean |x0| for failures 260.0 m vs 186.2 m for successes.
  Mean |theta0| 6.45 vs 5.81 deg, which is a much weaker separation.
  Conclusion: failures are driven by initial horizontal offset, not initial tilt.
  Same saturation mode as the first Q attempt, in milder form: large x error ->
  gimbal command beyond the limit -> no authority left for attitude.
- The PID clamps theta_cmd explicitly, so its equivalent error is bounded by
  construction rather than by cost weights. That looks structural rather than a
  question of tuning effort.
- Results cached in mc_results.npz so plots can be regenerated without rerunning
  1000 simulations.

## Day 12 — analysis plots
- figures/monte_carlo.png: landing scatter per controller, touchdown vz histogram,
  and LQR initial-offset vs landing-offset scatter.
- The initial-offset panel is the clearest evidence for the saturation argument:
  landing error rises monotonically with |x0| and the failures sit exactly where
  the trend crosses the 10 m criterion.
- Unexpected: in both controllers every run lies on a single straight line in
  (landing offset, vx). Neither controller converges in x before touchdown; the
  residual offset and residual drift are locked in a fixed ratio set by the ~29 s
  descent timeline. Dispersions move runs along that line, never off it.
  Implication: horizontal control is time-limited by the vertical profile. A
  two-phase guidance law with a terminal correction phase would break the coupling.
- vz histograms are narrow for both (PID ~-1.65, LQR ~-1.85), nothing near the
  -3 m/s limit. Vertical control is solved; all failures are horizontal.

  ## Day 13 — animation
- animate.py writes figures/landing.gif from land_2d.run().
- Unequal axis scaling made a 19 deg tilt look like 60 deg. Fixed with equal aspect
  and a camera that follows the vehicle.
- The state is a point mass, so ground contact is on the centre of mass. The body is
  drawn standing on (x, z); its length is illustrative only.
- Subsampling dropped the final frame (label showed vz -1.98 at z 0.1 m). Fixed by
  always keeping the last index; the final frame is held for 1 s.
- Verified against land_2d output: theta -19.9 deg at t 4.6 s (pitch clamp),
  final frame t 29.4 s, vz -1.66, theta 0.6 deg. Nose-left tilt with exhaust down-right
  while killing the 200 m offset, which matches the sign convention.

## Day 14 — README and report
- README final: animation at top, Monte Carlo section with figure, offset-drift
  coupling paragraph, updated file list and limitations. Removed "from scratch".
- Two-page report drafted (report.docx, kept outside the repo). To be rewritten in my
  own words before it goes into the repo as report.pdf.