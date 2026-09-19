# Vehicle parameters
m0 = 30000.0        # kg, initial mass
L = 15.0            # m, nozzle to centre of mass
I = 2.5e6           # kg·m², pitch moment of inertia
Isp = 300.0         # s, specific impulse

# Engine limits
T_max = 850000.0    # N
T_min = 0.4 * T_max
delta_max = 0.26    # rad, about 15 degrees

# Environment
g = 9.81            # m/s²
g0 = 9.80665        # m/s², standard gravity for Isp

# Simulation
dt = 0.01           # s

# Controller gains
throttle_kp = 120000.0
throttle_ki = 20000.0
throttle_kd = 8000.0

pos_kp = 0.006
pos_ki = 0.0
pos_kd = 0.05

att_kp = 1.5
att_ki = 0.0
att_kd = 1.5

theta_cmd_max = 0.35   # rad, about 20 degrees

# Guidance
descent_accel = 2.0    # m/s^2, sets the descent rate profile
descent_bias = 1.0     # m/s, minimum descent rate