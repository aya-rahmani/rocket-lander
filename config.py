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