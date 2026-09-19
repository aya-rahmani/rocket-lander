import numpy as np
import config as cfg


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


def clamp(value, low, high):
    return max(low, min(high, value))

def descent_target(z):
    """Desired descent rate at altitude z."""
    return -np.sqrt(2 * cfg.descent_accel * max(z, 0.0)) - cfg.descent_bias


class Lander:
    """Full cascade: position -> pitch -> gimbal, plus throttle."""

    def __init__(self):
        self.throttle = PID(cfg.throttle_kp, cfg.throttle_ki, cfg.throttle_kd)
        self.pos = PID(cfg.pos_kp, cfg.pos_ki, cfg.pos_kd)
        self.att = PID(cfg.att_kp, cfg.att_ki, cfg.att_kd)

    def step(self, state, m, dt):
        x, z, theta, vx, vz, omega = state

        theta_cmd = self.pos.step(0.0 - x, dt)
        theta_cmd = clamp(theta_cmd, -cfg.theta_cmd_max, cfg.theta_cmd_max)

        delta = self.att.step(theta_cmd - theta, dt)
        delta = clamp(delta, -cfg.delta_max, cfg.delta_max)

        T = m * cfg.g + self.throttle.step(descent_target(z) - vz, dt)
        T = clamp(T, 0.0, cfg.T_max)

        return T, delta