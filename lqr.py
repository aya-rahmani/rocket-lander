import numpy as np
from scipy.linalg import solve_continuous_are
import config as cfg


def build_matrices(m):
    """Linearized dynamics about hover. u = [dT, delta]."""
    A = np.zeros((6, 6))
    A[0, 3] = 1.0
    A[1, 4] = 1.0
    A[2, 5] = 1.0
    A[3, 2] = cfg.g

    B = np.zeros((6, 2))
    B[3, 1] = cfg.g
    B[4, 0] = 1.0 / m
    B[5, 1] = m * cfg.g * cfg.L / cfg.I

    return A, B


def lqr_gain(A, B, Q, R):
    """Solve the algebraic Riccati equation and return K."""
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K


if __name__ == "__main__":
    A, B = build_matrices(cfg.m0)

    Q = np.diag([1.0, 1.0, 10.0, 1.0, 1.0, 10.0])
    R = np.diag([1e-9, 1e2])

    K = lqr_gain(A, B, Q, R)
    np.set_printoptions(precision=4, suppress=True)
    print("K =")
    print(K)