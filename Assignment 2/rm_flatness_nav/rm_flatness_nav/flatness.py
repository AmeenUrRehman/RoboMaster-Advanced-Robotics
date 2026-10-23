"""Differential-flatness trajectory generation for a unicycle-type robot.

The flat output is the planar position z(t) = [x(t), y(t)]^T.  Every smooth curve
for z(t) gives the linear speed v and turn rate w in closed form (Eq. 1), so no
numerical optimiser is needed.  Each component is a cubic over horizon T (Eq. 2).
"""
import math

import numpy as np

EPS = 1e-6
V_MAX = 0.16        # m/s cruise limit (Eq. 3)
STANDOFF = 0.20     # m, final distance from the tag
T_MIN = 4.0         # s
T_GAIN = 1.6


def flat_inversion(xd, yd, xdd, ydd):
    """Eq. (1): recover (v, w) from the first and second derivatives of z(t)."""
    speed_sq = xd * xd + yd * yd
    v = math.sqrt(speed_sq)
    if speed_sq < EPS:
        # heading is undefined at standstill (0/0): command no turning
        return v, 0.0
    w = (xd * ydd - yd * xdd) / speed_sq
    return v, w


def solve_flat_polynomial(z0, zdot0, zf, zdotf, T):
    """Eq. (2): cubic coefficients [a0, a1, a2, a3] of z(t) matching the four
    boundary values z(0), z'(0), z(T), z'(T).  Solved with numpy.linalg.solve."""
    A = np.array([[1.0, 0.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0, 0.0],
                  [1.0, T, T ** 2, T ** 3],
                  [0.0, 1.0, 2.0 * T, 3.0 * T ** 2]])
    b = np.array([z0, zdot0, zf, zdotf], dtype=float)
    return np.linalg.solve(A, b)


def eval_cubic(alpha, t):
    """Position, velocity and acceleration of the cubic at time t."""
    a0, a1, a2, a3 = alpha
    return (a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3,
            a1 + 2.0 * a2 * t + 3.0 * a3 * t ** 2,
            2.0 * a2 + 6.0 * a3 * t)


def time_horizon(d0, standoff=STANDOFF, v_max=V_MAX, t_min=T_MIN, gain=T_GAIN):
    """Eq. (3): T = max(4.0, 1.6 (d0 - 0.20) / v_max)."""
    return max(t_min, gain * (d0 - standoff) / v_max)


class FlatTrajectory:
    """Straight-ahead cubic pursuit in the robot frame at the moment of centering.

    Boundary conditions (assignment, Problem 2 ii):
        x(0)=0, x'(0)=0.06 m/s, x(T)=d0-0.20, x'(T)=0;  y(0)=0, y'(0)=0, y(T)=0, y'(T)=0.
    """

    def __init__(self, d0, v0=0.06, standoff=STANDOFF, v_max=V_MAX):
        self.d0 = d0
        self.T = time_horizon(d0, standoff, v_max)
        goal = max(d0 - standoff, 0.0)
        self.alpha_x = solve_flat_polynomial(0.0, v0, goal, 0.0, self.T)
        self.alpha_y = solve_flat_polynomial(0.0, 0.0, 0.0, 0.0, self.T)

    def sample(self, t):
        """(v, w, x, y) at time t since the trajectory started; clamps to [0, T]."""
        t = min(max(t, 0.0), self.T)
        x, xd, xdd = eval_cubic(self.alpha_x, t)
        y, yd, ydd = eval_cubic(self.alpha_y, t)
        v, w = flat_inversion(xd, yd, xdd, ydd)
        # the cubic may undershoot slightly below zero speed near T; never reverse
        if xd < 0.0:
            v = 0.0
        return v, w, x, y

    def finished(self, t):
        return t >= self.T
