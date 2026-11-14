"""One-dimensional Kalman filter for the distance between the robot and the tag.

State      d_k   distance from the tag plane to the robot reference point (m)
Prediction d_k|k-1 = d_k-1 - ds_k                  (ds_k = forward wheel-odometry increment)
           P_k|k-1 = P_k-1 + Q_k,   Q_k = (alpha*|ds_k|)^2 + (sigma_v*dt)^2
Correction y_k = z_k - d_k|k-1                     (innovation, z_k = calibrated camera distance)
           S_k = P_k|k-1 + R_k
           K_k = P_k|k-1 / S_k
           d_k = d_k|k-1 + K_k*y_k,   P_k = (1 - K_k)*P_k|k-1
A measurement with |y_k| > g*sqrt(S_k) is rejected (innovation gate, g = 3).
"""
import math
from collections import deque


class DistanceKF:
    def __init__(self, d0, sigma0, odom_sigma_frac=0.03, process_sigma_mps=0.005, gate_sigma=3.0):
        self.d, self.P = float(d0), float(sigma0) ** 2
        self.alpha, self.sv, self.gate = odom_sigma_frac, process_sigma_mps, gate_sigma
        self.n_accept = self.n_reject = 0

    def predict(self, ds, dt):
        self.d -= ds
        self.P += (self.alpha * ds) ** 2 + (self.sv * dt) ** 2
        return self.d

    def correct(self, z, R):
        y = z - self.d
        S = self.P + R
        if abs(y) > self.gate * math.sqrt(S):
            self.n_reject += 1
            return {'innov': y, 'S': S, 'K': 0.0, 'accepted': False}
        K = self.P / S
        self.d += K * y
        self.P *= (1.0 - K)
        self.n_accept += 1
        return {'innov': y, 'S': S, 'K': K, 'accepted': True}

    @property
    def sigma(self):
        return math.sqrt(self.P)


class OdomHistory:
    """Time-stamped odometry so that a late camera frame can be moved to the present (latency compensation)."""

    def __init__(self, maxlen=500):
        self.buf = deque(maxlen=maxlen)

    def add(self, t, s):
        self.buf.append((t, s))

    def at(self, t):
        """Linear interpolation of the travelled distance s at time t (clamped to the stored span)."""
        if not self.buf:
            return 0.0
        if t <= self.buf[0][0]:
            return self.buf[0][1]
        prev = self.buf[0]
        for cur in self.buf:
            if cur[0] >= t:
                span = cur[0] - prev[0]
                w = 0.0 if span <= 0 else (t - prev[0]) / span
                return prev[1] + w * (cur[1] - prev[1])
            prev = cur
        return self.buf[-1][1]


def latency_compensate(z_capture, s_now, s_capture):
    """A frame shows the scene at capture time; the robot has since moved (s_now - s_capture) closer."""
    return z_capture - (s_now - s_capture)
