"""Wheel-rate kinematics (Eq. 4) and armature voltage model (Eq. 5) for the RoboMaster EP."""

WHEEL_RADIUS = 0.05   # r, m
CHASSIS_RADIUS = 0.20  # L_c, m
K_V = 0.38            # V.s/rad  (back-EMF)
K_A = 0.06            # V.s^2/rad (inertia)


def wheel_rates(vx, wz, r=WHEEL_RADIUS, lc=CHASSIS_RADIUS):
    """Eq. (4): returns (w1, w2, w3, w4) = (w_FR, w_FL, w_RL, w_RR) in rad/s."""
    w1 = (vx + lc * wz) / r   # front-right
    w2 = (vx - lc * wz) / r   # front-left
    return (w1, w2, w2, w1)   # w3 = w_RL = w2, w4 = w_RR = w1


class MotorVoltageModel:
    """Eq. (5): V_i = K_v w_i + K_a (w_i(t) - w_i(t-dt)) / dt, per motor."""

    def __init__(self, kv=K_V, ka=K_A):
        self.kv = kv
        self.ka = ka
        self._prev = None

    def reset(self):
        self._prev = None

    def step(self, omegas, dt):
        """Voltages for the current wheel rates; dt is the time since the last call."""
        if self._prev is None or dt <= 0.0:
            accel = (0.0,) * len(omegas)
        else:
            accel = tuple((w - p) / dt for w, p in zip(omegas, self._prev))
        self._prev = tuple(omegas)
        return tuple(self.kv * w + self.ka * a for w, a in zip(omegas, accel))
