"""Straight-line approach law and stop rule (pure functions, unit tested).

Forward speed     v_ref = clip(k_v (d̂ - d_stop), v_min, v_max), slew-limited by a_max
Heading           ω = sat(k_b β, ω_max) in the SDK sign (z > 0 = clockwise); β > 0 = tag to the right,
                  so the robot turns toward the tag and keeps its path on the line to the tag centre.
Stop              when d̂ - d_stop <= v t_lead (t_lead covers the command delay and the braking distance).
"""


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def speed_command(d_hat, d_stop, v_prev, dt, cfg):
    v_ref = clip(cfg['k_speed_per_s'] * (d_hat - d_stop), cfg['v_min_mps'], cfg['v_max_mps'])
    dv = cfg['accel_mps2'] * dt
    return clip(v_ref, v_prev - dv, v_prev + dv)


def yaw_rate_command(bearing_deg, cfg):
    """SDK chassis z (deg/s, clockwise positive): turn right when the tag is to the right."""
    if bearing_deg is None:
        return 0.0
    w = cfg['k_bearing_dps_per_deg'] * bearing_deg
    return clip(w, -cfg['wz_max_dps'], cfg['wz_max_dps'])


def should_stop(d_hat, d_stop, v, cfg):
    return d_hat - d_stop <= v * cfg['stop_lead_s']
