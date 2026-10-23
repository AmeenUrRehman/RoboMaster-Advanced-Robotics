import math

import numpy as np
import pytest

from rm_flatness_nav.flatness import (FlatTrajectory, eval_cubic, flat_inversion,
                                      solve_flat_polynomial, time_horizon)
from rm_flatness_nav.state_machine import (ALIGNING, ARRIVED, CHASING, SEARCHING,
                                           NavStateMachine, Observation)
from rm_flatness_nav.wheel_voltage import MotorVoltageModel, wheel_rates


def test_polynomial_meets_boundary_conditions():
    T = 7.0
    a = solve_flat_polynomial(0.0, 0.06, 1.45, 0.0, T)
    p0, v0, _ = eval_cubic(a, 0.0)
    pT, vT, _ = eval_cubic(a, T)
    assert (p0, v0) == pytest.approx((0.0, 0.06))
    assert (pT, vT) == pytest.approx((1.45, 0.0))


def test_time_horizon_formula_and_floor():
    assert time_horizon(1.65) == pytest.approx(1.6 * 1.45 / 0.16)
    assert time_horizon(0.5) == 4.0          # short approach: 4 s floor


def test_straight_trajectory_has_zero_turn_rate_and_limited_speed():
    tr = FlatTrajectory(1.65)
    ts = np.linspace(0, tr.T, 400)
    v = np.array([tr.sample(t)[0] for t in ts])
    w = np.array([tr.sample(t)[1] for t in ts])
    assert np.allclose(w, 0.0)
    assert v.max() <= 0.16
    assert tr.sample(tr.T)[2] == pytest.approx(1.65 - 0.20)
    assert tr.sample(tr.T)[0] == pytest.approx(0.0, abs=1e-6)


def test_flat_inversion_circle():
    # circle of radius R at angular speed om: v = R*om, w = om
    R, om, t = 2.0, 0.5, 1.3
    xd, yd = -R * om * math.sin(om * t), R * om * math.cos(om * t)
    xdd, ydd = -R * om ** 2 * math.cos(om * t), -R * om ** 2 * math.sin(om * t)
    v, w = flat_inversion(xd, yd, xdd, ydd)
    assert v == pytest.approx(R * om) and w == pytest.approx(om)


def test_flat_inversion_standstill_is_finite():
    assert flat_inversion(0.0, 0.0, 0.0, 0.0) == (0.0, 0.0)


def test_wheel_rates_eq4():
    w1, w2, w3, w4 = wheel_rates(0.1, 0.0)
    assert (w1, w2, w3, w4) == pytest.approx((2.0, 2.0, 2.0, 2.0))
    w1, w2, w3, w4 = wheel_rates(0.0, 0.5)           # (0 +/- 0.2*0.5)/0.05
    assert (w1, w2, w3, w4) == pytest.approx((2.0, -2.0, -2.0, 2.0))


def test_voltage_model_eq5():
    m = MotorVoltageModel()
    assert m.step((0, 0, 0, 0), 0.05) == (0, 0, 0, 0)
    v = m.step((2.0, 2.0, 2.0, 2.0), 0.05)           # 0.38*2 + 0.06*(2-0)/0.05
    assert v == pytest.approx((0.76 + 2.4,) * 4)
    v = m.step((2.0, 2.0, 2.0, 2.0), 0.05)           # steady: back-EMF only
    assert v == pytest.approx((0.76,) * 4)
    m.step((0, 0, 0, 0), 0.05)
    assert m.step((0, 0, 0, 0), 0.05) == (0, 0, 0, 0)  # halted -> exactly 0 V


def obs(now, visible=True, bearing=0.0, distance=1.6, tof=None):
    return Observation(now=now, visible=visible, bearing=bearing, distance=distance, tof=tof)


def test_fsm_search_turns_toward_last_seen_side():
    sm = NavStateMachine()
    assert sm.update(obs(0.0, visible=False)) == (0.0, 0.30)       # default CCW
    sm.update(obs(0.1, bearing=-0.5))                               # tag seen on the right
    v, w = sm.update(obs(0.2, visible=False))                       # still within 0.45 s
    assert (v, w) == (0.0, 0.0) and sm.state == ALIGNING
    v, w = sm.update(obs(0.7, visible=False))                       # lost > 0.45 s
    assert sm.state == SEARCHING and (v, w) == (0.0, -0.30)


def test_fsm_aligns_in_place_before_chasing():
    sm = NavStateMachine()
    v, w = sm.update(obs(0.0, bearing=0.5))
    assert sm.state == ALIGNING and v == 0.0 and w > 0                # tag left -> turn left
    v, w = sm.update(obs(0.1, bearing=-0.5))
    assert v == 0.0 and w < 0
    t = 0.2
    sm.update(obs(t, bearing=0.05))                                   # inside +/-0.18 rad
    assert sm.state == ALIGNING                                       # settling
    sm.update(obs(t + 0.35, bearing=0.05))
    assert sm.state == CHASING


def test_fsm_brake_on_camera_and_tof():
    for kw in ({'distance': 0.19}, {'tof': 0.18}):
        sm = NavStateMachine()
        sm.update(obs(0.0)); sm.update(obs(0.4))
        assert sm.state == CHASING
        assert sm.update(obs(0.5, **kw)) == (0.0, 0.0) and sm.state == ARRIVED
        assert sm.update(obs(0.6, bearing=0.5)) == (0.0, 0.0)         # latched


def test_fsm_chase_speed_within_limit():
    sm = NavStateMachine()
    sm.update(obs(0.0)); sm.update(obs(0.4))
    v, w = sm.update(obs(1.0))
    assert 0.0 < v <= 0.16


def test_fsm_loss_before_chase_returns_to_search():
    sm = NavStateMachine()
    sm.update(obs(0.0, bearing=0.5))                      # ALIGNING only
    sm.update(obs(0.1, visible=False))
    v, w = sm.update(obs(0.7, visible=False))
    assert sm.state == SEARCHING and v == 0.0


def test_fsm_tag_leaving_image_during_chase_keeps_following_trajectory():
    sm = NavStateMachine()
    sm.update(obs(0.0, distance=0.9)); sm.update(obs(0.4, distance=0.9))
    assert sm.state == CHASING
    v, w = sm.update(obs(2.0, visible=False))             # lost for > 0.45 s: close to the tag
    assert sm.state == CHASING and 0.0 < v <= 0.16
    t = 2.0
    while sm.state == CHASING and t < 40:
        t += 0.05
        sm.update(obs(t, visible=False))
    assert sm.state == ARRIVED and 'tag out of view' in sm.arrival_reason
    assert sm.update(obs(t + 0.1, visible=False)) == (0.0, 0.0)


def test_fsm_blind_chase_can_be_disabled():
    from rm_flatness_nav.state_machine import Params
    sm = NavStateMachine(Params(blind_chase=False))
    sm.update(obs(0.0)); sm.update(obs(0.4))
    sm.update(obs(1.7, visible=False))
    assert sm.state == SEARCHING
