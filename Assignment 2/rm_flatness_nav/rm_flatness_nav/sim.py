"""Offline simulation of the whole pipeline (state machine + flat trajectory + voltage model).

NOT hardware data.  It exists to check the logic before running the robot:
    python -m rm_flatness_nav.sim --distance 1.65 --lateral 0.30 --heading-deg 70 --out sim_out
Writes sim_voltage_log.csv and a v(t) / V1(t) plot into --out (clearly labelled SIMULATION).
"""
import argparse
import csv
import math
import os

import numpy as np

from .state_machine import ARRIVED, NavStateMachine, Observation
from .wheel_voltage import MotorVoltageModel, wheel_rates


def simulate(distance=1.65, lateral=0.30, heading_deg=70.0, detect_range=1.8, min_range=0.0, fov_deg=45.0,
             dt=0.05, t_end=90.0, seed=0, tau=0.15):
    """Unicycle robot with first-order velocity lag; tag at (distance, lateral) in the world."""
    rng = np.random.default_rng(seed)
    tag = np.array([distance, lateral])
    x = y = 0.0
    th = math.radians(heading_deg)       # initial heading: tag may start outside the camera cone
    v = w = 0.0
    sm, vm = NavStateMachine(), MotorVoltageModel()
    rows, hist, t, next_log, stop_ticks = [], [], 0.0, 0.0, 0
    while t < t_end:
        rel = tag - np.array([x, y])
        dist = float(np.hypot(*rel))
        bearing = math.atan2(rel[1], rel[0]) - th
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        visible = abs(bearing) < math.radians(fov_deg) and min_range <= dist <= detect_range
        obs = Observation(now=t, visible=visible,
                          bearing=bearing + rng.normal(0, 0.01) if visible else 0.0,
                          distance=dist + rng.normal(0, 0.01) if visible else float('inf'))
        v_cmd, w_cmd = sm.update(obs)
        # velocity lag, then integrate
        v += (v_cmd - v) * dt / tau
        w += (w_cmd - w) * dt / tau
        if sm.state == ARRIVED:
            v = w = 0.0
        x, y, th = x + v * math.cos(th) * dt, y + v * math.sin(th) * dt, th + w * dt
        volts = vm.step(wheel_rates(v_cmd, w_cmd), dt)
        hist.append((t, v, w, *volts, dist))
        if sm.state == ARRIVED:
            stop_ticks += 1
            if stop_ticks == 3:
                rows.append([f'{t:.1f}', *[f'{q:.3f}' for q in volts], f'ARRIVED ({sm.arrival_reason})'])
                break
        elif t >= next_log:
            rows.append([f'{int(round(t))}', *[f'{q:.3f}' for q in volts], sm.state])
            next_log += 1.0
        t += dt
    return rows, np.array(hist), sm, (x, y, th), tag


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--distance', type=float, default=1.65)
    ap.add_argument('--lateral', type=float, default=0.30)
    ap.add_argument('--heading-deg', type=float, default=70.0)
    ap.add_argument('--detect-range', type=float, default=1.8)
    ap.add_argument('--min-range', type=float, default=0.0, help='tag leaves the image closer than this (m)')
    ap.add_argument('--out', default='sim_out')
    a = ap.parse_args(argv)
    rows, hist, sm, pose, tag = simulate(a.distance, a.lateral, a.heading_deg, a.detect_range, a.min_range)
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, 'sim_voltage_log.csv'), 'w', newline='') as fh:
        wr = csv.writer(fh)
        wr.writerow(['t_s', 'V1_V', 'V2_V', 'V3_V', 'V4_V', 'state'])
        wr.writerows(rows)
    final = float(np.hypot(tag[0] - pose[0], tag[1] - pose[1]))
    print(f'SIMULATION: final state {sm.state} ({sm.arrival_reason}), distance to tag {final:.3f} m')
    for r in rows[:6] + [['...'] * 6] + rows[-2:]:
        print('  '.join(f'{c:>10}' for c in r))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(7, 5), sharex=True, constrained_layout=True)
    ax[0].plot(hist[:, 0], hist[:, 1], color='#0369A1'); ax[0].set_ylabel('v (m/s)')
    ax[1].plot(hist[:, 0], hist[:, 3], color='#C2410C'); ax[1].set_ylabel('V1 (V)'); ax[1].set_xlabel('t (s)')
    for q in ax:
        q.grid(alpha=0.3)
    ax[0].set_title('SIMULATION (not hardware data)')
    fig.savefig(os.path.join(a.out, 'sim_v_V1.png'), dpi=150)


if __name__ == '__main__':
    main()
