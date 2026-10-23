#!/usr/bin/env python3
"""Extract /odom (and /cmd_vel stats) from a rosbag2, save odom.csv, and plot X vs Y.

All poses are expressed in the START pose frame of the recording (x forward along the
robot's initial heading, y to its left, origin at its initial position), because that is
how the true end position is measured with a tape.  /odom itself is in the frame the
robot had when it booted, so the displacement is rotated by -yaw0.

Needs the ROS environment:  source ~/rm_setup.sh
Usage:
    python scripts/plot_odom.py logs/bag_ap [--true-end X Y] --out figures
Last stdout line is "STATS {json}" for scripts that call this.
"""
import argparse
import csv
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family': 'serif', 'font.serif': ['cmr10', 'DejaVu Serif'], 'mathtext.fontset': 'cm',
                     'axes.formatter.use_mathtext': True, 'axes.unicode_minus': False, 'font.size': 10})
import numpy as np
import rosbag2_py
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message


def read_bag(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir), rosbag2_py.ConverterOptions('', ''))
    odom, cmd = [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic.endswith('/odom'):
            m = deserialize_message(data, Odometry)
            q = m.pose.pose.orientation
            yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            odom.append((t_ns * 1e-9, m.pose.pose.position.x, m.pose.pose.position.y, yaw))
        elif topic.endswith('/cmd_vel'):
            m = deserialize_message(data, Twist)
            cmd.append((t_ns * 1e-9, m.linear.x, m.linear.y, m.angular.z))
    return np.array(odom), np.array(cmd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--true-end', type=float, nargs=2, metavar=('X', 'Y'),
                    help='tape-measured end position in the START frame (x forward, y left), metres')
    ap.add_argument('--out', default='figures')
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    d, c = read_bag(a.bag)
    t = d[:, 0] - d[0, 0]
    x0, y0, yaw0 = d[0, 1], d[0, 2], d[0, 3]
    dx, dy = d[:, 1] - x0, d[:, 2] - y0
    cs, sn = np.cos(-yaw0), np.sin(-yaw0)
    x, y = cs * dx - sn * dy, sn * dx + cs * dy          # start-frame coordinates
    yaw = np.unwrap(d[:, 3]) - np.unwrap(d[:, 3])[0]
    with open(os.path.join(a.out, 'odom.csv'), 'w', newline='') as fh:
        w = csv.writer(fh); w.writerow(['t_s', 'x_m', 'y_m', 'yaw_rad'])
        w.writerows(np.c_[t, x, y, yaw].tolist())

    path_len = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
    est_end = (float(x[-1]), float(y[-1]))
    stats = {'samples': int(len(t)), 'duration_s': float(t[-1]), 'path_m': path_len,
             'end_x': est_end[0], 'end_y': est_end[1], 'turn_deg': float(np.degrees(yaw[-1])),
             'max_vx': float(np.abs(c[:, 1]).max()) if len(c) else 0.0,
             'max_vy': float(np.abs(c[:, 2]).max()) if len(c) else 0.0,
             'max_wz': float(np.abs(c[:, 3]).max()) if len(c) else 0.0,
             'cmd_msgs': int(len(c))}
    print(f'samples={len(t)} duration={t[-1]:.1f}s rate={len(t) / t[-1]:.1f}Hz path={path_len:.3f}m')
    print(f'start-frame estimated end = ({est_end[0]:.3f}, {est_end[1]:.3f}) m (x forward, y left)')

    fig, ax = plt.subplots(figsize=(6, 5.5), constrained_layout=True)
    ax.plot(x, y, '-', color='#1f6feb', lw=1.5, label='/odom estimate')
    ax.plot(0, 0, 'go', label='start')
    ax.plot(*est_end, 'bs', label='estimated end')
    if a.true_end:
        ax.plot(*a.true_end, 'r*', ms=14, label='measured true end')
        err = float(np.hypot(est_end[0] - a.true_end[0], est_end[1] - a.true_end[1]))
        stats.update(true_x=a.true_end[0], true_y=a.true_end[1], error_m=err, error_pct=100 * err / path_len)
        print(f'endpoint error = {err:.3f} m  ({100 * err / path_len:.1f}% of path length)')
    ax.set_xlabel('X forward (m)'); ax.set_ylabel('Y left (m)'); ax.set_aspect('equal'); ax.grid(alpha=0.3)
    ax.set_title('Dead-reckoned trajectory from /odom (start frame)'); ax.legend(fontsize=8)
    fig.savefig(os.path.join(a.out, 'fig_odom.png'), dpi=200)
    print('STATS ' + json.dumps(stats))


if __name__ == '__main__':
    main()
