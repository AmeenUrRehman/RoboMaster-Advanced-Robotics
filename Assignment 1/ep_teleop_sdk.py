#!/usr/bin/env python3
"""Standalone RoboMaster EP keyboard teleoperation + 10 Hz telemetry logger.

Uses only the native `robomaster` Python SDK (no ROS 2 imports).

Keys (curses, non-blocking):
    W / S   forward / backward          (vx, m/s)
    A / D   strafe left / right         (vy, m/s)
    Q / E   yaw left (CCW) / right (CW) (wz, deg/s)
    SPACE   immediate stop (zero all velocities)
    X / ESC quit

Terminals only deliver key *presses* (auto-repeat), never releases, so a key
counts as "held" while repeats keep arriving.  When they stop for
HOLD_TIMEOUT seconds the key is treated as released and the chassis stops.

SDK sign convention: x forward, y RIGHT, z CLOCKWISE (deg/s).

Usage:
    python ep_teleop_sdk.py --conn rndis --duration 30 --out logs/rndis_run.csv
    python ep_teleop_sdk.py --conn ap    --duration 30 --out logs/ap_run.csv
"""
import argparse
import csv
import curses
import os
import threading
import time

from robomaster import robot

LOG_HZ = 10.0           # telemetry log rate
CMD_HZ = 20.0           # command dispatch rate
FIRST_PRESS_HOLD = 0.55  # covers the OS typematic delay before auto-repeat
REPEAT_HOLD = 0.15       # hold window once auto-repeat is running

# key -> (vx, vy, wz) direction
KEYMAP = {
    ord('w'): (1, 0, 0), ord('s'): (-1, 0, 0),
    ord('a'): (0, -1, 0), ord('d'): (0, 1, 0),
    ord('q'): (0, 0, -1), ord('e'): (0, 0, 1),
}

CSV_HEADER = [
    't_rel_s', 't_unix', 'batt_pct', 'yaw_deg', 'pitch_deg', 'roll_deg',
    # SDK ESC order: wheel1=front-right, wheel2=front-left, wheel3=rear-left, wheel4=rear-right
    'w_fr_rpm', 'w_fl_rpm', 'w_rl_rpm', 'w_rr_rpm',
    't_esc_rx_unix',  # arrival time of the newest ESC packet (network jitter)
    'cmd_vx', 'cmd_vy', 'cmd_wz',
]


class Telemetry:
    """Thread-safe latest-value store filled by the SDK subscription callbacks."""

    def __init__(self):
        self.lock = threading.Lock()
        self.batt = float('nan')
        self.att = (float('nan'),) * 3
        self.esc = (float('nan'),) * 4
        self.esc_rx = float('nan')

    def on_batt(self, percent):
        with self.lock:
            self.batt = percent

    def on_att(self, info):
        with self.lock:
            self.att = tuple(info[:3])

    def on_esc(self, info):
        with self.lock:
            self.esc = tuple(info[0])
            self.esc_rx = time.time()

    def snapshot(self):
        with self.lock:
            return self.batt, self.att, self.esc, self.esc_rx


def active_command(deadlines, vx_max, vy_max, wz_max):
    """Sum the direction of every key whose hold window has not expired."""
    now = time.monotonic()
    vx = vy = wz = 0.0
    for key, deadline in list(deadlines.items()):
        if deadline < now:
            del deadlines[key]
            continue
        dx, dy, dz = KEYMAP[key]
        vx, vy, wz = vx + dx * vx_max, vy + dy * vy_max, wz + dz * wz_max
    return vx, vy, wz


def run(stdscr, args):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    ep = robot.Robot()
    ep.initialize(conn_type=args.conn)
    chassis, battery = ep.chassis, ep.battery
    tel = Telemetry()
    battery.sub_battery_info(freq=10, callback=tel.on_batt)
    chassis.sub_attitude(freq=10, callback=tel.on_att)
    chassis.sub_esc(freq=10, callback=tel.on_esc)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    deadlines = {}
    t0 = time.monotonic()
    next_log = t0 + 1.0 / LOG_HZ
    next_cmd = t0
    cmd = (0.0, 0.0, 0.0)
    rows = 0
    status = 'running'

    with open(args.out, 'w', newline='') as fh:
        writer = csv.writer(fh)
        writer.writerow(CSV_HEADER)
        try:
            while True:
                now = time.monotonic()
                if args.duration and now - t0 >= args.duration:
                    status = 'duration reached'
                    break

                # ---- keyboard ----
                key = stdscr.getch()
                while key != -1:  # drain everything queued since last loop
                    k = key if key > 255 else (ord(chr(key).lower()) if 0 <= key < 256 else key)
                    if k == ord(' '):
                        deadlines.clear()
                        cmd = (0.0, 0.0, 0.0)
                        chassis.drive_speed(x=0, y=0, z=0)  # failsafe, immediate
                    elif k in (ord('x'), 27):
                        status = 'quit by user'
                        raise StopIteration
                    elif k in KEYMAP:
                        hold = REPEAT_HOLD if k in deadlines else FIRST_PRESS_HOLD
                        deadlines[k] = now + hold
                    key = stdscr.getch()

                # ---- command dispatch ----
                if now >= next_cmd:
                    cmd = active_command(deadlines, args.vx, args.vy, args.wz)
                    chassis.drive_speed(x=cmd[0], y=cmd[1], z=cmd[2])
                    next_cmd += 1.0 / CMD_HZ

                # ---- 10 Hz telemetry log ----
                if now >= next_log:
                    batt, att, esc, esc_rx = tel.snapshot()
                    writer.writerow([f'{now - t0:.4f}', f'{time.time():.4f}', batt,
                                     *att, *esc, f'{esc_rx:.4f}', *cmd])
                    rows += 1
                    next_log += 1.0 / LOG_HZ
                    stdscr.erase()
                    stdscr.addstr(0, 0, f'RoboMaster EP teleop [{args.conn}]  t={now - t0:6.1f}s  rows={rows}')
                    stdscr.addstr(1, 0, 'W/S fwd/back  A/D strafe  Q/E yaw  SPACE stop  X quit')
                    stdscr.addstr(3, 0, f'cmd  vx={cmd[0]:+.2f} m/s  vy={cmd[1]:+.2f} m/s  wz={cmd[2]:+.1f} deg/s')
                    stdscr.addstr(4, 0, f'batt {batt:5.1f} %')
                    stdscr.addstr(5, 0, 'yaw/pitch/roll  %7.2f %7.2f %7.2f deg' % att)
                    stdscr.addstr(6, 0, 'wheels FR/FL/RL/RR  %6.0f %6.0f %6.0f %6.0f rpm' % esc)
                    stdscr.refresh()

                time.sleep(0.005)
        except (StopIteration, KeyboardInterrupt):
            pass
        finally:
            # failsafe: always leave the chassis stopped and subscriptions closed
            for _ in range(3):
                chassis.drive_speed(x=0, y=0, z=0)
                time.sleep(0.02)
            chassis.unsub_esc()
            chassis.unsub_attitude()
            battery.unsub_battery_info()
            ep.close()
    return status, rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--conn', choices=['rndis', 'ap'], required=True,
                    help='rndis = wired USB (robot 192.168.42.2); ap = direct Wi-Fi hotspot')
    ap.add_argument('--out', default='logs/telemetry.csv', help='CSV output path')
    ap.add_argument('--duration', type=float, default=0.0, help='auto-stop after N seconds (0 = run until X)')
    ap.add_argument('--vx', type=float, default=0.3, help='forward speed magnitude, m/s')
    ap.add_argument('--vy', type=float, default=0.3, help='strafe speed magnitude, m/s')
    ap.add_argument('--wz', type=float, default=45.0, help='yaw rate magnitude, deg/s')
    args = ap.parse_args()
    status, rows = curses.wrapper(run, args)
    print(f'{status}: wrote {rows} telemetry rows to {args.out}')


if __name__ == '__main__':
    main()
