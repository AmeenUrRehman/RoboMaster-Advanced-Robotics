# Assessment 1: System provisioning, keyboard teleoperation, telemetry and ROS 2 integration

Platform: DJI RoboMaster EP Core, Ubuntu 24.04, ROS 2 Jazzy, Python 3.12, NumPy 1.26.

## Contents
| Path | Purpose |
|---|---|
| `ep_teleop_sdk.py` | Standalone keyboard teleoperation and 10 Hz telemetry logger using only the native `robomaster` SDK (no ROS 2 imports). |
| `rm_setup.sh` | Environment loader: sources ROS 2 and the workspace, activates the venv, sets `PYTHONPATH` and `PYTHONNOUSERSITE` so a NumPy 2.x install cannot shadow the NumPy 1.26 that the ROS 2 binaries were built against. |
| `run_ap_all.sh` | Scripted Wi-Fi (AP) session: ping benchmark, SDK teleop, ROS 2 driver, TF tree, bag recording, odometry plot. |
| `scripts/` | `ping_bench.sh`, `analyze_telemetry.py` (interval and RTT statistics and plots), `plot_odom.py` (trajectory from the bag, in the start-pose frame). |
| `logs/` | Raw data: `rndis_run.csv`, `ap_run.csv`, `ping_*.txt`, driver and verification logs, `bag_ap/` (rosbag2, MCAP). |
| `figures/` | Plots and the TF tree (`frames.pdf`). |
| `report/` | `report.tex` and the compiled `report.pdf`. |

## Teleoperation script
```
source ~/rm_setup.sh
python ep_teleop_sdk.py --conn rndis --duration 30 --out logs/rndis_run.csv   # robot on USB
python ep_teleop_sdk.py --conn ap    --duration 30 --out logs/ap_run.csv      # robot on its own Wi-Fi
```
Keys: `W/S` forward/back, `A/D` strafe left/right, `Q/E` yaw, `Space` stop, `X` quit. Releasing the keys stops the chassis.
SDK sign convention: x forward, y to the right, yaw clockwise.

CSV columns: `t_rel_s, t_unix, batt_pct, yaw_deg, pitch_deg, roll_deg, w_fr_rpm, w_fl_rpm, w_rl_rpm, w_rr_rpm, t_esc_rx_unix, cmd_vx, cmd_vy, cmd_wz`
(`t_esc_rx_unix` is the arrival time of the newest wheel-speed packet, used for the telemetry-interval analysis).

## ROS 2 part (robot in AP mode, laptop on the robot Wi-Fi)
```
source ~/rm_setup.sh
ros2 launch robomaster_ros ep.launch conn_type:=ap
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 bag record -o logs/bag_ap /odom /joint_states /cmd_vel
ros2 run tf2_tools view_frames
python scripts/plot_odom.py logs/bag_ap --true-end <x> <y> --out figures   # x forward, y left, metres, from the start pose
```

## Analysis
```
python scripts/analyze_telemetry.py --rndis-csv logs/rndis_run.csv --ap-csv logs/ap_run.csv \
    --rndis-ping logs/ping_rndis.txt --ap-ping logs/ping_ap.txt --out figures
```

## Main measured results
| | Wired (RNDIS) | Wireless (AP) |
|---|---|---|
| Ping RTT, N=100 (mean / 95th percentile / max) | 0.45 / 0.56 / 2.75 ms | 28.3 / 127 / 152 ms |
| Telemetry interval (mean, std) | 102.3 ms, 8.8 ms | 120.2 ms, 52.3 ms |
| Packet loss (ping) | 0 % | 0 % |

## Status
Done: environment, SDK script, wired and AP runs, latency analysis, ROS 2 driver over AP, TF tree, bag, kinematic derivation.
ROS 2 odometry error against tape measurements: 0.351 m (2.4 % of 14.86 m) for the Wi-Fi run and 1.146 m (29.9 % of 3.83 m) for the USB run with strafing. Demonstration video links are in the report.

## Build the report
```
tectonic -X compile report/report.tex     # or: xelatex report/report.tex
```
