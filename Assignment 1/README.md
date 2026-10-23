# SCE 672 Assessment 1: RoboMaster EP

Layout
- `ep_teleop_sdk.py`  Problem 2 script (native SDK, curses keyboard, 10 Hz CSV telemetry)
- `rm_setup.sh`       environment loader (also copied to `~/rm_setup.sh`)
- `scripts/`          `ping_bench.sh`, `analyze_telemetry.py`, `plot_odom.py`
- `logs/`             raw CSVs, ping output, rosbag, `system_diagnostics.txt`
- `figures/`          generated plots, `frames.pdf`
- `report/report.tex` the PDF report (search for `TODO` to find what still needs hardware data)

Compile the report: `~/tools/tectonic/tectonic -X compile report/report.tex`

## Hardware runbook (run from this folder)

Always start a terminal with `source ~/rm_setup.sh`.

### Problem 2(ii): wired RNDIS
1. Controller switch to **Direct/USB**, Micro-USB data cable to the laptop, robot powered on.
2. `ip -br addr` and note the new interface name and IP (Table 1). `ping -c 3 192.168.42.2` must answer.
3. `python ep_teleop_sdk.py --conn rndis --duration 30 --out logs/rndis_run.csv`
   (W/S, A/D, Q/E, Space = stop, X = quit). Screenshot the terminal.
4. `scripts/ping_bench.sh rndis`

### Problem 2(iii): wireless AP
1. Switch to **AP**. Join the `ROBOMASTER-XXXXXX` Wi-Fi network. `ip -br addr` and note interface and IP.
2. `python ep_teleop_sdk.py --conn ap --duration 30 --out logs/ap_run.csv`
3. `scripts/ping_bench.sh ap`
4. `python scripts/analyze_telemetry.py --rndis-csv logs/rndis_run.csv --ap-csv logs/ap_run.csv --rndis-ping logs/ping_rndis.txt --ap-ping logs/ping_ap.txt --out figures`
   Numbers for Table 1 and the discussion are in `figures/stats.json`.

### Problem 3 (stay in AP mode)
Terminal A: `ros2 launch robomaster_ros ep.launch conn_type:=ap`
Terminal B: `ros2 topic list`, `ros2 topic hz /odom`, `ros2 topic hz /joint_states` (screenshot all)
Terminal C: `ros2 bag record -o logs/bag_ap /odom /joint_states /cmd_vel`
Terminal D: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`  (drive a path with forward AND strafing legs, e.g. an L shape)
Stop the bag with Ctrl+C. Measure the robot's true end position with a tape from the start pose.
`ros2 bag info logs/bag_ap` (screenshot)
`python scripts/plot_odom.py logs/bag_ap --true-end X Y --out figures`

### Problem 4(iii)
With the driver running: `ros2 run tf2_tools view_frames` then move `frames.pdf` into `figures/`.

### Video (D3, 45 s max)
(a) SDK teleop with the live telemetry display, (b) ROS 2 teleop over AP with `ros2 topic echo /odom` in a second terminal.
