# SCE 672 Assessment 2: flatness pursuit

Package: `ws/src/rm_flatness_nav` (build: `cd ws && colcon build --symlink-install`).
Report: `report/report.tex` (`~/tools/tectonic/tectonic -X compile report/report.tex`; search for TODO).

## Offline checks (no robot)
    source ~/rm_setup.sh && source ws/install/setup.bash
    PYTHONPATH=ws/src/rm_flatness_nav python -m pytest -q ws/src/rm_flatness_nav/test
    python -m rm_flatness_nav.sim --out /tmp/sim   # simulation only, NOT hardware data

## Before the real run (all need the robot)
1. Measure the tag: printed black-square side (m), family (tag36h11?), and the ID of Tag 3.
2. Driver (USB `conn_type:=rndis` or AP `conn_type:=ap`), with ToF only if fitted:
       ros2 launch robomaster_ros ep.launch conn_type:=rndis tof_0:=true video_resolution:=720
3. Put the arm at X=105 mm, Z=30 mm (the node does it at start), then check what the camera sees:
       ros2 run rm_flatness_nav camera_pose --ros-args -p tag_size:=0.30
   Rows visible below ~15 px at the tag's real distance = the tag is too small/too far for the stream.
4. Run (the robot moves toward the tag; clear the floor, keep a hand near it):
       ros2 launch rm_flatness_nav flatness_nav.launch.py tag_id:=3 tag_size:=0.30 csv_path:=$PWD/motor_voltage_log.csv
5. Ctrl+C stops the node and publishes zero velocity. Output: `motor_voltage_log.csv` (1 Hz rows + final stop row).
6. Plot v(t) and V1(t) from a `ros2 bag record /cmd_vel` (or the CSV columns v_mps, V1_V) into `figures/fig_v_V1.png`.
