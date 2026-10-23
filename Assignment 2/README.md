# Assessment 2: differential-flatness AprilTag pursuit and motor voltage logging

Platform: DJI RoboMaster EP Core, ROS 2 Jazzy, `robomaster_ros` driver, OpenCV 4.6 (AprilTag dictionaries from `cv2.aruco`).

## What it does
1. **SEARCHING:** no tag for more than 0.45 s: rotate in place at 0.30 rad/s toward the side where the tag was last seen.
2. **ALIGNING:** tag visible: rotate in place until the bearing error is within 0.18 rad (10 degrees).
3. **CHASING:** cubic trajectory in the flat output `z = [x, y]`; `v` and `w` are recovered in closed form by flatness inversion. Horizon `T = max(4, 1.6 (d0 - 0.20) / 0.16)`. If the tag leaves the image (a nearby upright tag does), the trajectory continues from its time base.
4. **ARRIVED:** active brake when the camera distance or the front ToF reads 0.20 m or less, or when the trajectory ends.
5. Motor voltages from `V_i = K_v w_i + K_a (w_i(t) - w_i(t-dt))/dt` with `K_v = 0.38`, `K_a = 0.06`, logged once per second to `motor_voltage_log.csv` (and at 20 Hz to `motor_voltage_fast.csv`).

## Package `rm_flatness_nav`
| File | Purpose |
|---|---|
| `flatness.py` | `flat_inversion`, `solve_flat_polynomial` (uses `numpy.linalg.solve`), time horizon, `FlatTrajectory` |
| `wheel_voltage.py` | Wheel rates (Eq. 4) and armature voltage model (Eq. 5) |
| `state_machine.py` | The four-state supervisor (pure logic, unit tested) |
| `tag_detector.py` | AprilTag detection, pose with `solvePnP`, floor-plane range and bearing; retries at 2x image size for small tags |
| `nav_node.py` | ROS 2 node: camera, ToF, `cmd_vel`, 1 Hz voltage log and table |
| `camera_pose.py` | Predicts how large a floor tag appears in the camera |
| `sim.py` | Offline simulation of the whole pipeline (not hardware data) |

## Build and test
```
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
python -m pytest -q src/rm_flatness_nav/test        # 18 tests
```

## Run on the robot
```
ros2 launch robomaster_ros ep.launch conn_type:=rndis      # or conn_type:=ap
ros2 launch rm_flatness_nav flatness_nav.launch.py tag_id:=3 tag_size:=0.15 csv_path:=$PWD/motor_voltage_log.csv
```
`tag_size` is the measured side of the black square in metres. The run ends by itself after the robot halts.

## Measured result (Tag 3, `tag36h11`, 15 cm, upright, wired USB)
- Search 11.4 s, align 2.5 s, chase 10.9 s from d0 = 1.28 m, peak speed 0.137 m/s (limit 0.16 m/s).
- Final voltages V1 to V4 = 0.000 V at the halt; measured stop gap 20 cm.
- The stop came from the planned trajectory: the upright tag left the camera image below about 0.35 m, and the ToF returned no reading in range.

## Deviations from the assignment text
- The tag stood upright facing the robot instead of lying flat (a flat 15 cm tag is a thin sliver for this camera and cannot be read beyond about 0.7 m).
- The camera is fixed to the chassis (about 20 degrees downward); the arm pose (105 mm, 30 mm) does not change its tilt.

## Build the report
```
tectonic -X compile report/report.tex
```
