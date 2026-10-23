# RoboMaster: Advanced Robotics (SCE 672)

Term project assessments on the DJI RoboMaster EP Core: Ubuntu 24.04, ROS 2 Jazzy, Python 3.12.
**Private repository** (course honor code).

Author: Ameen Ur Rehman (202527430), King Fahd University of Petroleum & Minerals.

| Folder | Topic | Report |
|---|---|---|
| [`Assignment 1`](Assignment%201) | System provisioning, SDK keyboard teleoperation and telemetry (wired USB and Wi-Fi AP), ROS 2 integration, mecanum kinematics | `Assignment 1/report/report.pdf` |
| [`Assignment 2`](Assignment%202) | Differential-flatness AprilTag pursuit with state machine and motor-voltage logging (ROS 2 package `rm_flatness_nav`) | `Assignment 2/report/report.pdf` |

## Highlights
- **Assignment 1:** wired RNDIS ping averages 0.45 ms (max 2.75 ms) against 28.3 ms (max 152 ms) over the robot access point; the telemetry stream is evenly spaced over USB (102 +/- 9 ms) and bursty over Wi-Fi (120 +/- 52 ms).
- **Assignment 2:** the robot searches, aligns, follows a cubic flat-output trajectory (peak 0.137 m/s) and stops 20 cm from the tag with all motor voltages at 0.000 V; 18 unit tests, offline simulation, and real logs included.

Each assignment folder has its own README with the file layout, commands, results and open items.
Reports are LaTeX (`tectonic -X compile report/report.tex`).
