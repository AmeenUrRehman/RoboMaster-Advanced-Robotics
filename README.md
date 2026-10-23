# RoboMaster: Advanced Robotics (SCE 672)

Term project assessments on the DJI RoboMaster EP Core (Ubuntu 24.04, ROS 2 Jazzy, Python 3.12).
**Private repository** (course honor code).

Author: Ameen Ur Rehman (202527430), King Fahd University of Petroleum & Minerals.

| Folder | Content | Report |
|---|---|---|
| [`Assignment 1`](Assignment%201) | Environment provisioning, SDK keyboard teleoperation and telemetry (wired USB and Wi-Fi AP), ROS 2 integration, mecanum kinematics | `Assignment 1/report/report.pdf` |
| [`Assignment 2`](Assignment%202) | Differential-flatness AprilTag pursuit with a state machine and motor-voltage logging (ROS 2 package `rm_flatness_nav`) | `Assignment 2/report/report.pdf` |
| [`Assets`](Assets) | Printable AprilTags for Tags 1, 2, 3 (`tag36h11`, black square 150 mm, A4 PDFs), the mission figure, and the script that makes them | |
| [`Game`](Game) | **RoboMaster Quest**: a kid-friendly browser game that explains the project (open `Game/index.html`) | |

![Mission](Assets/fig_mission_chase.png)

## Highlights
- **Assignment 1:** wired RNDIS ping averages 0.45 ms (max 2.75 ms) against 28.3 ms (max 152 ms) over the robot access point; the telemetry stream is evenly spaced over USB (102 +/- 9 ms) and bursty over Wi-Fi (120 +/- 52 ms). The mecanum matrices were derived and verified numerically, and the NumPy 2.x failure was reproduced in a throw-away environment.
- **Assignment 2:** the robot searches, aligns, follows a cubic flat-output trajectory (peak 0.137 m/s) and stops 20 cm from the tag with all motor voltages at 0.000 V (two real runs, 18 unit tests, offline simulation, real logs included).

Each assignment folder has its own README with the file layout, commands, results and open items. The reports are LaTeX: `tectonic -X compile report/report.tex`.
