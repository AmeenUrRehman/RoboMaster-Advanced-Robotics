# Assignment 3 (Project Phase 3): RoboMaster odometry with prediction and correction

Platform: DJI RoboMaster EP Core, Ubuntu 24.04, DJI `robomaster` Python SDK over USB (RNDIS), OpenCV 4.6 (AprilTag `tag36h11` from `cv2.aruco`).

## What it does
The robot is placed facing an upright AprilTag (Tag 3, 15 cm) at a fixed **start distance of 1.50 m**. It detects the tag, drives straight towards it and stops when its estimated distance reaches the **stopping distance of 0.50 m**. Distances are measured from the tag plane to the **front face of the chassis front armour** (centre line). The same point is used for the robot's estimate and for the tape measurement.

The distance `d` is estimated with a 1-D Kalman filter:

| Step | Equation | Sensor |
|---|---|---|
| Prediction | `d = d - Δs`,  `P = P + (α Δs)² + (σv Δt)²` | wheel odometry (chassis position, 50 Hz) |
| Measurement | `z = a·|nᵀt| + b - [s(t) - s(t-L)]` | camera, AprilTag pose by `solvePnP` (720p, about 30 Hz) |
| Correction | `K = P/(P+R)`,  `d = d + K(z - d)`,  `P = (1-K)P`,  gate `|z-d| > 3√(P+R)` | |
| Steering | `ω = sat(kβ·β)`, `v = sat(kv(d - d_stop))` | tag bearing |

* `|nᵀt|` is the camera-to-tag-plane distance. It does not depend on the camera's 20° downward pitch.
* `a, b` come from a calibration at rest at five tape-measured distances (`config/calib.json`).
* `L` is the camera latency, estimated from two warm-up runs (`config/latency.json`).
* `R = σz(z)² + (σL·v)²` with `σz = c0 + c1 z²`.

Each trial stores three final estimates: **fused KF**, **odometry only** (`d0 - s`) and **camera only**. The error is `e = estimate - tape`. The analysis reports the bias, SD, **MAE**, RMSE, **range**, the 95 % CI and t-test of the bias, Shapiro-Wilk, and Wilcoxon tests between the estimators.

## Layout
| Path | Purpose |
|---|---|
| `rm_odom_tag/kf.py` | Kalman filter, odometry history, latency compensation |
| `rm_odom_tag/tag.py` | AprilTag detection, PnP, plane distance, calibrated camera model |
| `rm_odom_tag/control.py` | speed law, steering law, stop rule |
| `rm_odom_tag/trial.py` | one trial: DETECT → APPROACH → SETTLE, CSV log + annotated on-board video |
| `rm_odom_tag/robot_io.py` | `SdkIO` (robot) and `SimIO` (test simulator that renders the tag) |
| `run_trials.py` | session driver: `check`, `calib`, `warmup`, `trials`, `tape`, `report`, `session`, `sim` |
| `scripts/` | `fit_calib.py`, `estimate_latency.py`, `analyze.py`, `make_term_figs.py` |
| `config/setup.json` | experiment constants (start/stop distance, tag, gains, noise parameters) |
| `logs/` | **real robot data only**: `calib_raw.csv`, `warmup/`, `trials/trial_XX.csv/.mp4`, `trials.csv` |
| `figures/`, `report/` | plots and the LaTeX report (`report/report.pdf`) |
| `tests/` | 16 tests (filter, control, tag measurement, calibration, SDK interface, full simulated pipeline) |

## Run
```
source ~/rm_setup.sh && cd assignment3
python -m pytest -q tests            # 16 tests, no robot
python run_trials.py session         # check -> calib -> warmup -> 10 trials -> analysis -> report PDF
```
Single steps: `python run_trials.py check | calib | warmup | trials --n 10 | report`. Use `python run_trials.py tape 4 51.2` to enter or correct a tape value (cm or m). Use `--conn ap` for the robot's Wi-Fi.

After every trial the program asks for the tape-measured final distance. Leave the robot where it stopped and measure from the tag plane to the front face of the chassis armour.

## Results
Filled in from `logs/trials.csv` by `scripts/analyze.py` (see `report/report.pdf`).

## Video
Demonstration: (link added after recording)
