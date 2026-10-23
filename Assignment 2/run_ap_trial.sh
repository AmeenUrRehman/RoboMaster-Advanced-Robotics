#!/usr/bin/env bash
# One-shot AP-mode trial for Assessment 2 (Tag 3 pursuit).  Run in a NORMAL terminal AFTER the
# robot is on AP and the laptop is joined to its Wi-Fi:
#     ~/robomasters/assignment2/run_ap_trial.sh --tag-id 3 --tag-size 0.30 [--family tag36h11] [--tof]
# Optional --ssid NAME --pass PW joins the robot Wi-Fi for you.  Your Wi-Fi is restored at the end.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; cd "$HERE"; mkdir -p logs figures
TAG_ID=3; TAG_SIZE=""; FAMILY=tag36h11; TOF=false; SSID=""; PASS=""; RES=360
while [ $# -gt 0 ]; do case "$1" in
  --tag-id) TAG_ID="$2"; shift 2;; --tag-size) TAG_SIZE="$2"; shift 2;; --family) FAMILY="$2"; shift 2;;
  --tof) TOF=true; shift;; --ssid) SSID="$2"; shift 2;; --pass) PASS="$2"; shift 2;;
  --res) RES="$2"; shift 2;; *) echo "unknown arg $1"; exit 2;; esac; done
say(){ printf '\n\033[1;36m== %s\033[0m\n' "$*"; }; warn(){ printf '\033[1;33m%s\033[0m\n' "$*"; }
die(){ printf '\033[1;31mERROR: %s\033[0m\n' "$*"; exit 1; }
[ -n "$TAG_SIZE" ] || die "give the MEASURED tag side length in metres: --tag-size 0.30"
set -m
RESTORE_CON="${RESTORE_CON:-$(nmcli -t -f NAME,TYPE con show 2>/dev/null | awk -F: '$2=="802-11-wireless" && $1 !~ /^(ROBOMASTER|RMEP)/{print $1; exit}')}"
DRV=""; NAV=""
stop_group(){ [ -n "$1" ] && kill -INT -- "-$1" 2>/dev/null; }
cleanup(){ trap - EXIT INT TERM
  [ -n "$DRV" ] && timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
  stop_group "$NAV"; stop_group "$DRV"; sleep 3
  [ -n "$RESTORE_CON" ] && { say "Restoring Wi-Fi: $RESTORE_CON"; nmcli con up "$RESTORE_CON" >/dev/null 2>&1 && echo reconnected || warn "join '$RESTORE_CON' manually"; }; }
trap cleanup EXIT INT TERM
[ -n "$SSID" ] && { say "Joining $SSID"; nmcli dev wifi connect "$SSID" ${PASS:+password "$PASS"} || die "join failed"; sleep 5; }

say "0. Preflight"
source ~/rm_setup.sh; source "$HERE/ws/install/setup.bash" || die "build first: cd ws && colcon build --symlink-install"
ip -br addr | grep -q '192\.168\.2\.' || die "no 192.168.2.x address: join the robot Wi-Fi (switch on AP)."
ping -c 3 -W 2 192.168.2.1 >/dev/null 2>&1 || die "robot 192.168.2.1 not answering"
echo "robot reachable"

say "1. Driver (AP, ${RES}p video, ToF=$TOF)"
ros2 launch robomaster_ros ep.launch conn_type:=ap video_resolution:=$RES tof_0:=$TOF > logs/driver.log 2>&1 < /dev/null &
DRV=$!
for i in $(seq 1 45); do ros2 topic list 2>/dev/null | grep -q '^/camera/image_color$' && break; sleep 1; done
ros2 topic list 2>/dev/null | grep -q '^/camera/image_color$' || die "camera topic never appeared (logs/driver.log)"
{ echo '$ ros2 topic hz /camera/image_color (6 s)'; timeout 6 ros2 topic hz /camera/image_color 2>&1 | tail -3; } | tee logs/camera_rate.txt

say "2. Raise the arm to X=105 mm, Z=30 mm and report the camera pose / tag visibility"
ros2 topic pub --once /target_arm_position geometry_msgs/msg/Point "{x: 0.105, y: 0.0, z: 0.030}" >/dev/null 2>&1; sleep 6
ros2 run rm_flatness_nav camera_pose --ros-args -p tag_size:=$TAG_SIZE 2>&1 | tee logs/camera_pose.txt

say "3. Tag 3 pursuit: THE ROBOT WILL MOVE"
echo "Place the robot so Tag 3 is on the floor 1.5-1.8 m ahead (it may start facing away: it will search)."
echo "Keep a hand near the robot. Press Ctrl+C to abort at any time (zero velocity is sent)."
read -r -p "Enter to start... " _
rm -f logs/motor_voltage_log.csv logs/motor_voltage_fast.csv
ros2 run rm_flatness_nav flatness_nav --ros-args -p tag_id:=$TAG_ID -p tag_size:=$TAG_SIZE -p tag_family:=$FAMILY \
  -p use_tof:=$TOF -p set_arm_pose:=false -p csv_path:=$HERE/logs/motor_voltage_log.csv \
  -p fast_csv_path:=$HERE/logs/motor_voltage_fast.csv
NAV=""

say "4. Results"
if [ -s logs/motor_voltage_log.csv ]; then
  python scripts/make_table.py logs/motor_voltage_log.csv report/table1.tex
  python scripts/plot_run.py logs/motor_voltage_fast.csv figures/fig_v_V1.png
  cp logs/motor_voltage_log.csv motor_voltage_log.csv
  cat logs/motor_voltage_log.csv | cut -d, -f1-6
else warn "no log produced"; fi
say "DONE. Wi-Fi restores now. Then tell Claude: 'trial done'."
