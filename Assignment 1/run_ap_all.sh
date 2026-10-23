#!/usr/bin/env bash
# One-shot AP-mode session for SCE 672 Assessment 1.
#
# Run this in a NORMAL terminal (not via Claude) AFTER the robot is switched to
# AP mode and the laptop is joined to ROBOMASTER-XXXXXX Wi-Fi:
#
#     ~/robomasters/assignment1/run_ap_all.sh
#
# Optional: let the script join the robot Wi-Fi itself (drops your internet):
#     ./run_ap_all.sh --ssid ROBOMASTER-XXXXXX --pass 12341234
#
# What it does, in order (everything is saved under logs/ and figures/):
#   0. preflight       host interface/IP, route, ping 192.168.2.1
#   1. ping N=100      logs/ping_ap.txt                      (Problem 2(iv))
#   2. SDK teleop 30 s logs/ap_run.csv                       (Problem 2(iii))   YOU DRIVE
#   3. ROS 2 driver    logs/ap_driver.log, topic checks      (Problem 3(i))
#   4. TF tree         figures/frames.pdf, tf publishers     (Problem 4(iii))
#   5. bag + teleop    logs/bag_ap/                          (Problem 3(ii,iii)) YOU DRIVE
#   6. odom plot       figures/fig_odom.png, odom.csv        (Problem 3(iv))
#   7. restore Wi-Fi to the profile you were on before, then print what to send back.
# (no set -u: ROS setup scripts read unset variables)

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
mkdir -p logs figures
SSID=""; PASS=""
while [ $# -gt 0 ]; do
  case "$1" in
    --ssid) SSID="$2"; shift 2 ;;
    --pass) PASS="$2"; shift 2 ;;
    *) echo "unknown arg $1"; exit 2 ;;
  esac
done

say()  { printf '\n\033[1;36m== %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m%s\033[0m\n' "$*"; }
die()  { printf '\033[1;31mERROR: %s\033[0m\n' "$*"; exit 1; }

# Job control: every background/foreground step gets its own process group, so Ctrl+C
# in the teleop window only stops teleop (not this script) and kill -INT -PGID works.
set -m

# ---- the Wi-Fi profile to restore at the end (first saved non-robot Wi-Fi) ----
RESTORE_CON="${RESTORE_CON:-$(nmcli -t -f NAME,TYPE con show 2>/dev/null | awk -F: '$2=="802-11-wireless" && $1 !~ /^ROBOMASTER/{print $1; exit}')}"
DRIVER_PID=""; BAG_PID=""
stop_group() { [ -n "$1" ] && kill -INT -- "-$1" 2>/dev/null; }
cleanup() {
  trap - EXIT INT TERM
  stop_group "$BAG_PID"; stop_group "$DRIVER_PID"; sleep 3
  if [ -n "$RESTORE_CON" ]; then
    say "Restoring Wi-Fi: $RESTORE_CON"
    nmcli con up "$RESTORE_CON" >/dev/null 2>&1 && echo "reconnected." || warn "Could not auto-reconnect; join '$RESTORE_CON' manually."
  fi
}
trap cleanup EXIT INT TERM

# ---- optional: join the robot AP ourselves ----
if [ -n "$SSID" ]; then
  say "Joining $SSID (internet will drop until the script ends)"
  nmcli dev wifi connect "$SSID" ${PASS:+password "$PASS"} || die "could not join $SSID"
  sleep 5
fi

# ---- 0. preflight ----
say "0. Preflight"
source ~/rm_setup.sh
WIF="$(ip -br addr | awk '/192\.168\.2\./{print $1; exit}')"
[ -n "$WIF" ] || die "no interface has a 192.168.2.x address. Join the ROBOMASTER-XXXXXX Wi-Fi (robot switch on AP) and rerun."
{
  echo "# captured $(date -Is)"
  echo '$ ip -br addr';  ip -br addr
  echo '$ ip route';     ip route
  echo "\$ nmcli -f GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY dev show $WIF"
  nmcli -f GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY dev show "$WIF"
  echo "\$ iw dev $WIF link"; iw dev "$WIF" link 2>&1
} > logs/ap_host_info.txt
grep -E "$WIF" logs/ap_host_info.txt | head -2
ping -c 3 -W 2 192.168.2.1 >/dev/null 2>&1 || die "robot 192.168.2.1 does not answer ping. Check AP switch/SSID."
echo "robot reachable on $WIF"
BATT=$(timeout 25 python - <<'EOF' 2>/dev/null
import time
from robomaster import robot
ep = robot.Robot(); ep.initialize(conn_type="ap")
g = []
ep.battery.sub_battery_info(freq=5, callback=lambda p: g.append(p))
time.sleep(2); print(g[0] if g else "none")
ep.battery.unsub_battery_info(); ep.close()
EOF
)
[ -n "$BATT" ] && [ "$BATT" != none ] || die "SDK could not connect over ap."
echo "SDK connected, battery ${BATT}%"
[ "${BATT:-0}" -ge 20 ] 2>/dev/null || warn "battery below 20%: consider charging first."

# ---- 1. ping ----
say "1. Ping N=100 (about 20 s)"
scripts/ping_bench.sh ap | tail -3

# ---- 2. SDK teleop ----
say "2. SDK teleoperation, 30 s, THE ROBOT WILL MOVE"
echo "Clear ~1 m around the robot. Keys: W/S fwd/back, A/D strafe, Q/E yaw, SPACE stop, X quit."
echo "Use ALL three motions. Take a screenshot of the live display for the report/video."
read -r -p "Press Enter to start... " _
python ep_teleop_sdk.py --conn ap --duration 30 --out logs/ap_run.csv
python - <<'EOF'
import csv, numpy as np
r = list(csv.DictReader(open('logs/ap_run.csv')))
d = {k: np.array([float(x[k]) for x in r]) for k in r[0]}
print(f"rows={len(r)} duration={d['t_rel_s'][-1]:.1f}s  used: vx={abs(d['cmd_vx']).max()>0} vy={abs(d['cmd_vy']).max()>0} wz={abs(d['cmd_wz']).max()>0}")
EOF
sleep 3   # let the robot release the SDK session before the ROS driver takes it

# ---- 3. ROS 2 driver ----
say "3. ROS 2 driver over AP"
ros2 launch robomaster_ros ep.launch conn_type:=ap > logs/ap_driver.log 2>&1 < /dev/null &
DRIVER_PID=$!
for i in $(seq 1 45); do
  ros2 topic list 2>/dev/null | grep -q '^/odom$' && break
  kill -0 "$DRIVER_PID" 2>/dev/null || { tail -20 logs/ap_driver.log; die "driver exited early (see logs/ap_driver.log)"; }
  sleep 1
done
ros2 topic list 2>/dev/null | grep -q '^/odom$' || die "/odom never appeared (see logs/ap_driver.log)"
{
  echo '$ ros2 node list';          ros2 node list
  echo; echo '$ ros2 topic list';    ros2 topic list
  echo; echo '$ ros2 topic info /cmd_vel -v'; ros2 topic info /cmd_vel -v
  echo; echo '$ ros2 topic hz /odom (8 s)';          timeout 8 ros2 topic hz /odom 2>&1 | tail -4
  echo; echo '$ ros2 topic hz /joint_states (8 s)';  timeout 8 ros2 topic hz /joint_states 2>&1 | tail -4
} 2>&1 | tee logs/ap_ros_verification.txt | tail -25

# ---- 4. TF tree ----
say "4. TF tree"
( cd figures && rm -f frames_*.pdf frames_*.gv && timeout 40 ros2 run tf2_tools view_frames >/dev/null 2>&1; \
  f="$(ls -t frames_*.pdf 2>/dev/null | head -1)"; [ -n "$f" ] && cp "$f" frames.pdf && echo "saved figures/frames.pdf" || warn "view_frames produced no PDF" )
{ echo '$ ros2 topic info /tf -v'; ros2 topic info /tf -v; echo; echo '$ ros2 topic info /tf_static -v'; ros2 topic info /tf_static -v; } > logs/ap_tf_info.txt 2>&1

# ---- 5. bag + keyboard teleop ----
say "5. ROS 2 keyboard teleop with bag recording, THE ROBOT WILL MOVE"
echo "Drive an L-shaped path: forward leg, then a STRAFING leg (sideways), then stop."
echo "teleop_twist_keyboard: i=fwd  k=stop  ,=back  j/l=rotate   SHIFT+J / SHIFT+L = strafe left/right (holonomic)"
echo "Mark the robot's START position on the floor now, and measure its END position with a tape afterwards."
echo "Press Ctrl+C in the teleop window when done. Also record your video now if you want to (second terminal: ros2 topic echo /odom)."
[ -d logs/bag_ap ] && mv logs/bag_ap "logs/bag_ap_old_$(date +%H%M%S)"
read -r -p "Press Enter to start recording and teleop... " _
ros2 bag record -o logs/bag_ap /odom /joint_states /cmd_vel > logs/ap_bag_record.log 2>&1 < /dev/null &
BAG_PID=$!
sleep 3
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# failsafe stop, then close the bag
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1
sleep 1
stop_group "$BAG_PID"; wait "$BAG_PID" 2>/dev/null; BAG_PID=""
ros2 bag info logs/bag_ap | tee logs/ap_bag_info.txt

# ---- 6. odom analysis ----
say "6. Odometry trajectory"
echo "Measured END position of the robot relative to its START (metres): x forward, y LEFT."
read -r -p "x y (e.g. 1.20 0.35), or Enter to skip: " TX TY
if [ -n "${TX:-}" ] && [ -n "${TY:-}" ]; then
  python scripts/plot_odom.py logs/bag_ap --true-end "$TX" "$TY" --out figures | tee logs/ap_odom_summary.txt
else
  python scripts/plot_odom.py logs/bag_ap --out figures | tee logs/ap_odom_summary.txt
fi

# ---- 7. wired data present? ----
if [ -f logs/rndis_run.csv ] && [ "$(wc -l < logs/rndis_run.csv)" -gt 250 ]; then
  say "Wired run found: building comparison"
  python scripts/analyze_telemetry.py --rndis-csv logs/rndis_run.csv --ap-csv logs/ap_run.csv \
    --rndis-ping logs/ping_rndis.txt --ap-ping logs/ping_ap.txt --out figures > logs/stats_stdout.txt 2>&1 && echo "figures/stats.json written"
else
  warn "No complete wired run (logs/rndis_run.csv needs ~300 rows): comparison plots skipped for now."
fi

# ---- done ----
say "DONE. Files to send back to Claude (just say 'AP done'):"
ls -1 logs/ap_* logs/bag_ap figures/frames.pdf figures/fig_odom.png 2>/dev/null
echo
echo "The script will now put your Wi-Fi back to: ${RESTORE_CON:-<none>}"
