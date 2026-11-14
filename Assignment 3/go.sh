#!/usr/bin/env bash
# Offline robot session: wait for the robot, run everything, record the terminal output.
# Usage:  ./go.sh        (robot Wi-Fi, default)      ./go.sh usb   (USB cable - recommended, no Wi-Fi drops)
# If the link drops, it waits for the robot and continues by itself (saved work is skipped).
cd "$(dirname "$0")"
exec 9>/tmp/rm_go.lock
if ! flock -n 9; then
  OLD=$(pgrep -f "run_trials.py session")
  echo "An older session is still running (PID $OLD) - probably stuck after a Wi-Fi drop."
  read -r -p "Stop it and start fresh? Saved work is kept. [Y/n] " a
  if [[ "$a" =~ ^[Nn] ]]; then exit 1; fi
  pkill -f "run_trials.py session"; sleep 2; pkill -9 -f "run_trials.py session" 2>/dev/null
  flock -w 5 9 || { echo "Could not stop it. Close the other terminal and try again."; exit 1; }
fi
source ~/rm_setup.sh >/dev/null 2>&1
if [ "$1" = "usb" ]; then CONN=rndis; IP=192.168.42.2; else CONN=ap; IP=192.168.2.1
  # Wi-Fi power saving makes this laptop's Realtek card drop the robot network every 1-2 min
  nmcli con modify RMEP-21b442 802-11-wireless.powersave 2 >/dev/null 2>&1
fi
mkdir -p logs/session
while true; do
  echo "Waiting for the robot at $IP ($CONN) ... (Ctrl+C to quit)"
  until ping -c 1 -W 1 "$IP" >/dev/null 2>&1; do sleep 2; printf '.'; done
  echo; echo "Robot found."
  sleep 3
  LOG="logs/session/session_$(date +%Y%m%d_%H%M%S).txt"
  script -q -e -c "python -u run_trials.py session --conn $CONN" "$LOG"
  RC=$?
  [ "$RC" -eq 3 ] || break
  echo; echo "Link lost - will continue automatically as soon as the robot is reachable again."
  sleep 5
done
echo; echo "Finished. Terminal log saved to $LOG"
echo "Now reconnect to normal Wi-Fi and tell Claude Code: session done"
