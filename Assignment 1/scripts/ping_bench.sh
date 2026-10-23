#!/usr/bin/env bash
# Ping the robot N=100 times (0.2 s apart) and save the raw output.
# Usage: scripts/ping_bench.sh <rndis|ap> [count]
#   rndis -> 192.168.42.2   ap -> 192.168.2.1
set -euo pipefail
mode="${1:?usage: ping_bench.sh <rndis|ap> [count]}"
count="${2:-100}"
case "$mode" in
  rndis) ip=192.168.42.2 ;;
  ap)    ip=192.168.2.1 ;;
  *) echo "mode must be rndis or ap" >&2; exit 1 ;;
esac
out="$(dirname "$0")/../logs/ping_${mode}.txt"
ping -c "$count" -i 0.2 "$ip" | tee "$out"
