#!/usr/bin/env bash
# Apply the RT tuning, then measure wake-up latency for 60 s on the control CPU and two others.
# Usage:  sudo ./rt_check.sh        (results also in rtla.log next to this script)
cd "$(dirname "$(readlink -f "$0")")" || exit 1
[ "$(id -u)" -eq 0 ] || { echo "run with sudo"; exit 1; }
{
  echo "===== rt_check $(date '+%F %T')"
  ./setup_rt.sh
  rtla timerlat top -d 60s -a 1000 -q
  echo "===== rt_check done (exit $?)"
} 2>&1 | tee -a rtla.log
chown "${SUDO_USER:-root}": rtla.log timerlat_trace.txt 2>/dev/null
