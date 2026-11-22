#!/usr/bin/env bash
# Robot Server launcher:
#   1. stops any running robot_server.py (SIGINT = Ctrl+C, so the controller brakes and stops cleanly)
#   2. refuses to start if another program holds the cameras
#   3. hardware-resets both RealSense cameras and waits until they are back
#   2b. sudo setup_rt.sh: robot NIC EEE/flow control/ASPM off, deep CPU idle states off, NIC IRQs + softirq on one
#       CPU at RT priority (all reset on reboot; Wi-Fi power-save fix is persistent)
#   4. starts the server: 1 kHz control process alone on the NIC IRQ CPU, everything else on the other CPUs
cd "$(dirname "$(readlink -f "$0")")" || exit 1
source "${FRANKA_VENV:-$HOME/venvs/franka}/bin/activate"
ROBOT_IP=${ROBOT_IP:-172.16.0.2}

log() { echo "[run_server] $*" | tee -a server.log; }

# ---- 1. stop an old server --------------------------------------------------------------------------
# relative or full-path launches; only real python processes (not shells whose command line mentions the file)
old=$(for p in $(pgrep -f "python.*[ /]robot_server\.py( |$)"); do
        [ "$p" != "$$" ] && ps -o comm= -p "$p" 2>/dev/null | grep -q '^python' && echo "$p"
      done)
if [ -n "$old" ]; then
  kids=$(for p in $old; do pgrep -P "$p"; done)
  log "stopping old robot_server.py (pid $(echo $old)) with SIGINT ..."
  kill -INT $old 2>/dev/null
  for _ in $(seq 1 100); do                       # up to 10 s for a clean controller stop
    alive=$(for p in $old; do kill -0 "$p" 2>/dev/null && echo "$p"; done)
    [ -z "$alive" ] && break
    sleep 0.1
  done
  if [ -n "$alive" ]; then
    log "old server did not exit, sending SIGTERM/SIGKILL"
    kill -TERM $alive 2>/dev/null; sleep 2; kill -KILL $alive 2>/dev/null
  fi
  for k in $kids; do kill -0 "$k" 2>/dev/null && kill -KILL "$k" 2>/dev/null; done   # orphaned control/gripper processes
  sleep 1
fi

# ---- 2. somebody else holding the cameras? -----------------------------------------------------------
holders=$(for v in /dev/video*; do fuser "$v" 2>/dev/null; done | tr ' ' '\n' | sort -u | grep -v '^$')
if [ -n "$holders" ]; then
  log "ERROR: cameras are in use by other programs - close them first:"
  for p in $holders; do ps -o pid=,cmd= -p "$p" | cut -c1-100 | tee -a server.log; done
  exit 1
fi

# ---- 2b. real-time tuning (sudo: asks for the password unless cached) -----------------------------------
rt_out=$(sudo ROBOT_IP="$ROBOT_IP" ./setup_rt.sh 2>&1); echo "$rt_out" | tee -a server.log
CONTROL_CPU=$(echo "$rt_out" | sed -n 's/^CONTROL_CPU=//p')
[ -n "$CONTROL_CPU" ] || { log "ERROR: setup_rt.sh failed, not starting the server"; exit 1; }
OTHER_CPUS=$(seq 0 $(($(nproc --all) - 1)) | grep -vx "$CONTROL_CPU" | paste -sd,)
echo "$CONTROL_CPU" > .control_cpu     # read by run_client.sh

# ---- 3. power-cycle the cameras ----------------------------------------------------------------------
python -u reset_cameras.py 2>&1 | tee -a server.log
[ "${PIPESTATUS[0]}" -eq 0 ] || { log "ERROR: camera reset failed, not starting the server"; exit 1; }

# ---- 4. start --------------------------------------------------------------------------------------------
log "starting robot_server.py $(date '+%F %T'): control process on CPU $CONTROL_CPU, everything else on $OTHER_CPUS"
taskset -c "$OTHER_CPUS" python -u robot_server.py --ip "$ROBOT_IP" --port 8000 --control-cpu "$CONTROL_CPU" "$@" \
  2>&1 | tee -a server.log      # extra args (e.g. --v-max 0.4, --joint-impedance ...) go straight to robot_server.py
