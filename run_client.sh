#!/usr/bin/env bash
# Bridge client: kept off the CPU that runs the robot's 1 kHz control process (= the FCI NIC IRQ CPU).
# Extra options are passed through, e.g.  ./run_client.sh --max-decisions 5
cd "$(dirname "$(readlink -f "$0")")" || exit 1
source "${CLIENT_VENV:-$HOME/venvs/policy_bridge}/bin/activate"
ctl=$(cat .control_cpu 2>/dev/null)          # written by run_server.sh
[ -n "$ctl" ] || { echo "[run_client] start run_server.sh first (no .control_cpu)"; exit 1; }
OTHER_CPUS=$(seq 0 $(($(nproc --all) - 1)) | grep -vx "$ctl" | paste -sd,)
echo "[run_client] $(date '+%F %T') start (avoiding control CPU $ctl)" >> client.log
taskset -c "$OTHER_CPUS" python robot_client2.py \
  --robot-url ws://127.0.0.1:8000 \
  --ssh-host 10.103.1.92 \
  --ssh-port 5544 \
  --ssh-user lameya \
  --remote-policy-port 8001 \
  --mode correct \
  --prompt "pick up the red mug and place it on the plate" \
  "$@" 2>&1 | tee -a client.log
