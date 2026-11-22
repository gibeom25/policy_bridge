#!/usr/bin/env bash
# CaCoFlow bridge client (see CACOFLOW.md). Kept off the robot's 1 kHz control CPU like run_client.sh.
#   ./run_cacoflow.sh --info-only
#   ./run_cacoflow.sh --instruction "put the blue mug in the bowl" --gripper-one-means closed --trial-id blue_bowl_p01
cd "$(dirname "$(readlink -f "$0")")" || exit 1
source "${CLIENT_VENV:-$HOME/venvs/policy_bridge}/bin/activate"
if [ -z "$CACOFLOW_API_TOKEN" ]; then
  read -rs -p 'CaCoFlow API token: ' CACOFLOW_API_TOKEN; echo
  export CACOFLOW_API_TOKEN
fi
PIN=()
ctl=$(cat .control_cpu 2>/dev/null)          # written by run_server.sh
if [ -n "$ctl" ]; then
  PIN=(taskset -c "$(seq 0 $(($(nproc --all) - 1)) | grep -vx "$ctl" | paste -sd,)")
fi
echo "[run_cacoflow] $(date '+%F %T') start" >> cacoflow.log
"${PIN[@]}" python cacoflow_bridge.py \
  --robot-url ws://127.0.0.1:8000 \
  --ssh-host "${CACOFLOW_SSH_HOST:-155.230.12.123}" \
  --ssh-port "${CACOFLOW_SSH_PORT:-22}" \
  --ssh-user "${CACOFLOW_SSH_USER:-tanvir}" \
  --remote-policy-port 8000 \
  "$@" 2>&1 | tee -a cacoflow.log
