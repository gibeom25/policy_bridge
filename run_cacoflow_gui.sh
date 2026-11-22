#!/usr/bin/env bash
# CaCoFlow trial GUI (see CACOFLOW.md, "GUI"). Opens http://127.0.0.1:${GUI_PORT:-8080}/
#   ./run_cacoflow_gui.sh
# If the Robot Server is not running, asks for the sudo password once (setup_rt.sh needs root) and lets the GUI
# start the Robot Server itself with the controller settings saved in the GUI. Nothing moves the robot until
# you press "로봇 준비" (connect) and "시행 시작" in the browser.
cd "$(dirname "$(readlink -f "$0")")" || exit 1
PORT="${GUI_PORT:-8080}"
source "${CLIENT_VENV:-$HOME/venvs/policy_bridge}/bin/activate"

if ss -ltn 2>/dev/null | grep -q ":${PORT} "; then
  echo "[run_cacoflow_gui] port ${PORT} already in use (GUI already running?). Set GUI_PORT=<other>."; exit 1
fi
if [ -f run_server.sh ] && ! ss -ltn 2>/dev/null | grep -q ":8000 " && ! pgrep -f "python.*[ /]robot_server\.py" >/dev/null; then
  echo "[run_cacoflow_gui] Robot Server is not running: sudo password needed once for the RT setup"
  sudo -v || { echo "[run_cacoflow_gui] sudo failed; the GUI will start but cannot start the Robot Server"; }
fi
export CACOFLOW_GUI_AUTOSTART=1
PIN=()
ctl=$(cat .control_cpu 2>/dev/null)          # written by run_server.sh; the GUI re-pins itself after a server start
if [ -n "$ctl" ] && ss -ltn 2>/dev/null | grep -q ":8000 "; then
  PIN=(taskset -c "$(seq 0 $(($(nproc --all) - 1)) | grep -vx "$ctl" | paste -sd,)")
fi
(sleep 2; xdg-open "http://127.0.0.1:${PORT}/" >/dev/null 2>&1) &
echo "[run_cacoflow_gui] $(date '+%F %T') start" >> cacoflow_gui.log
"${PIN[@]}" python cacoflow_gui.py --port "$PORT" 2>&1 | tee -a cacoflow_gui.log
