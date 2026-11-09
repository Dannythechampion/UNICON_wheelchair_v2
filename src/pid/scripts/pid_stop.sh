#!/usr/bin/env bash
# Stop FF-PD only. READY sensors/localization/ROS bag remain running.
set -Eeuo pipefail
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
CHECK="$SCRIPT_DIR/pid_drive_check.py"
msg() { printf '%s\n' "$*"; }
fail() { printf '[FAIL] %s\n' "$*" >&2; exit 1; }
[[ "$#" == 0 ]] || fail '사용법: bash src/pid/scripts/pid_stop.sh'
[[ -f "$ROS_SETUP" ]] || fail "ROS 설치 파일 없음: $ROS_SETUP"
[[ -f "$CHECK" ]] || fail "검사 파일 없음: $CHECK"
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
: "${ROS_MASTER_URI:=http://127.0.0.1:11311}"
export ROS_MASTER_URI
timeout 4 rosparam list >/dev/null 2>&1 || msg '[WARN] ROS Master unreachable; physical stop cannot be verified. Attempting recorded-process cleanup.'

node_alive() { timeout 3 rosnode list 2>/dev/null | grep -Fx /ff_pid >/dev/null; }
request_manual() { timeout 6 rostopic pub -1 /mode_cmd std_msgs/Int16 'data: 77' >/dev/null 2>&1; }

if node_alive; then
  timeout 5 rosservice call /ff_pid/start 'data: false' >/dev/null 2>&1 || \
    msg '[WARN] FF-PD service unavailable; MANUAL is still requested.'
else
  msg '[INFO] FF-PD node absent; continuing idempotent STOP.'
fi
request_manual || msg '[WARN] MANUAL request failed; verify physical joystick/E-stop.'
VERIFIED=1
if ! python3 "$CHECK" wheel --mode MANUAL --stopped --timeout 4; then
  VERIFIED=0
  msg '[WARN] Physical stopped+MANUAL state NOT VERIFIED (NO_DATA is not stopped).'
fi
if node_alive; then
  timeout 5 rosnode kill /ff_pid >/dev/null 2>&1 || true
fi
timeout 5 rosnode kill /pid_safety >/dev/null 2>&1 || true
timeout 5 rosnode kill /pid_astar_planner >/dev/null 2>&1 || true
STATE_DIR="${PID_READY_STATE_DIR:-$HOME/.local/state/pid_ready}"
if [[ -s "$STATE_DIR/latest_run_dir" ]]; then
  IFS= read -r RUN_DIR < "$STATE_DIR/latest_run_dir"
  for item in ff_pid.pid pid_safety.pid pid_astar_planner.pid; do
    [[ -s "$RUN_DIR/$item" ]] || continue
    IFS= read -r pid < "$RUN_DIR/$item"
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
    [[ "$pgid" == "$pid" ]] || continue
    cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
    if [[ ( "$item" == ff_pid.pid && "$cmd" == *ff_pid_node.py* ) ||
          ( "$item" == pid_safety.pid && "$cmd" == *pid_safety.py* ) ||
          ( "$item" == pid_astar_planner.pid && "$cmd" == *pid_astar_planner.py* ) ]]; then
      kill -TERM -- "-$pgid" 2>/dev/null || true
    fi
  done
fi
if node_alive; then
  msg '[WARN] FF-PD node remains registered; inspect its logged process.'
  exit 1
fi
if [[ "$VERIFIED" != 1 ]]; then
  msg '[WARN] FF-PD process cleanup requested, but physical stop is NOT verified.'
  exit 2
fi
msg '[OK] FF-PD and Safety ended. READY sensor/GICP/bag remain running.'
