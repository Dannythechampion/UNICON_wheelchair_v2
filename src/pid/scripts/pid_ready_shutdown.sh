#!/usr/bin/env bash
# READY cleanup: stop driving, finalize bag, stop process groups of this READY
# session even after startup failure. Missing feedback is not physical stop.
set -Eeuo pipefail
S="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="${PID_READY_STATE_DIR:-$HOME/.local/state/pid_ready}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
MODE="${1:-normal}"
if [[ "$MODE" == --startup-failure ]]; then
  [[ "$#" == 2 ]] || exit 64
  RUN_DIR="$2"
elif [[ "$MODE" == normal ]]; then
  [[ "$#" == 0 ]] || exit 64
  [[ -s "$STATE_DIR/latest_run_dir" ]] || { echo '[OK] No READY run to stop'; exit 0; }
  IFS= read -r RUN_DIR < "$STATE_DIR/latest_run_dir"
else
  echo 'usage: pid_ready_shutdown.sh [--startup-failure RUN_DIR]' >&2
  exit 64
fi
[[ -d "$RUN_DIR" && -f "$RUN_DIR/started_pids.tsv" ]] || {
  echo "[FAIL] READY registry missing: $RUN_DIR" >&2; exit 1;
}
[[ -r "$ROS_SETUP" ]] || { echo '[FAIL] ROS setup missing' >&2; exit 1; }
set +u
source "$ROS_SETUP"
set -u
warn() { printf '[WARN] %s\n' "$*" >&2; }
info() { printf '[INFO] %s\n' "$*"; }
master_alive() { timeout 3 rosparam list >/dev/null 2>&1; }
node_alive() { timeout 3 rosnode list 2>/dev/null | grep -Fx "$1" >/dev/null; }
failed=0

if master_alive; then
  if node_alive /ff_pid; then
    info 'Drive STOP requested before READY shutdown.'
    bash "$S/pid_stop.sh" || warn 'Drive stop not verifiable; physically disable motor if needed.'
  else
    timeout 4 rostopic pub -1 /mode_cmd std_msgs/Int16 'data: 77' >/dev/null 2>&1 || true
  fi
else
  warn 'ROS Master unavailable; physical motor state cannot be verified.'
fi

for name in ff_pid.pid pid_safety.pid; do
  [[ -s "$RUN_DIR/$name" ]] || continue
  IFS= read -r pid < "$RUN_DIR/$name"
  [[ "$pid" =~ ^[0-9]+$ ]] || continue
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
  [[ "$pgid" == "$pid" ]] || continue
  cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
  if [[ ( "$name" == ff_pid.pid && "$cmd" == *ff_pid_node.py* ) ||
        ( "$name" == pid_safety.pid && "$cmd" == *pid_safety.py* ) ]]; then
    kill -TERM -- "-$pgid" 2>/dev/null || true
  fi
done

# Request SIGINT to finalize rosbag before ROS publishers are stopped.
if [[ -s "$RUN_DIR/rosbag.pid" ]]; then
  IFS= read -r pid < "$RUN_DIR/rosbag.pid"
  if [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/cmdline" ]]; then
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
    cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
    if [[ "$pgid" == "$pid" && "$cmd" == *rosbag* && "$cmd" == *record* ]]; then
      info "Closing rosbag PID=$pid"
      kill -INT -- "-$pgid" 2>/dev/null || true
      for ((j=0;j<40;j++)); do
        state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
        [[ -z "$state" || "$state" == Z* ]] && break
        sleep 0.25
      done
      state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
      if [[ -n "$state" && "$state" != Z* ]]; then
        warn "Rosbag not confirmed closed: PID=$pid"
        failed=1
      fi
    elif [[ -n "$cmd" ]]; then
      warn "Recorded ROS bag PID no longer identifies its recorder: $pid"
      failed=1
    fi
  fi
fi

shopt -s nullglob
active_bags=("$RUN_DIR"/ready_topics*.bag.active)
if (( ${#active_bags[@]} )); then
  warn "Bag segments still active/incomplete: ${active_bags[*]}"
  failed=1
fi
shopt -u nullglob

while IFS=$'\t' read -r name pid; do
  [[ "$pid" =~ ^[0-9]+$ ]] || continue
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
  [[ "$pgid" == "$pid" ]] || continue
  cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
  case "$name" in
    roscore) expected='roscore' ;;
    livox) expected='msg_MID360.launch' ;;
    fastlio) expected='mapping_mid360.launch' ;;
    localization) expected='moving_localization.launch' ;;
    base) expected='wheel.launch' ;;
    geometry) expected='hybrid_geometric_objects.py' ;;
    fusion) expected='hybrid_object_fusion.py' ;;
    exclusion) expected='localization_exclusion_boxes.py' ;;
    stop_watchdog) expected='stop_watchdog.py' ;;
    *) warn "Unknown registered component: $name"; failed=1; continue ;;
  esac
  [[ "$cmd" == *"$expected"* ]] || {
    warn "PID identity changed: $name $pid (not signalled)"
    failed=1
    continue
  }
  info "Stopping $name process group $pgid"
  kill -TERM -- "-$pgid" 2>/dev/null || true
done < <(tac "$RUN_DIR/started_pids.tsv")

sleep 2
while IFS=$'\t' read -r name pid; do
  [[ "$pid" =~ ^[0-9]+$ ]] || continue
  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]' || true)"
  if [[ -n "$state" && "$state" != Z* ]]; then
    cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
    if [[ "$cmd" == *roslaunch* || "$cmd" == *roscore* || "$cmd" == *python* ]]; then
      warn "Registered process still running: $name pid=$pid"
      failed=1
    fi
  fi
done < "$RUN_DIR/started_pids.tsv"

if [[ "$failed" -eq 0 ]]; then
  if [[ -s "$STATE_DIR/latest_run_dir" ]]; then
    IFS= read -r recorded_run < "$STATE_DIR/latest_run_dir"
    if [[ "$recorded_run" == "$RUN_DIR" ]]; then
      rm -f "$STATE_DIR/latest_run_dir"
    fi
  fi
  echo '[OK] READY processes terminated; bag closure checked.'
else
  warn 'Cleanup incomplete. Check PIDs and bag before restarting.'
fi
warn 'MANUAL command is not physical stop confirmation.'

# Background-only ROS bag postprocessing. Does not alter READY shutdown result,
# talk to ROS Master, command the motor or remove originals.
# The organizer independently skips sessions whose recording/processes are alive.
AUTO_ORGANIZER="$S/pid_rosbag_auto.py"
if [[ -f "$AUTO_ORGANIZER" && "${PID_ROSBAG_AUTO:-1}" == 1 ]]; then
  setsid nohup bash -c 'source "$1"; exec python3 "$2"' \
    _ "$ROS_SETUP" "$AUTO_ORGANIZER" \
    >"$RUN_DIR/rosbag_auto.log" 2>&1 </dev/null 9>&- &
  info "ROS bag organization queued in background; log: $RUN_DIR/rosbag_auto.log"
fi
exit "$failed"
