#!/usr/bin/env bash
# Record only lightweight ROS1 topics for READY, manual driving, and FF-PD.
# Usage: bash pid_ready_bag.sh start <run_dir>
#        bash pid_ready_bag.sh status|stop|mark <LABEL>
set -Eeuo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
STATE_DIR="${PID_READY_STATE_DIR:-$HOME/.local/state/pid_ready}"
MODE="${1:-}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "[FAIL] ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

latest_run_dir() {
  local f="$STATE_DIR/latest_run_dir"
  [[ -s "$f" ]] || { echo "[FAIL] No READY run has been recorded ($f)" >&2; return 1; }
  local p
  IFS= read -r p < "$f"
  [[ -d "$p" ]] || { echo "[FAIL] READY run directory not found: $p" >&2; return 1; }
  printf '%s\n' "$p"
}

read_pid() {
  local f="$1/rosbag.pid"
  [[ -s "$f" ]] || return 1
  local p
  IFS= read -r p < "$f"
  [[ "$p" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "$p"
}

recorder_alive() {
  local pid="$1"
  kill -0 "$pid" 2>/dev/null || return 1
  [[ -r "/proc/$pid/cmdline" ]] || return 1
  # Avoid accidentally signalling a reused PID.
  local cmd
  cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline")"
  [[ "$cmd" == *rosbag* && "$cmd" == *record* ]]
}

case "$MODE" in
  start)
    [[ "$#" == 2 ]] || { echo "Usage: $0 start <run_dir>" >&2; exit 64; }
    RUN_DIR="$2"
    [[ -d "$RUN_DIR" ]] || { echo "[FAIL] RUN_DIR missing: $RUN_DIR" >&2; exit 1; }
    # Do not silently start a second recorder for one READY session.
    if pid="$(read_pid "$RUN_DIR")" && recorder_alive "$pid"; then
      echo "[FAIL] ROS bag recorder is already running: PID $pid" >&2
      exit 1
    fi
    command -v rosbag >/dev/null || { echo "[FAIL] rosbag command not found" >&2; exit 1; }
    free_mb="$(df -Pm "$RUN_DIR" | awk 'NR==2 {print $4}')"
    if [[ ! "$free_mb" =~ ^[0-9]+$ ]] || (( free_mb < 2048 )); then
      echo "[FAIL] ROS bag needs at least 2 GiB of free disk space (free: ${free_mb:-unknown} MiB)" >&2
      exit 1
    fi
    # In its own session: Ctrl+C in the READY monitor must not kill recording.
    # Split segments every 15 min or 512 MB, whichever occurs first.
    # Bash ignores SIGINT for asynchronous jobs. Reset it explicitly before
    # exec so SIGINT finalizes the .bag instead of being silently ignored.
    setsid nohup python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
      rosbag record \
      --split --size=512 --duration=15m \
      -o "$RUN_DIR/ready_topics" \
      /fast_lio_icp/pose \
      /fast_lio_icp/localization_diagnostics \
      /Odometry \
      /livox/imu \
      /wheel_status \
      /wheel_cmd \
      /mode_cmd \
      /cmd_vel \
      /ff_pid/status \
      /ff_pid/motion_policy \
      /ff_pid/replan/request \
      /ff_pid/replan/candidate \
      /ff_pid/replan/stage \
      /ff_pid/replan/commit \
      /ff_pid/replan/ack \
      /ff_pid/active_route \
      /pid_safety/status \
      /perception/objects_summary \
      /uart_tx_diag \
      /stop_watchdog/alarm \
      /manual_mark \
      >"$RUN_DIR/rosbag.log" 2>&1 </dev/null &
    pid=$!
    printf '%s\n' "$pid" > "$RUN_DIR/rosbag.pid"
    sleep 1
    if ! recorder_alive "$pid"; then
      echo "[FAIL] ROS bag exited immediately. Inspect: $RUN_DIR/rosbag.log" >&2
      exit 1
    fi
    echo "[OK] ROS bag recorder active (PID $pid); run: $RUN_DIR"
    echo "[INFO] Recording starts before sensor/localization startup."
    ;;

  status)
    [[ "$#" == 1 ]] || { echo "Usage: $0 status" >&2; exit 64; }
    RUN_DIR="$(latest_run_dir)"
    if pid="$(read_pid "$RUN_DIR")" && recorder_alive "$pid"; then
      echo "[OK] Recorder active: PID $pid"
    else
      echo "[WARN] Recorder not active"
    fi
    echo "[INFO] Recording directory: $RUN_DIR"
    find "$RUN_DIR" -maxdepth 1 -type f \( -name 'ready_topics*.bag' -o -name 'ready_topics*.bag.active' \) -printf '%f (%s bytes)\n' | sort
    ;;

  mark)
    [[ "$#" == 2 ]] || { echo "Usage: $0 mark START_A|START_D4|BUMP_ENTRY|BUMP_EXIT" >&2; exit 64; }
    label="$2"
    [[ "$label" =~ ^[A-Za-z0-9_.:-]+$ ]] || { echo "[FAIL] Invalid mark label: $label" >&2; exit 64; }
    RUN_DIR="$(latest_run_dir)"
    if ! pid="$(read_pid "$RUN_DIR")" || ! recorder_alive "$pid"; then
      echo "[FAIL] ROS bag recorder is not active. Mark would be lost." >&2
      exit 1
    fi
    # Publish repeatedly for 3s so rosbag has time to connect to this
    # transient topic. Repeated records of one label represent ONE mark.
    set +e
    timeout --signal=INT 3 rostopic pub -r 5 /manual_mark std_msgs/String "data: '$label'" >/dev/null 2>&1
    rc=$?
    set -e
    if [[ "$rc" != 0 && "$rc" != 124 ]]; then
      echo "[FAIL] Could not publish $label to /manual_mark (exit $rc)" >&2
      exit "$rc"
    fi
    echo "[OK] Published $label to /manual_mark (check bag contents after recording)."
    ;;

  stop)
    [[ "$#" == 1 ]] || { echo "Usage: $0 stop" >&2; exit 64; }
    RUN_DIR="$(latest_run_dir)"
    if ! pid="$(read_pid "$RUN_DIR")" || ! recorder_alive "$pid"; then
      echo "[WARN] Recorder not active. No stop signal sent." >&2
      exit 1
    fi
    # start runs 'setsid', so PID is the recorder's isolated process group.
    pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"
    if [[ "$pgid" != "$pid" ]]; then
      echo "[FAIL] Recorder PGID does not match recorded PID; refusing to signal other processes." >&2
      exit 1
    fi
    echo "[INFO] Gracefully closing ROS bag (SIGINT to recorder process group $pgid)..."
    kill -INT -- "-$pgid"
    for (( i=0; i<60; i++ )); do
      if ! recorder_alive "$pid"; then break; fi
      sleep 0.5
    done
    if recorder_alive "$pid"; then
      echo "[WARN] Recorder is still running; not force-killing it. Check: $RUN_DIR/rosbag.log" >&2
      exit 1
    fi
    shopt -s nullglob
    active=("$RUN_DIR"/ready_topics*.bag.active)
    if (( ${#active[@]} > 0 )); then
      echo "[WARN] Recorder exited but .bag.active remains; inspect before using: ${active[*]}" >&2
      exit 1
    fi
    echo "[OK] ROS bag stopped and closed in: $RUN_DIR"
    echo "[INFO] Sensor/localization/base ROS nodes were not stopped."
    ;;

  *)
    echo "Usage: $0 start <run_dir> | status | mark <LABEL> | stop" >&2
    exit 64
    ;;
esac
