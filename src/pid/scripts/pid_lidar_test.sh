#!/usr/bin/env bash
# Standalone, non-driving MID-360 connection test.
# start owns a private ROS master and Livox driver; stop owns their shutdown.
set -Eeuo pipefail
MODE="${1:-status}"
case "$MODE" in start|stop|status) ;; *) echo "Usage: $0 {start|status|stop}" >&2; exit 64;; esac
[ "$#" -le 1 ] || exit 64

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STATE_DIR="${PID_LIDAR_TEST_STATE_DIR:-$HOME/.local/state/pid_lidar_test}"
LOG_DIR="${PID_LIDAR_TEST_LOG_DIR:-$HOME/pid_lidar_test_logs}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
LIVOX_WS="${LIVOX_WS:-$HOME/ws_livox}"
MONITOR="$HERE/pid_lidar_test_monitor.py"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
# Never start/stop a master on another host; this is a NUC-local test.
case "$ROS_MASTER_URI" in
  http://127.0.0.1:11311|http://localhost:11311) ;;
  *) echo "[FAIL] LiDAR test requires local ROS_MASTER_URI (127.0.0.1:11311)." >&2; exit 1 ;;
esac
mkdir -p "$STATE_DIR"
exec 9>"$STATE_DIR/control.lock"
flock -x 9

fail() { echo "[FAIL] $*" >&2; exit 1; }
pid_alive() {
  local pid="$1" stat
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  [[ -r "/proc/$pid/stat" ]] || return 1
  stat="$(ps -o stat= -p "$pid" 2>/dev/null || true)"
  [[ -n "$stat" && "$stat" != Z* ]]
}
owned_group() {
  local name="$1" pid pgid cmd
  [[ -s "$STATE_DIR/$name.pid" ]] || return 1
  read -r pid < "$STATE_DIR/$name.pid"
  pid_alive "$pid" || return 1
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]')"
  [[ "$pgid" == "$pid" ]] || return 1
  cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
  case "$name" in
    master) [[ "$cmd" == *roscore* ]] ;;
    driver) [[ "$cmd" == *roslaunch* && "$cmd" == *msg_MID360.launch* ]] ;;
    monitor) [[ "$cmd" == *pid_lidar_test_monitor.py* ]] ;;
    *) return 1 ;;
  esac
}
terminate_owned() {
  local name="$1" pid i
  [[ -s "$STATE_DIR/$name.pid" ]] || return 0
  read -r pid < "$STATE_DIR/$name.pid"
  if ! pid_alive "$pid"; then return 0; fi
  if ! owned_group "$name"; then
    echo "[FAIL] $name PID identity mismatch ($pid); manual inspection required" >&2
    return 1
  fi
  kill -TERM -- "-$pid" 2>/dev/null || true
  for i in {1..35}; do
    pid_alive "$pid" || return 0
    sleep 0.2
  done
  echo "[FAIL] $name did not terminate (PID=$pid)" >&2
  return 1
}
stop_owned() {
  local bad=0
  for name in monitor driver master; do terminate_owned "$name" || bad=1; done
  if (( bad == 0 )); then
    rm -f "$STATE_DIR/active" "$STATE_DIR/status.json" \
      "$STATE_DIR/master.pid" "$STATE_DIR/driver.pid" "$STATE_DIR/monitor.pid"
    echo "[OK] LiDAR test stopped; run READY separately."
  else
    echo "[FAIL] LiDAR test cleanup incomplete. READY must remain blocked." >&2
    return 1
  fi
}
case "$MODE" in
  status)
    if [[ -f "$STATE_DIR/active" ]]; then
      if [[ -f "$STATE_DIR/status.json" ]]; then cat "$STATE_DIR/status.json"
      else printf '{"state":"STARTING","lidar":"WAITING","imu":"WAITING"}\n'; fi
    else
      printf '{"state":"STOPPED","lidar":"UNKNOWN","imu":"UNKNOWN"}\n'
    fi
    exit 0
    ;;
  stop)
    if [[ -f "$STATE_DIR/active" ]]; then stop_owned
    else echo "[OK] LiDAR test is not active."; fi
    exit 0
    ;;
esac

[[ ! -f "$STATE_DIR/active" ]] || fail "LiDAR test is already active. Stop it before restarting."
[[ -f "$ROS_SETUP" ]] || fail "ROS Noetic setup missing: $ROS_SETUP"
[[ -f "$LIVOX_WS/devel/setup.bash" ]] || fail "Livox workspace setup missing: $LIVOX_WS/devel/setup.bash"
[[ -f "$MONITOR" ]] || fail "LiDAR test monitor missing: $MONITOR"
set +u
source "$ROS_SETUP"
source "$LIVOX_WS/devel/setup.bash"
set -u
# Refuse to attach the test to a preexisting ROS session.
if timeout 3 rosparam list >/dev/null 2>&1; then
  fail "ROS Master already exists. Stop READY/other ROS sessions before LiDAR test."
fi
if pgrep -f '[l]ivox_ros_driver2' >/dev/null 2>&1 || \
   pgrep -f '[m]sg_MID360.launch' >/dev/null 2>&1; then
  fail "Existing Livox process detected. Do not start a second driver."
fi
mkdir -p "$LOG_DIR"
RUN_DIR="$(mktemp -d "$LOG_DIR/$(date +%Y%m%d_%H%M%S)_XXXXXX")"
printf '%s\n' "$RUN_DIR" > "$STATE_DIR/active"
printf '{"state":"STARTING","master":"STARTING","driver":"WAITING","lidar":"WAITING","imu":"WAITING"}\n' > "$STATE_DIR/status.json"
start_detached() {
  local name="$1"; shift
  setsid nohup "$@" > "$RUN_DIR/$name.log" 2>&1 < /dev/null 9>&- &
  echo "$!" > "$STATE_DIR/$name.pid"
}
rollback() {
  local exit_code=$?
  trap - EXIT
  if (( exit_code != 0 )); then
    echo "[FAIL] LiDAR test startup failed; attempting cleanup" >&2
    stop_owned || true
  fi
  exit "$exit_code"
}
trap rollback EXIT
start_detached master roscore
master_up=false
for ((i=0; i<50; i++)); do
  if timeout 2 rosparam list >/dev/null 2>&1; then master_up=true; break; fi
  sleep 0.2
done
[[ "$master_up" == true ]] || fail "ROS Master did not start; see $RUN_DIR/master.log"
start_detached driver roslaunch livox_ros_driver2 msg_MID360.launch
sleep 0.5
read -r driver_pid < "$STATE_DIR/driver.pid"
pid_alive "$driver_pid" || fail "Livox driver exited early; see $RUN_DIR/driver.log"
start_detached monitor python3 -u "$MONITOR" --state-dir "$STATE_DIR"
sleep 0.5
read -r monitor_pid < "$STATE_DIR/monitor.pid"
pid_alive "$monitor_pid" || fail "LiDAR monitor exited early; see $RUN_DIR/monitor.log"
trap - EXIT
echo "[OK] LiDAR test started. It runs until the operator presses Test End."
echo "[INFO] Run logs: $RUN_DIR"
