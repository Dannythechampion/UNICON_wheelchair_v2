#!/usr/bin/env bash
# UNICON wheelchair — all-day ROS1 readiness stack, NOT autonomous driving.
# Install in <repo>/src/pid/scripts/ and run ON THE NUC.
# Usage: bash pid_ready.sh [start|status|watch]
#   start:  bring up sensors/localization/perception/base, then watch changes
#   status: snapshot only; never start/restart nodes
#   watch:  watch changes only; never start/restart nodes
# Starting FF-PID is intentionally outside the scope of this script.
set -Eeuo pipefail

MODE="${1:-start}"
case "$MODE" in start|status|watch) ;; *) echo "Usage: $0 [start|status|watch]" >&2; exit 64 ;; esac
[ "$#" -le 1 ] || { echo "Usage: $0 [start|status|watch]" >&2; exit 64; }

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="${REPO_ROOT:-$(cd "$SCRIPT_DIR/../../.." && pwd)}"
LOCALIZATION_WS="${LOCALIZATION_WS:-${REPO_ROOT%/source-linux}/ws}"
LIVOX_WS="${LIVOX_WS:-$HOME/ws_livox}"
FASTLIO_WS="${FASTLIO_WS:-$HOME/fast_lio_ws}"
BASE_WS="${BASE_WS:-$HOME/catkin_ws}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
: "${ROS_MASTER_URI:=http://127.0.0.1:11311}"
export ROS_MASTER_URI

# FF–PID 실제 주행 경로는 나중에 별도로 선택합니다.
# 준비 단계에서 사용할 초기 정합 기준은 항상 A_to_D5의 시작점(A)입니다.
ROUTE2_KEY=A_to_D5
AUTO_INIT_GLOBAL_ONLY=false
VN_IMU="${VN_IMU:-0}"
REGISTRATION_BACKEND="${REGISTRATION_BACKEND:-fast_vgicp_cuda}"
RVIZ="${RVIZ:-false}"
AUTO_INIT_TIMEOUT_S="${AUTO_INIT_TIMEOUT_S:-180}"
FASTLIO_HEALTH_S="${FASTLIO_HEALTH_S:-8.0}"
PERCEPTION_PROFILE="legacy_geometric"

LOG_BASE="${PID_READY_LOG_DIR:-$HOME/pid_ready_logs}"
STATE_DIR="${PID_READY_STATE_DIR:-$HOME/.local/state/pid_ready}"
MONITOR="$SCRIPT_DIR/pid_ready_monitor.py"
BAG_HELPER="$SCRIPT_DIR/pid_ready_bag.sh"

RED=$'\033[31m'; GRN=$'\033[32m'; YLW=$'\033[33m'; NC=$'\033[0m'
if [ ! -t 1 ] || [ "${NO_COLOR:-0}" = 1 ]; then RED=''; GRN=''; YLW=''; NC=''; fi
msg()  { printf '%s\n' "$*"; }
ok()   { printf '%s[OK]%s   %s\n' "$GRN" "$NC" "$*"; }
warn() { printf '%s[WARN]%s %s\n' "$YLW" "$NC" "$*" >&2; }
fail() { printf '%s[FAIL]%s %s\n' "$RED" "$NC" "$*" >&2; exit 1; }
require_file() { [ -f "$1" ] || fail "missing file: $1"; }
source_ws() { require_file "$1/devel/setup.bash"; set +u; source "$1/devel/setup.bash"; set -u; }

require_file "$ROS_SETUP"
# shellcheck disable=SC1090
set +u
source "$ROS_SETUP"
set -u
require_file "$MONITOR"
require_file "$BAG_HELPER"

run_monitor() {
  # Only the display process runs in this terminal. ROS children are detached.
  python3 -u "$MONITOR" "--$1"
}

if [ "$MODE" = status ] || [ "$MODE" = watch ]; then
  timeout 3 rosparam list >/dev/null 2>&1 || fail "ROS Master not reachable: $ROS_MASTER_URI"
  run_monitor "$MODE"
  exit $?
fi

[ "$VN_IMU" = 0 ] || fail "VN_IMU=1 is not in this initial version; MID-360 built-in IMU only"
[ "$REGISTRATION_BACKEND" = fast_vgicp_cuda ] || [ "$REGISTRATION_BACKEND" = pcl_gicp ] || fail "invalid REGISTRATION_BACKEND"
[[ "$AUTO_INIT_TIMEOUT_S" =~ ^[0-9]+$ ]] || fail "AUTO_INIT_TIMEOUT_S must be an integer"
require_file "$REPO_ROOT/tools/route2_intern_env.sh"
require_file "$REPO_ROOT/tools/perception_profile.sh"

# 팀 저장소 지도·경로를 검증하되 초기 정합 경로는 A 시작점으로 고정합니다.
# shellcheck disable=SC1090
export REPO="$REPO_ROOT"
source "$REPO_ROOT/tools/route2_intern_env.sh" || fail "route2 asset selection failed"
INIT_ROUTE="${INIT_ROUTE:-$AUTO_INIT_ROUTE}"
# shellcheck disable=SC1090
source "$REPO_ROOT/tools/perception_profile.sh"
[ "$START_POINTPILLARS" = false ] || fail "this ready script forbids learned/PointPillars detection"
[ "$GEOMETRIC_FIXED_MAP_SUBTRACTION" = false ] || fail "collision geometry must keep mapped walls"
for f in "$MAP" "$TRAJ" "$INIT_ROUTE" "$DRIVABLE_MASK"; do require_file "$f"; done
if [ -n "${TRAJ_SHA256:-}" ]; then
  actual="$(sha256sum "$TRAJ" | awk '{print $1}')"
  [ "$actual" = "$TRAJ_SHA256" ] || fail "trajectory SHA-256 mismatch: $TRAJ"
fi

# Preserve the source map/route identity; verify it before any process starts.
[ -s "$MAP" ] || fail "map is empty"
[ -s "$INIT_ROUTE" ] || fail "initial route is empty"
[ -s "$BAND" ] || fail "safety band is empty"

mkdir -p "$STATE_DIR" "$LOG_BASE"
exec 9>"$STATE_DIR/start.lock"
flock -n 9 || fail "another pid_ready startup is currently running"
RUN_DIR="$LOG_BASE/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR"
PIDS="$RUN_DIR/started_pids.tsv"
: > "$PIDS"
printf '%s\n' "$RUN_DIR" > "$STATE_DIR/latest_run_dir"

# No sweeps, pkill, or auto takeover of a running wheelchair.
# If an earlier session is active, use 'watch' rather than starting over.
master_alive() { timeout 3 rosparam list >/dev/null 2>&1; }
if master_alive; then
  names="$(timeout 4 rosnode list 2>/dev/null || true)"
  for node in /moving_icp_localizer /laserMapping /uart /wheel_cmd \
              /waypoint_follower /dwa_follower /hybrid_geometric_objects \
              /hybrid_object_fusion /localization_exclusion_boxes \
              /safety_gate /tip_guard /terrain_guard /semantic_safety_supervisor; do
    if printf '%s\n' "$names" | grep -Fxq "$node"; then
      fail "$node is already running; do not restart localization. Use: bash $0 watch"
    fi
  done
  # Refuse pre-existing publishers even if a node has a nonstandard name.
  topic_has_publishers() {
    local info
    info="$(timeout 4 rostopic info "$1" 2>/dev/null || true)"
    printf '%s\n' "$info" | awk '
      /^Publishers:/ { in_publishers=1; next }
      /^Subscribers:/ { in_publishers=0 }
      in_publishers && /^[[:space:]]*\*/ { found=1 }
      END { exit !found }'
  }
  for topic in /cmd_vel /wheel_cmd /wheel_status /Odometry \
               /fast_lio_icp/pose /livox/lidar /perception/objects_summary; do
    if topic_has_publishers "$topic"; then
      fail "$topic already has a publisher; use 'watch', never start a second stack"
    fi
  done
fi
# Also check orphaned hardware processes that are no longer registered to a
# master. Restarting ROS over them can leave two live serial/pose producers.
for pattern in '[m]oving_icp_localizer' '[f]astlio_mapping' '[l]ivox_ros_driver2' \
               '[u]art.py' '[w]heel_cmd_tmp.py'; do
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    fail "orphan or existing process matches $pattern; investigate before startup"
  fi
done

# NUC base_model is not in GitHub. Validate which actual package is being used.
source_ws "$BASE_WS"
base_path="$(rospack find base_model 2>/dev/null || true)"
[ -n "$base_path" ] || fail "base_model is absent from BASE_WS: $BASE_WS"
if [ "${ALLOW_NONSTANDARD_BASE:-0}" != 1 ] && [ "$base_path" != "$BASE_WS/src/base_model" ]; then
  fail "unexpected base_model path: $base_path (expected $BASE_WS/src/base_model)"
fi
base_nodes="$(roslaunch --nodes base_model wheel.launch 2>"$RUN_DIR/base_launch_validation.log")" || fail "base_model/wheel.launch cannot be parsed"
for node in /uart /wheel_cmd /odom_pub; do
  printf '%s\n' "$base_nodes" | grep -Fxq "$node" || fail "wheel.launch missing $node; inspect $base_path/launch/wheel.launch"
done
extra_nodes="$(printf '%s\n' "$base_nodes" | grep -Ev '^(/uart|/wheel_cmd|/odom_pub|/joint_state_publisher|/robot_state_publisher)$|^$' || true)"
[ -z "$extra_nodes" ] || fail "wheel.launch starts unexpected nodes: $extra_nodes"

# Avoid serializing full point clouds in the terminal. Topic checks use only
# message headers; all other ROS console output is saved to files.
wait_topic() {
  local topic="$1" max="${2:-25}" i
  for ((i=0; i<max; i++)); do
    if timeout 3 rostopic echo -n1 "$topic" >/dev/null 2>&1; then return 0; fi
    sleep 1
  done
  return 1
}
check_node() {
  local node="$1" i
  for ((i=0;i<12;i++)); do
    timeout 3 rosnode ping -c1 "$node" >/dev/null 2>&1 && return 0
    sleep 1
  done
  return 1
}

start_detached() {
  local name="$1"; shift
  # One independent process group per component. Shell exit / Ctrl+C does
  # not terminate the sensors, the localizer or the base.
  setsid nohup env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
    NUMEXPR_NUM_THREADS=1 VECLIB_MAXIMUM_THREADS=1 \
    "$@" > "$RUN_DIR/$name.log" 2>&1 < /dev/null &
  local pid=$!
  printf '%s\t%s\n' "$name" "$pid" >> "$PIDS"
  sleep 0.2
  kill -0 "$pid" 2>/dev/null || fail "$name exited immediately; see $RUN_DIR/$name.log"
}

# Every error attempts to select manual mode, but NEVER assumes this guarantees
# physical stop. Do not automatically tear down an active UART while in AUTO.
on_failure() {
  local code=$?
  trap - EXIT
  if [ "$code" = 0 ]; then return 0; fi
  warn "startup failed ($code); existing detached nodes may still be running"
  warn "logs: $RUN_DIR"
  if timeout 2 rosnode ping -c1 /uart >/dev/null 2>&1; then
    timeout 4 rostopic pub -1 /mode_cmd std_msgs/Int16 'data: 77' >/dev/null 2>&1 || true
    warn "sent MANUAL request; verify the physical joystick/E-stop"
  fi
  warn "Do not rerun 'start' over surviving nodes. Investigate logs first."
  exit "$code"
}
trap on_failure EXIT

msg '=================================================='
msg '               UNICON PID READY'
msg '=================================================='
msg "Map          : $MAP_ID"
msg "Init route   : $INIT_ROUTE"
msg "Init method  : known-start first, global fallback"
msg "ROS master   : $ROS_MASTER_URI"
msg "Logs         : $RUN_DIR"
msg '--------------------------------------------------'

# Preserve the NUC GPU search library path used by the field launcher.
# The CUDA search is needed only if the known-start prior fails and the
# original auto_initial_pose.py performs its global fallback.
CUDA_PY_ROOT="$HOME/.local/lib/python3.8/site-packages/nvidia"
if [ -d "$CUDA_PY_ROOT" ]; then
  CUDA_PY_LIBS="$(find "$CUDA_PY_ROOT" -mindepth 2 -maxdepth 2 -type d -name lib | sort | paste -sd: -)"
  if [ -n "$CUDA_PY_LIBS" ]; then
    export LD_LIBRARY_PATH="$CUDA_PY_LIBS${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  fi
fi

if master_alive; then
  ok 'ROS Master (existing)'
else
  start_detached roscore roscore
  for ((i=0;i<20;i++)); do master_alive && break; sleep 1; done
  master_alive || fail "roscore did not become available"
  ok 'ROS Master'
fi
rosparam set /use_sim_time false

# Record localization, wheel feedback, manual driving and named map marks.
# The recorder survives Ctrl+C in READY; stop it via pid_ready_bag.sh stop.
bash "$BAG_HELPER" start "$RUN_DIR" || fail "ROS bag recorder could not start"
ok "ROS bag recording: $RUN_DIR"

source_ws "$LIVOX_WS"
start_detached livox roslaunch livox_ros_driver2 msg_MID360.launch
wait_topic '/livox/lidar/header' 30 || fail 'Livox lidar silent; see livox.log'
ok 'Livox MID-360'
wait_topic '/livox/imu/header' 20 || fail 'Livox built-in IMU silent'
ok 'Livox IMU'

source_ws "$FASTLIO_WS"
start_detached fastlio roslaunch fast_lio mapping_mid360.launch rviz:=false
wait_topic '/Odometry/header' 30 || fail 'FAST-LIO Odometry silent'
source_ws "$LOCALIZATION_WS"
if ! rosrun static_livox_localization fastlio_init_health.py \
     _duration_s:="$FASTLIO_HEALTH_S" > "$RUN_DIR/fastlio_health.log" 2>&1; then
  fail 'FAST-LIO stationary initialization check failed; see fastlio_health.log'
fi
ok 'FAST-LIO initialization healthy'

rosparam set /fast_lio_icp/auto_initialization_verified false
rosparam set /fast_lio_icp/auto_initialization_stable false
rosparam set /fast_lio_icp/auto_initialization_source none
start_detached localization roslaunch static_livox_localization moving_localization.launch \
  rviz:="$RVIZ" visualization:=false auto_init:=true auto_init_global_only:=false \
  map_path:="$MAP" map_sha256:="$MAP_SHA256" map_id:="$MAP_ID" \
  auto_init_map:="$MAP" auto_init_traj:="$TRAJ" \
  auto_init_route:="$INIT_ROUTE" \
  auto_init_body_frame_profile:=builtin \
  auto_init_min_refined_score:="${MIN_REFINED_SCORE:-0.65}" \
  auto_init_require_gpu:="${AUTO_INIT_REQUIRE_GPU:-true}" \
  auto_init_gpu_lateral_radius_m:="${AUTO_INIT_GPU_LATERAL_RADIUS_M:-10.0}" \
  auto_init_gpu_lateral_step_m:="${AUTO_INIT_GPU_LATERAL_STEP_M:-1.0}" \
  registration_backend:="$REGISTRATION_BACKEND"
msg '[WAIT] GICP known-start initialization and stationary verification...'
localized=false
start_ts=$SECONDS
while (( SECONDS - start_ts < AUTO_INIT_TIMEOUT_S )); do
  state="$(timeout 3 rostopic echo -n1 /fast_lio_icp/localization_diagnostics/status[0]/message 2>/dev/null | tr -d '"' | head -1 || true)"
  verified="$(timeout 2 rosparam get /fast_lio_icp/auto_initialization_verified 2>/dev/null || echo false)"
  stable="$(timeout 2 rosparam get /fast_lio_icp/auto_initialization_stable 2>/dev/null || echo false)"
  source_kind="$(timeout 2 rosparam get /fast_lio_icp/auto_initialization_source 2>/dev/null || echo none)"
  if [ "$state" = TRACKING ] && [ "$verified" = true ] && [ "$stable" = true ]; then
    case "$source_kind" in known_start_route|global_search) localized=true; break;; esac
  fi
  sleep 1
done
[ "$localized" = true ] || fail "GICP did not reach verified TRACKING; see localization.log"
ok "GICP TRACKING (initialization: $source_kind)"

source_ws "$LOCALIZATION_WS"
start_detached geometry rosrun static_livox_localization hybrid_geometric_objects.py \
  __name:=hybrid_geometric_objects \
  _body_frame_profile:=builtin _drivable_mask:="$DRIVABLE_MASK" \
  _map_path:="$MAP" _map_sha256:="$MAP_SHA256" \
  _fixed_map_subtraction:=false \
  _roi_x_min_m:="$GEOMETRIC_ROI_X_MIN_M" \
  _forward_fov_half_deg:="$GEOMETRIC_FORWARD_FOV_HALF_DEG" \
  _min_cell_points:="$GEOMETRIC_MIN_CELL_POINTS" \
  _min_cluster_points:="$GEOMETRIC_MIN_CLUSTER_POINTS" \
  _max_clusters:="$GEOMETRIC_MAX_CLUSTERS" \
  /perception/objects_summary:=/perception/geometric_objects_summary \
  /perception/dynamic_boxes:=/perception/geometric_exclusion_candidates
start_detached fusion rosrun static_livox_localization hybrid_object_fusion.py \
  __name:=hybrid_object_fusion _route:="$INIT_ROUTE" \
  _body_frame_profile:=builtin _require_learned:=false
# Use the existing GICP exclusion policy unchanged.
start_detached exclusion rosrun static_livox_localization localization_exclusion_boxes.py \
  __name:=localization_exclusion_boxes
wait_topic '/perception/objects_summary' 30 || fail 'fusion object summary silent'
wait_topic '/perception/dynamic_boxes' 20 || fail 'GICP exclusion stream silent'
ok 'Geometric perception + object tracking'
ok 'GICP dynamic-object exclusions'

# Base launch is NUC-only, validated above; does not start the old follower.
source_ws "$BASE_WS"
start_detached base roslaunch base_model wheel.launch
wait_topic '/wheel_status' 25 || fail 'wheel_status silent; check serial port/base controller'
check_node /uart || fail 'UART node missing'
check_node /wheel_cmd || fail 'wheel_cmd node missing'
# Set MANUAL explicitly before reporting any readiness. This is a stop request,
# not a proof that the physical chair has stopped.
timeout 5 rostopic pub -1 /mode_cmd std_msgs/Int16 'data: 77' >/dev/null 2>&1 || fail 'cannot request MANUAL mode'
manual=false
for ((i=0;i<8;i++)); do
  mode_byte="$(timeout 3 rostopic echo -n1 '/wheel_status/data[1]' 2>/dev/null | tr -dc '0-9' || true)"
  [ "$mode_byte" = 77 ] && { manual=true; break; }
  sleep 1
done
[ "$manual" = true ] || fail "base did not acknowledge MANUAL mode"
ok 'UART + wheel base (MANUAL)'

source_ws "$LOCALIZATION_WS"
start_detached stop_watchdog rosrun static_livox_localization stop_watchdog.py
check_node /stop_watchdog || fail 'stop_watchdog node missing'
ok 'Stop watchdog'

# A node can be alive but publish stale/broken data. Validate the complete
# readiness contract (including verified TRACKING and perception status=OK).
python3 "$MONITOR" --check-ready || fail 'ready health check failed (inspect node logs)'

msg '--------------------------------------------------'
msg 'SYSTEM READY — DRIVE NOT STARTED'
msg 'FF-PID must be started separately; existing SafetyGate/TipGuard are NOT active.'
msg 'A real operator-controlled physical stop/joystick must be verified before motion.'
msg "View logs: $RUN_DIR"
msg 'Ctrl+C stops this DISPLAY only; ROS nodes continue.'
msg 'Reattach later: bash pid_ready.sh watch'
msg '--------------------------------------------------'

# The event monitor only subscribes; it never publishes a drive command.
trap - EXIT
run_monitor status
run_monitor watch
