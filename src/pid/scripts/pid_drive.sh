#!/usr/bin/env bash
# One command to choose a route and START FF-PD. READY must already be running.
# No prepare/status/pause subcommands. The separate pid_stop.sh handles stopping.
set -Eeuo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
BASE_WS="${BASE_WS:-$HOME/catkin_ws}"
LOCALIZATION_WS="${LOCALIZATION_WS:-${REPO_ROOT%/source-linux}/ws}"
STATE_DIR="${PID_READY_STATE_DIR:-$HOME/.local/state/pid_ready}"
CHECK="$SCRIPT_DIR/pid_drive_check.py"
MONITOR="$SCRIPT_DIR/pid_ready_monitor.py"
NODE="$SCRIPT_DIR/ff_pid_node.py"
SAFETY="$SCRIPT_DIR/pid_safety.py"
BAG_HELPER="$SCRIPT_DIR/pid_ready_bag.sh"
STOP="$SCRIPT_DIR/pid_stop.sh"

# JSON event lines are intentionally mixed with human-readable shell output:
# a future bridge must select frames with type=drive_event / drive_preflight.
# Never confuse script launch with successful motor/controller START.
emit_drive_event() {
  local status="$1" reason="$2" detail="$3"
  python3 -c 'import json,sys; print(json.dumps({"type":"drive_event","status":sys.argv[1],"reason":sys.argv[2],"route":sys.argv[3] or None,"message":sys.argv[4]},ensure_ascii=False,separators=(",",":")),flush=True)' \
    "$status" "$reason" "${ROUTE:-}" "$detail" || true
}
fail() {
  printf '[FAIL] %s\n' "$*" >&2
  emit_drive_event REJECTED SCRIPT_CHECK_FAILED "$*" 
  exit 1
}
need() { [[ -f "$1" ]] || fail "파일 없음: $1"; }

ROUTES=(A_to_D4 A_to_D5 D4_to_A D4_to_D5 D5_to_A D5_to_D4)
select_route() {
  local input="${1:-}" i
  if [[ -z "$input" ]]; then
    [[ -t 0 ]] || fail "경로를 인자로 지정하세요: bash src/pid/scripts/pid_drive.sh A_to_D5"
    printf '\n주행할 경로를 선택하세요.\n' >&2
    for i in "${!ROUTES[@]}"; do printf '  %d) %s\n' "$((i+1))" "${ROUTES[$i]}" >&2; done
    printf '번호(1-6): ' >&2
    IFS= read -r input || fail '경로 선택이 취소됐습니다.'
  fi
  if [[ "$input" =~ ^[1-6]$ ]]; then input="${ROUTES[$((input-1))]}"; fi
  for i in "${ROUTES[@]}"; do [[ "$i" == "$input" ]] && { printf '%s\n' "$input"; return 0; }; done
  fail "잘못된 경로: $input"
}

[[ "$#" -le 2 ]] || fail '사용법: bash src/pid/scripts/pid_drive.sh [A_to_D5|1-6] [--test-clear]'
TEST_CLEAR=false
if [[ "$#" -eq 2 ]]; then
  [[ "$2" == "--test-clear" ]] || fail '두 번째 인자는 --test-clear만 사용할 수 있습니다.'
  [[ "$1" != "position" ]] || fail 'position에는 --test-clear를 사용할 수 없습니다.'
  TEST_CLEAR=true
fi
ROUTE=""
if [[ "${1:-}" != "position" ]]; then
  ROUTE="$(select_route "${1:-}")"
fi
for f in "$ROS_SETUP" "$CHECK" "$MONITOR" "$NODE" "$SAFETY" "$STOP"; do need "$f"; done
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
for ws in "$BASE_WS" "$LOCALIZATION_WS"; do
  need "$ws/devel/setup.bash"
  set +u
  # shellcheck disable=SC1090
  source "$ws/devel/setup.bash"
  set -u
done
export ROS_PACKAGE_PATH="$REPO_ROOT/src${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
: "${ROS_MASTER_URI:=http://127.0.0.1:11311}"
export ROS_MASTER_URI

timeout 4 rosparam list >/dev/null 2>&1 || fail 'READY가 실행되지 않았거나 ROS Master에 연결되지 않습니다.'
if [[ "${1:-}" == "position" ]]; then
  python3 "$CHECK" position
  exit $?
fi
[[ -s "$STATE_DIR/latest_run_dir" ]] || fail 'READY 실행 기록이 없습니다. pid_ready.sh부터 실행하세요.'
IFS= read -r RUN_DIR < "$STATE_DIR/latest_run_dir"
[[ -d "$RUN_DIR" ]] || fail "READY 로그 디렉터리가 없습니다: $RUN_DIR"

# Ensure ROS package resolution uses this checkout. Never launch a second controller.
pid_path="$(rospack find pid 2>/dev/null || true)"
loc_path="$(rospack find static_livox_localization 2>/dev/null || true)"
[[ "$(readlink -f "$pid_path")" == "$(readlink -f "$REPO_ROOT/src/pid")" ]] || fail "잘못된 pid 패키지 경로: $pid_path"
[[ "$(readlink -f "$loc_path")" == "$(readlink -f "$REPO_ROOT/src/static_livox_localization")" ]] || fail "잘못된 localization 패키지 경로: $loc_path"

if timeout 3 rosnode list 2>/dev/null | grep -Ex '/ff_pid|/pid_safety' >/dev/null; then
  fail '/ff_pid 또는 /pid_safety가 이미 실행 중입니다. 중지하려면 pid_stop.sh를 실행하세요.'
fi
info="$(timeout 5 rostopic info /cmd_vel 2>/dev/null || true)"
if printf '%s\n' "$info" | awk '/^Publishers:/ {p=1;next} /^Subscribers:/ {p=0} p && /^[[:space:]]*\*/ {found=1} END {exit !found}'; then
  fail '/cmd_vel의 다른 발행자가 존재합니다. 중복 제어를 허용하지 않습니다.'
fi

# Read-only preflight. Compares chair pose with the WHOLE selected route, not its START waypoint.
python3 "$MONITOR" --check-ready || fail 'READY 점검 실패. 수동 모드를 유지합니다.'
python3 "$CHECK" wheel --mode MANUAL --stopped || fail 'MANUAL 정차 상태를 확인할 수 없습니다.'
# Require a compatible pose anywhere on the selected route before launching FF-PD.
# The check prints turn-first, approach-second guidance on refusal.
# The pose checker emits one typed JSON line on stdout for the future app bridge
# and the two human-readable alignment guides to stderr. No continuous polling.
if ! python3 "$CHECK" pose-on-route --json --route "$ROUTE" \
  --max-distance 1.0 --max-heading-rad 0.5235987756; then
  fail '출발 위치 점검 실패. MANUAL로 조정한 뒤 주행 시작을 다시 누르세요.'
fi

if [[ -f "$BAG_HELPER" ]]; then
  bag_status="$(bash "$BAG_HELPER" status 2>&1 || true)"
  if ! grep -Fq '[OK] Recorder active:' <<< "$bag_status"; then
    printf '[WARN] READY ROS bag 녹화 상태를 확인하지 못했습니다.\n' >&2
  fi
fi

LOG="$RUN_DIR/ff_pid_${ROUTE}_$(date +%Y%m%d_%H%M%S).log"
setsid nohup env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python3 -u "$NODE" "_route:=$ROUTE" '_body_frame_profile:=builtin' \
  > "$LOG" 2>&1 < /dev/null &
NODE_PID=$!
printf '%s\n' "$NODE_PID" > "$RUN_DIR/ff_pid.pid"

ready=0
for ((i=0; i<35; i++)); do
  if timeout 2 rosservice info /ff_pid/start >/dev/null 2>&1 &&
     timeout 2 rosnode list 2>/dev/null | grep -Fx /ff_pid >/dev/null; then
    ready=1; break
  fi
  kill -0 "$NODE_PID" 2>/dev/null || break
  sleep 0.3
done
[[ "$ready" == 1 ]] || { bash "$STOP" >/dev/null 2>&1 || true; fail "FF-PD 노드 시작 실패. 로그: $LOG"; }

SAFETY_LOG="$RUN_DIR/pid_safety_${ROUTE}_$(date +%Y%m%d_%H%M%S).log"
# Always overwrite the private ROS param: the previous test=true can survive node exit.
SAFETY_PARAMS=("_route:=$ROUTE" "_test_clear:=false")
if [[ "$TEST_CLEAR" == true ]]; then
  SAFETY_PARAMS=("_route:=$ROUTE" "_test_clear:=true")
  printf '[WARN] TEST CLEAR: 장애물/마스크 감시 없이 CLEAR만 발행합니다.\n' >&2
fi
setsid nohup env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python3 -u "$SAFETY" "${SAFETY_PARAMS[@]}" \
  > "$SAFETY_LOG" 2>&1 < /dev/null &
SAFETY_PID=$!
printf '%s\n' "$SAFETY_PID" > "$RUN_DIR/pid_safety.pid"
safety_ready=0
for ((i=0; i<35; i++)); do
  if timeout 2 rosnode list 2>/dev/null | grep -Fx /pid_safety >/dev/null; then
    safety_ready=1; break
  fi
  kill -0 "$SAFETY_PID" 2>/dev/null || break
  sleep 0.3
done
[[ "$safety_ready" == 1 ]] || { bash "$STOP" >/dev/null 2>&1 || true; fail "Safety 노드 시작 실패. 로그: $SAFETY_LOG"; }
timeout 5 rostopic echo -n1 /pid_safety/status >/dev/null 2>&1 || {
  bash "$STOP" >/dev/null 2>&1 || true
  fail "Safety 상태를 받지 못했습니다. 로그: $SAFETY_LOG"
}
# An active process is insufficient: wait for a fresh valid policy before AUTO.
if [[ "$TEST_CLEAR" != true ]]; then
  if ! python3 - "$ROUTE" <<'PY'
import json, sys, time
import rospy
from std_msgs.msg import String
route = sys.argv[1]
rospy.init_node("pid_safety_preflight", anonymous=True, disable_signals=True)
deadline = time.monotonic() + 6.0
while time.monotonic() < deadline:
    try:
        msg = rospy.wait_for_message("/ff_pid/motion_policy", String, timeout=1.0)
        data = json.loads(msg.data)
        now = rospy.Time.now().to_sec()
        if (data.get("route") == route and data.get("state") in ("CLEAR", "SLOW")
                and abs(now - float(data["stamp"])) <= 0.5
                and 0.0 < float(data["speed_limit_mps"]) <= 0.8):
            sys.exit(0)
    except (rospy.ROSException, KeyError, TypeError, ValueError):
        pass
sys.exit(1)
PY
  then
    bash "$STOP" >/dev/null 2>&1 || true
    fail "Safety가 주행 가능한 최신 정책을 발행하지 않았습니다. 로그: $SAFETY_LOG"
  fi
fi

ARMED=1
safe_stop_on_exit() {
  local code=$?
  trap - EXIT INT TERM HUP
  if [[ "${ARMED:-0}" == 1 ]]; then
    printf '\n[INFO] 주행 터미널 종료: FF-PD 정지 요청\n' >&2
    bash "$STOP" || printf '[WARN] 자동 정지 확인 실패. 실제 휠체어 정지를 확인하세요.\n' >&2
  fi
  exit "$code"
}
trap safe_stop_on_exit EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

# Transition MANUAL -> AUTO, verify motor feedback, then issue the sole START service call.
timeout 6 rostopic pub -1 /mode_cmd std_msgs/Int16 'data: 65' >/dev/null 2>&1 || fail 'AUTO 모드 요청 실패'
python3 "$CHECK" wheel --mode AUTO --stopped || fail 'AUTO 전환 및 정차 상태 확인 실패'
reply="$(timeout 7 rosservice call /ff_pid/start 'data: true' 2>&1)" || fail "주행 시작 서비스 실패: $reply"
printf '%s\n' "$reply" | grep -Eq '^success: [Tt]rue' || fail "제어기가 시작을 거부했습니다: $reply"

# The START service returned success. The drive process MUST continue running.
emit_drive_event STARTED START_CONFIRMED "FF-PD START 서비스가 주행을 승인했습니다."
printf '\n[RUN] %s 경로 주행 시작\n' "$ROUTE"
printf '[INFO] 현재 위치부터 선택 경로를 추종합니다. 시작점 일치 조건은 없습니다.\n'
printf '[INFO] 별도 터미널에서: bash src/pid/scripts/pid_stop.sh\n'
if [[ "$TEST_CLEAR" == true ]]; then
  printf '[WARN] TEST CLEAR 모드: 장애물 및 마스크 자동 STOP 비활성화.\n'
else
  printf '[INFO] Safety CLEAR/SLOW/STOP 연결됨 (인식 타임아웃 1.0초).\n'
fi
printf '[INFO] 이 터미널의 Ctrl+C도 정지 요청을 보냅니다.\n'
# Only state transitions are printed now; detailed telemetry is reserved for later.
python3 "$CHECK" watch --route "$ROUTE"
