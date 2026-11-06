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
timeout 4 rosparam list >/dev/null 2>&1 || fail 'ROS Master에 연결할 수 없습니다. 물리적 정지를 사용하세요.'

node_alive() { timeout 3 rosnode ping -c1 /ff_pid >/dev/null 2>&1; }
request_manual() { timeout 6 rostopic pub -1 /mode_cmd std_msgs/Int16 'data: 77' >/dev/null 2>&1; }

if node_alive; then
  if timeout 7 rosservice call /ff_pid/start 'data: false' >/dev/null 2>&1; then
    msg '[INFO] FF-PD 제어기 정지 명령을 보냈습니다.'
  else
    msg '[WARN] FF-PD 일시정지 서비스에 응답이 없습니다. MANUAL 전환을 시도합니다.'
  fi
else
  msg '[INFO] 실행 중인 /ff_pid 노드가 없습니다.'
fi

# Wait for wheel feedback. Even if verification fails, try to request MANUAL.
if python3 "$CHECK" wheel --mode MANUAL --stopped --timeout 1.0 >/dev/null 2>&1; then
  msg '[OK] 이미 MANUAL 상태이며 바퀴가 정지했습니다.'
else
  if ! python3 "$CHECK" wheel --mode AUTO --stopped --timeout 8; then
    msg '[WARN] AUTO 상태에서 정지가 확인되지 않았습니다. MANUAL 전환을 시도합니다.'
  fi
  request_manual || fail 'MANUAL 모드 요청 실패. 물리적 조이스틱/긴급정지 사용 필요.'
fi

python3 "$CHECK" wheel --mode MANUAL --stopped --timeout 8 ||
  fail 'MANUAL + 바퀴 정지가 확인되지 않았습니다. FF-PD 노드는 중지 상태로 유지하며, 물리적 정지 확인이 필요합니다.'

if node_alive; then
  timeout 8 rosnode kill /ff_pid >/dev/null 2>&1 || fail '정차 상태는 확인됐지만 FF-PD 노드 종료에 실패했습니다.'
  for ((i=0;i<20;i++)); do
    node_alive || break
    sleep 0.2
  done
  node_alive && fail 'FF-PD 노드가 여전히 실행 중입니다.'
fi
msg '[OK] 자율주행 종료. READY 센서·GICP·ROS bag 녹화는 계속 유지됩니다.'
