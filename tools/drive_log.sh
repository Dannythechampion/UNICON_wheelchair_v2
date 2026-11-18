#!/usr/bin/env bash
# The read-only drive logger (tools/drive_log/drive_logger.py), one per drive.
#
#   drive_log.sh start    go.sh calls this before it commands anything
#   drive_log.sh stop     stop.sh calls this after the wheels are stopped
#   drive_log.sh status
#
# Output: ${DRIVE_LOG_ROOT:-~/drive_logs}/<YYYYmmdd_HHMMSS>/ with drive.csv,
# objects.jsonl, smith.jsonl and events.log. DRIVE_LOG=0 turns it off.
#
# It subscribes and writes files, nothing else, so neither start nor stop may
# ever fail the drive: every path out of here is exit 0. A logger left over
# from the hand-started ~/drive_logger.py counts as running, so the two never
# write side by side.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOGGER="$HERE/drive_log/drive_logger.py"
ROOT="${DRIVE_LOG_ROOT:-$HOME/drive_logs}"
PATTERN='[d]rive_logger.py'

latest() { ls -dt "$ROOT"/*/ 2>/dev/null | head -1; }
running() { pgrep -f "$PATTERN" >/dev/null 2>&1; }

case "${1:-status}" in
  start)
    if [ "${DRIVE_LOG:-1}" = "0" ]; then
      echo "drive log: off (DRIVE_LOG=0)"
      exit 0
    fi
    if running; then
      echo "drive log: already running -> $(latest)"
      exit 0
    fi
    if [ ! -f "$LOGGER" ]; then
      echo "drive log: WARNING $LOGGER missing, not logging" >&2
      exit 0
    fi
    mkdir -p "$ROOT" 2>/dev/null || true
    before="$(latest)"
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash >/dev/null 2>&1 || true
    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
    setsid nohup python3 "$LOGGER" > "$ROOT/drive_logger.out" 2>&1 < /dev/null &
    for _ in $(seq 1 30); do
      now="$(latest)"
      [ -n "$now" ] && [ "$now" != "$before" ] && break
      sleep 0.1
    done
    if running; then
      echo "drive log: $(latest)"
    else
      echo "drive log: WARNING logger exited at start, see $ROOT/drive_logger.out" >&2
    fi
    ;;
  stop)
    if ! running; then
      echo "drive log: not running"
      exit 0
    fi
    pkill -INT -f "$PATTERN" 2>/dev/null || true
    for _ in $(seq 1 30); do
      running || break
      sleep 0.1
    done
    running && { pkill -TERM -f "$PATTERN" 2>/dev/null || true; }
    echo "drive log: saved $(latest)"
    ;;
  status)
    if running; then
      echo "drive log: running -> $(latest)"
    else
      echo "drive log: not running (last $(latest))"
    fi
    ;;
  *)
    echo "usage: $0 start|stop|status" >&2
    ;;
esac
exit 0
