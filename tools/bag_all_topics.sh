#!/usr/bin/env bash
# Full-topic ROS bag (rosbag record -a) for every field bring-up.
#
#   bag_all_topics.sh start   step [7/7] of start_wheelchair_localization.sh
#   bag_all_topics.sh stop    the bring-up's own sweep and stop_stack.sh
#   bag_all_topics.sh status
#
# Output: ${ALL_TOPICS_BAG_DIR:-~/localization_trials}/all_topics_<stamp>_<n>.bag
# ALL_TOPICS_BAG=0 turns it off.
#
# WHY
# ---
# The DWA stack only ever bagged hand-picked topic lists (blackbox_*,
# hybrid_*, person_bypass_*): no raw /livox/lidar, no registered cloud, so a
# drive could not be replayed through perception or localization afterwards.
# The FF-PID READY stack has recorded -a since 2026-11-19
# (src/pid/scripts/pid_ready_bag.sh); this is the same recorder for DWA, with
# the same exclusions, split size and close procedure.
#
# CLOSING
# -------
# The sweeps elsewhere SIGTERM '[r]osbag record' and then SIGKILL survivors
# after a few seconds. A multi-GB bag that is still writing its index when the
# SIGKILL lands is left as .bag.active and needs a reindex (2026-11-19: the
# DWA blackbox/hybrid/person_bypass bags of that evening all were). So stop
# SIGINTs this recorder's own process group FIRST and waits for it to finish,
# and never escalates itself. The 15 min / 2 GB split bounds what a power cut
# (also 2026-11-19, 18:59) can leave unindexed to the last segment.
#
# Neither start nor stop may fail a bring-up or a teardown: every path out of
# here is exit 0.
set -u

OUT_DIR="${ALL_TOPICS_BAG_DIR:-$HOME/localization_trials}"
STATE_DIR="${ALL_TOPICS_BAG_STATE_DIR:-$HOME/.local/state/dwa_bag}"
PID_FILE="$STATE_DIR/all_topics.pid"
LOG_FILE="${ALL_TOPICS_BAG_LOG:-$HOME/live_all_topics.log}"
# Raw LiDAR + registered clouds run to roughly 3-5 GB per 10 min after lz4
# (tools/record_debug_bag.sh), hence the free-space floor and buffer.
MIN_FREE_MB="${ALL_TOPICS_BAG_MIN_FREE_MB:-10240}"
BUFFER_MB="${ALL_TOPICS_BAG_BUFFER_MB:-1024}"
SPLIT_MB="${ALL_TOPICS_BAG_SPLIT_MB:-2048}"
SPLIT_DURATION="${ALL_TOPICS_BAG_SPLIT_DURATION:-15m}"
CLOSE_TIMEOUT_S="${ALL_TOPICS_BAG_CLOSE_TIMEOUT_S:-30}"
# Same exclusions as tools/record_debug_bag.sh and pid_ready_bag.sh: the
# latched full-map preview and the live preview are large and reconstructible,
# markers are cosmetic, rosout is noise, compressed images are duplicates.
EXCLUDE="${ALL_TOPICS_BAG_EXCLUDE:-^/fast_lio_icp/(map_preview|live_preview|.*_marker)\$|^/rosout(_agg)?\$|/compressed|/theora}"

read_pid() {
  local pid
  [ -s "$PID_FILE" ] || return 1
  IFS= read -r pid < "$PID_FILE" || return 1
  case "$pid" in ''|*[!0-9]*) return 1 ;; esac
  printf '%s\n' "$pid"
}

# Alive AND still a rosbag recorder: a reused PID must never be signalled.
recorder_alive() {
  local cmd
  kill -0 "$1" 2>/dev/null || return 1
  cmd="$(tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null || true)"
  case "$cmd" in *rosbag*record*) return 0 ;; *) return 1 ;; esac
}

active_files() {
  ls "$OUT_DIR"/all_topics_*.bag.active 2>/dev/null || true
}

do_stop() {
  local pid pgid i left
  if ! pid="$(read_pid)"; then
    echo "all-topics bag: not running"
    return 0
  fi
  if ! recorder_alive "$pid"; then
    rm -f "$PID_FILE"
    echo "all-topics bag: not running (stale pid $pid)"
  else
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d '[:space:]')"
    if [ "$pgid" = "$pid" ]; then
      # start ran it under setsid, so the PID is its own process group: the
      # rosbag wrapper and the recorder child both get the SIGINT.
      kill -INT -- "-$pgid" 2>/dev/null || true
    else
      echo "all-topics bag: WARNING pgid '$pgid' != pid $pid; signalling the pid only" >&2
      kill -INT "$pid" 2>/dev/null || true
    fi
    echo "all-topics bag: closing (SIGINT to $pid, up to ${CLOSE_TIMEOUT_S}s)"
    for ((i = 0; i < CLOSE_TIMEOUT_S * 2; i++)); do
      recorder_alive "$pid" || break
      sleep 0.5
    done
    if recorder_alive "$pid"; then
      echo "all-topics bag: WARNING recorder $pid still running after ${CLOSE_TIMEOUT_S}s; not force-killing it" >&2
      return 0
    fi
    rm -f "$PID_FILE"
  fi
  left="$(active_files)"
  if [ -n "$left" ]; then
    echo "all-topics bag: WARNING unclosed bag(s) remain, reindex before use:" >&2
    echo "$left" | sed 's/^/  /' >&2
  else
    echo "all-topics bag: closed"
  fi
  return 0
}

do_start() {
  local pid free_mb stamp
  if [ "${ALL_TOPICS_BAG:-1}" = "0" ]; then
    echo "all-topics bag: off (ALL_TOPICS_BAG=0)"
    return 0
  fi
  if pid="$(read_pid)" && recorder_alive "$pid"; then
    echo "all-topics bag: already recording (pid $pid)"
    return 0
  fi
  if ! command -v rosbag >/dev/null 2>&1; then
    echo "all-topics bag: WARNING rosbag not on PATH, not recording" >&2
    return 0
  fi
  mkdir -p "$OUT_DIR" "$STATE_DIR" 2>/dev/null || true
  free_mb="$(df -Pm "$OUT_DIR" 2>/dev/null | awk 'NR==2 {print $4}')"
  case "$free_mb" in ''|*[!0-9]*) free_mb=0 ;; esac
  if [ "$free_mb" -lt "$MIN_FREE_MB" ]; then
    echo "all-topics bag: WARNING skipped, needs ${MIN_FREE_MB} MiB free in $OUT_DIR (free: ${free_mb} MiB)" >&2
    return 0
  fi
  stamp="$(date +%Y%m%d_%H%M%S)"
  # A background job of a non-interactive shell starts with SIGINT ignored,
  # and an ignored signal stays ignored across exec. Restore the default
  # before exec'ing rosbag or stop's SIGINT would never reach the recorder.
  setsid nohup python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
    rosbag record -a --lz4 -b "$BUFFER_MB" \
    --split --size="$SPLIT_MB" --duration="$SPLIT_DURATION" \
    -x "$EXCLUDE" \
    -O "$OUT_DIR/all_topics_$stamp" \
    > "$LOG_FILE" 2>&1 < /dev/null &
  pid=$!
  printf '%s\n' "$pid" > "$PID_FILE"
  sleep 0.5
  if recorder_alive "$pid"; then
    echo "all-topics bag: recording -> $OUT_DIR/all_topics_${stamp}_*.bag (pid $pid)"
  else
    rm -f "$PID_FILE"
    echo "all-topics bag: WARNING recorder exited at once; see $LOG_FILE" >&2
  fi
  return 0
}

case "${1:-status}" in
  start) do_start ;;
  stop) do_stop ;;
  status)
    if pid="$(read_pid)" && recorder_alive "$pid"; then
      echo "all-topics bag: recording (pid $pid)"
    else
      echo "all-topics bag: not running"
    fi
    left="$(active_files)"
    [ -z "$left" ] || { echo "open/unclosed:"; echo "$left" | sed 's/^/  /'; }
    ;;
  *) echo "usage: $0 start|stop|status" >&2 ;;
esac
exit 0
