#!/usr/bin/env bash
# Bring the chair up on the route2_intern map (2026-10-26 GLIM) and stop,
# armed but not driving. Nothing moves until go.sh.
#
# The start script refuses a partial map override, and the follower refuses a
# route, band and mask that are not hash-bound to each other, so the whole set
# is chosen here by route name instead of typed out at the chair.
#
#   ./start_route2_intern.sh            # at a terminal: pick from and to
#                                       # by number; otherwise D5_to_A
#   ./start_route2_intern.sh D4 D5      # from D4 to D5
#   ./start_route2_intern.sh D4_to_D5   # the same, as a route key
#   LATENCY_S=0 ./start_route2_intern.sh D5 A
#   ./start_route2_intern.sh D4_to_D5_smooth  # the smoothed 10-28 line
#   ./start_route2_intern.sh D5_to_A_raw      # the unsmoothed 10-28 line
#
# The menu offers the destination of the last route brought up here as the
# origin (~/.route2_last_destination). That is where the chair should be if
# that drive was finished, not a measurement: route2_intern_check.sh running
# measures it.
#
# The route has never been driven autonomously. Guards stay on
# (SAFETY_POLICIES is not touched), the controller is DWA, and LATENCY_S is
# left to the caller: 0 against the 0.55 s default is still open from the
# 10-16 run. See docs/2026-10-28-route2-intern-map-ko.md.
set -eo pipefail

STATIONS=(A D4 D5)
LAST_FILE="${ROUTE2_LAST_FILE:-$HOME/.route2_last_destination}"

station_name() {
  case "$1" in
    A) echo "A  애지문" ;;
    D4) echo "D4 정몽구" ;;
    D5) echo "D5 제2공학관" ;;
    *) echo "$1" ;;
  esac
}

# menu TITLE DEFAULT STATION... -> the chosen station on stdout. Prompts go
# to stderr and answers come from the terminal, so $(menu ...) works.
menu() {
  local title="$1" default="$2"
  shift 2
  local items=("$@") i answer
  echo "$title" >&2
  for i in "${!items[@]}"; do
    if [ "${items[$i]}" = "$default" ]; then
      echo "  $((i + 1))) $(station_name "${items[$i]}")   [Enter]" >&2
    else
      echo "  $((i + 1))) $(station_name "${items[$i]}")" >&2
    fi
  done
  while :; do
    read -r -p "> " answer </dev/tty || return 1
    answer="${answer^^}"
    if [ -z "$answer" ] && [ -n "$default" ]; then
      echo "$default"; return
    fi
    if [[ "$answer" =~ ^[0-9]+$ ]] && [ "$answer" -ge 1 ] && [ "$answer" -le "${#items[@]}" ]; then
      echo "${items[$((answer - 1))]}"; return
    fi
    for i in "${items[@]}"; do
      [ "$answer" = "$i" ] && { echo "$i"; return; }
    done
    echo "  1-${#items[@]} 중 번호로 고르세요" >&2
  done
}

pick_key() {
  local last from to s others=()
  last="$(cat "$LAST_FILE" 2>/dev/null || true)"
  case "$last" in A|D4|D5) ;; *) last="" ;; esac
  echo "" >&2
  [ -n "$last" ] && echo "(지난번에 띄운 경로의 도착지: $(station_name "$last"))" >&2
  from="$(menu "출발지 - 휠체어가 지금 있는 곳" "$last" "${STATIONS[@]}")" || return 1
  for s in "${STATIONS[@]}"; do [ "$s" = "$from" ] || others+=("$s"); done
  echo "" >&2
  to="$(menu "목적지" "" "${others[@]}")" || return 1
  echo "" >&2
  read -r -p "$(station_name "$from") -> $(station_name "$to") (${from}_to_${to}) 로 띄웁니다. [Enter=예, n=취소] " s </dev/tty || return 1
  case "$s" in n|N|no|NO) echo "취소" >&2; return 1 ;; esac
  echo "${from}_to_${to}"
}

if [ "$#" -ge 2 ]; then
  KEY="${1^^}_to_${2^^}"
elif [ "$#" -eq 1 ]; then
  KEY="$1"
elif [ -t 0 ]; then
  KEY="$(pick_key)" || exit 1
else
  KEY=D5_to_A
fi
# The checkout this script lives in, so a deploy starts on its own assets.
SRC="${SRC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# Exported, not prefixed onto the `.`: bash restores a prefix assignment once
# the builtin returns, and ~/hybrid_current.sh sources route2_intern_env.sh
# again further down - without the key in its environment it falls back to
# D5_to_A and brings that route up whatever was asked for here.
export ROUTE2_KEY="$KEY"
REPO="$SRC" . "$SRC/tools/route2_intern_env.sh" || exit $?
export PROFILE="${PROFILE:-dwa}"
# D4_to_D5, D4_to_D5_smooth -> D5: the menu's guess at the next origin.
DEST="$(echo "$KEY" | cut -d_ -f3)"
case "$DEST" in A|D4|D5) echo "$DEST" > "$LAST_FILE" 2>/dev/null || true ;; esac

echo "=============================================================="
echo " route2_intern  $KEY"
echo "   map        $MAP_ID"
echo "   route      $ROUTE"
echo "   profile    $PROFILE   latency_s ${LATENCY_S:-<script default 0.55>}"
echo "   failsafe   the joystick - operator aboard, hand on it"
echo "=============================================================="
echo ""

cd "$HOME"
exec ./start_wheelchair_localization.sh
