#!/usr/bin/env bash
# Bring the chair up on the route2_intern map (2026-10-26 GLIM) and stop,
# armed but not driving. Nothing moves until go.sh.
#
# The start script refuses a partial map override, and the follower refuses a
# route, band and mask that are not hash-bound to each other, so the whole set
# is chosen here by route name instead of typed out at the chair.
#
#   ./start_route2_intern.sh            # D5_to_A
#   ./start_route2_intern.sh D5_to_A
#   LATENCY_S=0 ./start_route2_intern.sh D5_to_A
#   ./start_route2_intern.sh A_to_D5    # the same line from A, the other way
#   ./start_route2_intern.sh D4_to_D5   # any of A, D4, D5 to any other:
#                                       # D5_to_D4 D4_to_A A_to_D4 D4_to_D5
#   ./start_route2_intern.sh D5_to_A_raw  # the unsmoothed 10-28 line
#
# The route has never been driven autonomously. Guards stay on
# (SAFETY_POLICIES is not touched), the controller is DWA, and LATENCY_S is
# left to the caller: 0 against the 0.55 s default is still open from the
# 10-16 run. See docs/2026-10-28-route2-intern-map-ko.md.
set -eo pipefail

KEY="${1:-D5_to_A}"
# The checkout this script lives in, so a deploy starts on its own assets.
SRC="${SRC:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
# Exported, not prefixed onto the `.`: bash restores a prefix assignment once
# the builtin returns, and ~/hybrid_current.sh sources route2_intern_env.sh
# again further down - without the key in its environment it falls back to
# D5_to_A and brings that route up whatever was asked for here.
export ROUTE2_KEY="$KEY"
REPO="$SRC" . "$SRC/tools/route2_intern_env.sh" || exit $?
export PROFILE="${PROFILE:-dwa}"

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
