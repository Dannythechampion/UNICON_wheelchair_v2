#!/usr/bin/env bash
# Bring the chair up on the route2_intern map (2026-10-26 GLIM) and stop,
# armed but not driving. Nothing moves until go.sh.
#
# The start script refuses a partial map override, and the follower refuses a
# route, band and mask that are not hash-bound to each other, so the whole set
# is chosen here by route name instead of typed out at the chair.
#
#   ./start_route2_intern.sh D5_to_A
#   LATENCY_S=0 ./start_route2_intern.sh D5_to_A
#
# The route has never been driven autonomously. Guards stay on
# (SAFETY_POLICIES is not touched), the controller is DWA, and LATENCY_S is
# left to the caller: 0 against the 0.55 s default is still open from the
# 10-16 run. See docs/2026-10-28-route2-intern-map-ko.md.
set -eo pipefail

KEY="${1:-}"
case "$KEY" in
  D5_to_A)
    STEM="20261028_route2_D5_to_A"; MASK="route_2d_map_route2_D5_to_A.yaml" ;;
  *)
    echo "usage: $0 D5_to_A" >&2
    exit 64 ;;
esac

SRC="${SRC:-$HOME/wheelchair_localization_src}"
MAP_DIR="$SRC/maps/route2_intern_20261026"
export MAP="$MAP_DIR/route2_intern.pcd"
export MAP_SHA256="e0809f29a7c185b7e40b3cb1889e558fa5aa0bec51834023681d19cb77f74432"
export MAP_ID="route2_intern_20261026"
export TRAJ="$MAP_DIR/traj_lidar.txt"
export TRAJ_SHA256="0a4b35d9039343293afd74e2522b5b8dd5ac36ab124d34dfe9aa62d41e508ef5"

# The PCD travels by Git LFS. A checkout without `git lfs pull` holds a
# 130-byte pointer under the same name, and the localizer would only find out
# after the whole stack is up.
[ -f "$MAP" ] || { echo "ERROR: $MAP missing" >&2; exit 66; }
if head -c 64 "$MAP" | grep -q 'git-lfs'; then
  echo "ERROR: $MAP is a Git LFS pointer - run 'git lfs pull' in $SRC" >&2
  exit 66
fi
ACTUAL_MAP_SHA256="$(sha256sum "$MAP" | awk '{print $1}')"
[ "$ACTUAL_MAP_SHA256" = "$MAP_SHA256" ] || {
  echo "ERROR: $MAP SHA-256 mismatch" >&2
  echo "  expected: $MAP_SHA256" >&2
  echo "  actual  : $ACTUAL_MAP_SHA256" >&2
  exit 65
}

export ROUTE="$SRC/routes/${STEM}_waypoints.json"
export AUTO_INIT_ROUTE="$ROUTE"
export BAND="$SRC/routes/${STEM}_safety_band.json"
export DRIVABLE_MASK="$SRC/routes/$MASK"
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
