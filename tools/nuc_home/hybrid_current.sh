#!/usr/bin/env bash
# Entry point the operator app reaches through the Bluetooth bridge.
#
# It follows ~/wheelchair_deploys/current rather than naming a deploy, so
# whatever is selected on the NUC is what [로컬 켜기] and [주행 시작] launch.
# Switch with:  bash ~/use_deploy.sh <deploy-name>
set -eo pipefail

DEPLOYS=/home/mprp3/wheelchair_deploys
if [ ! -e "$DEPLOYS/current" ]; then
  echo "ERROR: $DEPLOYS/current is not set. Run: bash ~/use_deploy.sh <name>" >&2
  exit 78
fi
DEPLOY="$(cd -P "$DEPLOYS/current" && pwd)"
REPO="$DEPLOY/source-linux"
[ -d "$REPO" ] || { echo "ERROR: $REPO missing" >&2; exit 78; }

export LOCALIZATION_WS="$DEPLOY/ws"
export BASE_START="$REPO/tools/start_wheelchair_localization.sh"
export BASE_GO="$REPO/tools/go.sh" BASE_STOP="$REPO/tools/stop.sh"
# Route set. A deploy that ships tools/route2_intern_env.sh drives the
# route2_intern map (GLIM 2026-10-26) and its D5->D4->A route, map and route
# together; ROUTE2_KEY picks the route on that map. Older deploys, or
# ROUTE_SET=v9, keep the 08-16 v9 route on the merged 0707+0725 map.
if [ "${ROUTE_SET:-route2_intern}" = route2_intern ] && [ -f "$REPO/tools/route2_intern_env.sh" ]; then
  # A missing or wrong PCD refuses start and go, never stop.
  if ! REPO="$REPO" . "$REPO/tools/route2_intern_env.sh"; then
    case "${1:-}" in start|go) exit 66 ;; esac
  fi
else
  export ROUTE="$REPO/routes/20260816_route_v9_clearance_waypoints.json"
  export BAND="$REPO/routes/20260816_route_v9_clearance_safety_band.json"
  export DRIVABLE_MASK="$REPO/routes/route_2d_map_v9.yaml"
  export AUTO_INIT_ROUTE="$ROUTE"
fi
export AUTO_INIT_GLOBAL_ONLY=true
export RVIZ=true
export PYTHONNOUSERSITE=1
export PYTHONPATH="$DEPLOY/python${PYTHONPATH:+:$PYTHONPATH}"
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH="/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# Nothing here exports START_POINTPILLARS. The profile decides, and both the
# bring-up and the drive read it from tools/perception_profile.sh - exporting
# it once refused the drive on a stack that had started without the detector.
export PERCEPTION_PROFILE="${PERCEPTION_PROFILE:-legacy_geometric}"
export PREFER_DWA_GPU=true REQUIRE_GPU=true
export CLIFF_REQUIRED=false SAFETY_POLICIES=true
export LOG="$DEPLOY/logs"
mkdir -p "$LOG"

case "${1:-}" in
  start|go)
    if [ ! -f "$DEPLOY/OFFLINE_VERIFIED" ]; then
      echo "NOT READY: $(basename "$DEPLOY") has no OFFLINE_VERIFIED" >&2
      exit 2
    fi
    ;;
esac
echo "[deploy] $(basename "$DEPLOY")  profile=$PERCEPTION_PROFILE  map=${MAP_ID:-<start default>}  route=$(basename "$ROUTE")" >&2
exec bash "$REPO/tools/hybrid.sh" "$@"
