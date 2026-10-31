#!/usr/bin/env bash
# Read-only checks for a route2_intern DWA drive, on the NUC. Publishes
# nothing and calls no service, so it is safe to run at any time, chair
# armed or not. Exit status is the number of failed checks.
#
#   bash ~/wheelchair_deploys/current/source-linux/tools/route2_intern_check.sh pre
#       before bring-up: which deploy is current, that it is verified and
#       ships route2_intern, that the PCD hash matches, that the ~/ wrappers
#       are the ones in tools/nuc_home/
#   bash .../route2_intern_check.sh running [KEY]
#       after ~/start_route2_intern.sh [KEY], before ~/go.sh: that the
#       follower is the DWA one (not pure pursuit), on route KEY, and
#       localization is TRACKING
#
# KEY is the one given to start_route2_intern.sh (D4_to_D5, A_to_D5, ...).
# It has to be given again here: the key start_route2_intern.sh exports lives
# only in that process tree, and a fresh SSH shell would check against
# D5_to_A. Without KEY, `running` takes it from the follower's route and says
# so - that shows what is up, not that it is what was meant.
#
# go.sh repeats the motion-critical checks itself; this exists so a first-time
# operator can see what is up before anyone presses [주행 시작].
set -o pipefail

MODE="${1:-pre}"
KEY_ARG="${2:-}"
DEPLOYS="${DEPLOYS:-$HOME/wheelchair_deploys}"
FAILED=0
ok()   { echo "  OK    $*"; }
bad()  { echo "  FAIL  $*"; FAILED=$((FAILED + 1)); }
note() { echo "  ..    $*"; }

# The route file name -> its route2_intern_env.sh key, or nothing.
route_key_of() {
  local name
  name="$(basename "$1")"
  case "$name" in
    20261028_route2_D5_to_A_waypoints.json) echo D5_to_A_raw ;;
    20261030_route2_A_to_D5_waypoints.json) echo A_to_D5_raw ;;
    20261031_route2_*_waypoints.json)
      name="${name#20261031_route2_}"; echo "${name%_waypoints.json}" ;;
  esac
}

resolve_key() {
  echo "[route key]"
  if [ -n "$KEY_ARG" ]; then
    export ROUTE2_KEY="$KEY_ARG"
    note "$ROUTE2_KEY (argument)"
    return
  fi
  if [ -n "${ROUTE2_KEY:-}" ]; then
    note "$ROUTE2_KEY (ROUTE2_KEY from the environment)"
    return
  fi
  if [ "$MODE" = running ]; then
    source /opt/ros/noetic/setup.bash >/dev/null 2>&1
    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
    local key
    key="$(route_key_of "$(rosparam get /waypoint_follower/route 2>/dev/null)")"
    if [ -n "$key" ]; then
      export ROUTE2_KEY="$key"
      note "$ROUTE2_KEY (from the running follower - pass the key you started with to check it)"
      return
    fi
  fi
  note "D5_to_A (default - pass the key you started with, e.g. running D4_to_D5)"
}

deploy_checks() {
  echo "[deploy]"
  if [ ! -e "$DEPLOYS/current" ]; then
    bad "$DEPLOYS/current is not set - bash ~/use_deploy.sh <name>"
    return
  fi
  DEPLOY="$(cd -P "$DEPLOYS/current" && pwd)"
  REPO="$DEPLOY/source-linux"
  ok "current -> $(basename "$DEPLOY")"
  if [ -d "$REPO/.git" ]; then
    note "commit $(git -C "$REPO" log -1 --format='%h %s' 2>/dev/null)"
  fi
  [ -f "$DEPLOY/OFFLINE_VERIFIED" ] && ok "OFFLINE_VERIFIED present" \
    || bad "no OFFLINE_VERIFIED - start and go will refuse this deploy"

  # Without this file hybrid_current.sh silently drives the v9 route on the
  # merged map, which is a different campus path from the one at D5.
  if [ ! -f "$REPO/tools/route2_intern_env.sh" ]; then
    bad "deploy has no tools/route2_intern_env.sh - it would drive v9, not route2_intern"
    return
  fi
  if [ "${ROUTE_SET:-route2_intern}" != route2_intern ]; then
    bad "ROUTE_SET=$ROUTE_SET is exported - unset it for route2_intern"
  fi
  if ( REPO="$REPO" . "$REPO/tools/route2_intern_env.sh" ) >/dev/null 2>&1; then
    REPO="$REPO" . "$REPO/tools/route2_intern_env.sh"
    ok "route2_intern map hash matches ($MAP_ID)"
    ok "route $(basename "$ROUTE")"
    python3 - "$ROUTE" "${ROUTE2_KEY:-D5_to_A}" <<'EOF'
import json, sys
w = json.load(open(sys.argv[1]))["waypoints"]
start, end = sys.argv[2].split("_to_")
print("  ..    start  x=%.1f y=%.1f heading %.0f deg   (%s)"
      % (w[0]["x"], w[0]["y"], w[0]["yaw_deg"], start))
print("  ..    end    x=%.1f y=%.1f                 (%s)   %d waypoints"
      % (w[-1]["x"], w[-1]["y"], end, len(w)))
EOF
  else
    bad "route2_intern_env.sh refused:"
    ( REPO="$REPO" . "$REPO/tools/route2_intern_env.sh" ) 2>&1 | sed 's/^/          /'
  fi
}

wrapper_checks() {
  echo "[home wrappers]"
  local f
  for f in start_wheelchair_localization.sh start_route2_intern.sh go.sh stop.sh hybrid_current.sh; do
    if [ ! -f "$HOME/$f" ]; then
      bad "~/$f missing - install -m 755 $REPO/tools/nuc_home/$f ~/"
    elif cmp -s "$HOME/$f" "$REPO/tools/nuc_home/$f"; then
      ok "~/$f matches tools/nuc_home/"
    else
      bad "~/$f differs from tools/nuc_home/$f (diff them before driving)"
    fi
  done
}

running_checks() {
  echo "[running stack]"
  source /opt/ros/noetic/setup.bash >/dev/null 2>&1
  export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"

  if ! rosnode ping -c1 /waypoint_follower >/dev/null 2>&1; then
    bad "/waypoint_follower is not running - ~/start_route2_intern.sh first"
    return
  fi
  # The hybrid bring-up replaces the base follower twice: gpu_dwa_follower.py,
  # then person_bypass_dwa_follower.py under the same node name.
  if pgrep -f '[p]erson_bypass_dwa_follower.py' >/dev/null; then
    ok "follower is person_bypass_dwa_follower.py (DWA)"
  else
    bad "person_bypass_dwa_follower.py is not running"
  fi
  if pgrep -f '[w]aypoint_follower\.py' >/dev/null; then
    bad "waypoint_follower.py (pure pursuit) is running - stop and restart"
  fi

  local route latency
  route="$(rosparam get /waypoint_follower/route 2>/dev/null)"
  if [ -n "${ROUTE:-}" ] && [ "$route" = "$ROUTE" ]; then
    ok "follower route is $(basename "$route")"
  else
    bad "follower route is '${route:-unset}', expected '${ROUTE:-<pre checks failed>}'"
  fi
  latency="$(rosparam get /waypoint_follower/latency_s 2>/dev/null)"
  note "latency_s ${latency:-unset}"

  local state
  state="$(timeout 3 rostopic echo -n1 /fast_lio_icp/localization_diagnostics 2>/dev/null \
    | grep -m1 'message:' | sed 's/.*message: *//' | tr -d '"')"
  [ "$state" = TRACKING ] && ok "localization TRACKING" \
    || bad "localization is '${state:-silent}' - wait, or re-seat at D5 and restart"

  timeout 3 rostopic echo -n1 /perception/objects_summary >/dev/null 2>&1 \
    && ok "object tracking publishing" || bad "/perception/objects_summary is silent"

  local status
  status="$(timeout 3 rostopic echo -n1 /waypoint_follower/status 2>/dev/null \
    | grep -m1 'data:' | sed 's/.*data: *//')"
  note "follower status ${status:-silent} (PAUSED is expected before go.sh)"
}

case "$MODE" in
  pre)     resolve_key; deploy_checks; [ -n "${REPO:-}" ] && wrapper_checks ;;
  running) resolve_key; deploy_checks; running_checks ;;
  *) echo "usage: $0 pre|running [KEY]" >&2; exit 64 ;;
esac

echo ""
if [ "$FAILED" -eq 0 ]; then
  echo "ALL CHECKS PASSED ($MODE)"
else
  echo "$FAILED CHECK(S) FAILED ($MODE) - do not drive"
fi
exit "$FAILED"
