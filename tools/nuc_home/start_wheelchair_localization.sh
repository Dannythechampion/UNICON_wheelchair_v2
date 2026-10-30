#!/usr/bin/env bash
# [로컬라이제이션 스택 기동] as the operator app reaches it, via the bridge.
#
# All three home wrappers must name the same entry point; they follow
# ~/wheelchair_deploys/current, switched with ~/use_deploy.sh.
#
# Original: start_wheelchair_localization.sh.pre-early-avoidance
#           (bash ~/revert_early_avoidance.sh)
set -eo pipefail

# The bridge parses THIS file for the route when the stack is down, which is
# exactly when the app is open and the operator is about to press the button.
# ros1_bluetooth_bridge.py matches ^\s*ROUTE="\$\{ROUTE:-<path>\}" and falls
# back to a hard-coded 1897-point 20260814 algorithm route when it finds
# nothing - so the app drew that route while the chair was pinned to the
# 1917-point v9 clearance route. Replacing the original bring-up with a thin
# wrapper removed the line it reads; this puts it back, pointing at whatever
# deploy is current.
# route2_intern D5->D4->A since 2026-10-30; hybrid_current.sh sets the
# route that actually drives (v9 on deploys without route2_intern_env.sh).
ROUTE="${ROUTE:-/home/mprp3/wheelchair_deploys/current/source-linux/routes/20261028_route2_D5_to_A_waypoints.json}"
BAND="${BAND:-/home/mprp3/wheelchair_deploys/current/source-linux/routes/20261028_route2_D5_to_A_safety_band.json}"
export ROUTE BAND

# hybrid.sh and its helpers fall back to $HOME/start_wheelchair_localization.sh
# when BASE_START is unset, and that fallback is this wrapper.
if [ -n "${WHEELCHAIR_ENTRY_ACTIVE:-}" ]; then
  echo "ERROR: start_wheelchair_localization.sh re-entered - BASE_START is" >&2
  echo "       not reaching the deploy's tools/. Refusing rather than recursing." >&2
  exit 70
fi
export WHEELCHAIR_ENTRY_ACTIVE=1

exec bash "$HOME/hybrid_current.sh" start "$@"
