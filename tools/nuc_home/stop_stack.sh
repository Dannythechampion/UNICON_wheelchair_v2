#!/usr/bin/env bash
# [스택 내리기] as the operator app reaches it, via the Bluetooth bridge.
#
# The bridge runs from ~/wheelchair_localization_src with --script-dir ~, so it
# looks for ~/stop_stack.sh. tools/stop_stack.sh reads its sweep list from the
# bring-up next to it; here that would be the home wrapper, which has no sweep
# loop. Point it at the current deploy's bring-up instead, and stop the drive
# through ~/stop.sh like every other path that ends at the wheels.
set -eo pipefail

DEPLOYS=/home/mprp3/wheelchair_deploys
if [ ! -e "$DEPLOYS/current" ]; then
  echo "ERROR: $DEPLOYS/current is not set. Run: bash ~/use_deploy.sh <name>" >&2
  exit 78
fi
REPO="$(cd -P "$DEPLOYS/current" && pwd)/source-linux"

export BASE_START="$REPO/tools/start_wheelchair_localization.sh"
export BASE_STOP="$HOME/stop.sh"

# Deploys made before tools/stop_stack.sh existed still have the same bring-up
# sweep loop, so the bridge checkout's copy can tear them down.
STACK_STOP="$REPO/tools/stop_stack.sh"
[ -f "$STACK_STOP" ] || STACK_STOP="$HOME/wheelchair_localization_src/tools/stop_stack.sh"
[ -f "$STACK_STOP" ] || { echo "ERROR: stop_stack.sh not found in $REPO/tools or ~/wheelchair_localization_src/tools" >&2; exit 66; }

exec bash "$STACK_STOP" "$@"
