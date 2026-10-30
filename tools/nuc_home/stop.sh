#!/usr/bin/env bash
# [주행 정지] as the operator app reaches it, via the Bluetooth bridge.
#
# All three of these must name the SAME entry point. On 2026-08-28 16:40 they
# did not: this one pointed at the early-avoidance deploy while go.sh and
# stop.sh pointed at hybrid_smooth_success.sh, so [로컬 켜기] brought up one
# tree and [주행 시작] drove another. They now all follow
# ~/wheelchair_deploys/current, which is switched with ~/use_deploy.sh.
#
# Original: stop.sh.pre-early-avoidance  (bash ~/revert_early_avoidance.sh)
set -eo pipefail

# hybrid.sh and its helpers fall back to $HOME/stop.sh when BASE_START/BASE_GO/
# BASE_STOP are unset, and that fallback is this wrapper. A missing export
# would spawn this forever, on a path that ends at the wheels.
if [ -n "${WHEELCHAIR_ENTRY_ACTIVE:-}" ]; then
  echo "ERROR: stop.sh re-entered - BASE_START/BASE_GO/BASE_STOP are not" >&2
  echo "       reaching the deploy's tools/. Refusing rather than recursing." >&2
  exit 70
fi
export WHEELCHAIR_ENTRY_ACTIVE=1

exec bash "$HOME/hybrid_current.sh" stop "$@"
