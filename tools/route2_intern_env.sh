# Source this, do not run it. Sets the route2_intern map and one of its
# hash-bound route sets for start_wheelchair_localization.sh, go.sh and the
# bridge, so the NUC entry points and start_route2_intern.sh pick the same
# files.
#
#   REPO=<checkout> ROUTE2_KEY=D5_to_A . tools/route2_intern_env.sh
#
# REPO defaults to the checkout this file lives in. ROUTE2_KEY defaults to
# D5_to_A. The six A/D4/D5 keys are the 2026-10-31 manual routes
# (routes/route2_manual_20261031/, picked by hand, one common mask):
#
#   D5_to_A  A_to_D5  D5_to_D4  D4_to_D5  D4_to_A  A_to_D4
#
# The same six with _smooth (D4_to_D5_smooth, ...) are the 10-28 line
# smoothed and cut at D4 (tools/build_route2_intern_segments.py), on its own
# mask. D5_to_A_raw and A_to_D5_raw are the unsmoothed 10-28 line on the
# 10-28 mask, as driven on 10-30, kept for comparison. Returns non-zero, with
# the variables untouched, when the key is unknown or the PCD is missing,
# still a Git LFS pointer, or does not match its SHA-256.

route2_intern_env() {
  local repo="${REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
  local key="${ROUTE2_KEY:-D5_to_A}"
  local stem mask
  case "$key" in
    D5_to_A|A_to_D5|D5_to_D4|D4_to_D5|D4_to_A|A_to_D4)
      stem="route2_manual_20261031/$key"; mask="route2_manual_20261031/common_drivable_mask.yaml" ;;
    D5_to_A_smooth|A_to_D5_smooth|D5_to_D4_smooth|D4_to_D5_smooth|D4_to_A_smooth|A_to_D4_smooth)
      stem="20261031_route2_${key%_smooth}"; mask="route_2d_map_route2_intern_20261031.yaml" ;;
    D5_to_A_raw)
      stem="20261028_route2_D5_to_A"; mask="route_2d_map_route2_D5_to_A.yaml" ;;
    A_to_D5_raw)
      stem="20261030_route2_A_to_D5"; mask="route_2d_map_route2_D5_to_A.yaml" ;;
    *)
      echo "ERROR: unknown route2_intern route '$key' (known: D5_to_A, A_to_D5, D5_to_D4, D4_to_D5, D4_to_A, A_to_D4, each also as <key>_smooth, D5_to_A_raw, A_to_D5_raw)" >&2
      return 64 ;;
  esac

  local map_dir="$repo/maps/route2_intern_20261026"
  local map="$map_dir/route2_intern.pcd"
  local map_sha256="e0809f29a7c185b7e40b3cb1889e558fa5aa0bec51834023681d19cb77f74432"

  # The PCD travels by Git LFS. A checkout without `git lfs pull` holds a
  # 130-byte pointer under the same name, and the localizer would only find
  # out after the whole stack is up.
  [ -f "$map" ] || { echo "ERROR: $map missing" >&2; return 66; }
  if head -c 64 "$map" | grep -q 'git-lfs'; then
    echo "ERROR: $map is a Git LFS pointer - run 'git lfs pull' in $repo" >&2
    return 66
  fi
  local actual
  actual="$(sha256sum "$map" | awk '{print $1}')"
  if [ "$actual" != "$map_sha256" ]; then
    echo "ERROR: $map SHA-256 mismatch" >&2
    echo "  expected: $map_sha256" >&2
    echo "  actual  : $actual" >&2
    return 65
  fi

  export MAP="$map"
  export MAP_SHA256="$map_sha256"
  export MAP_ID="route2_intern_20261026"
  export TRAJ="$map_dir/traj_lidar.txt"
  export TRAJ_SHA256="0a4b35d9039343293afd74e2522b5b8dd5ac36ab124d34dfe9aa62d41e508ef5"
  export ROUTE="$repo/routes/${stem}_waypoints.json"
  export AUTO_INIT_ROUTE="$ROUTE"
  export BAND="$repo/routes/${stem}_safety_band.json"
  export DRIVABLE_MASK="$repo/routes/$mask"
  export ROUTE2_KEY="$key"
}

route2_intern_env
