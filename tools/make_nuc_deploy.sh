#!/usr/bin/env bash
# Build a NUC deploy under ~/wheelchair_deploys/<name> from a commit on
# GitHub, in the layout ~/hybrid_current.sh launches: source-linux (the
# checkout), ws (catkin workspace over its static_livox_localization), and the
# shared CUDA-PointPillars / python / tensorrt-lib of an earlier deploy.
#
#   bash tools/make_nuc_deploy.sh <commit> [name]
#
# Runs on the NUC. It never switches the app over: when it prints DEPLOY_READY,
# check the result and then `bash ~/use_deploy.sh <name>`.
set -eo pipefail

COMMIT="${1:?usage: make_nuc_deploy.sh <commit> [name]}"
NAME="${2:-main-${COMMIT:0:7}-$(date +%Y%m%d)}"
URL="${URL:-https://github.com/Dannythechampion/UNICON_wheelchair_v2}"
DEPLOYS="$HOME/wheelchair_deploys"
SHARED="${SHARED:-$DEPLOYS/main-b2c36ae-20260827}"
EXTEND="${EXTEND:-$HOME/livox_static_localization_ws/devel}"
D="$DEPLOYS/$NAME"

[ ! -e "$D" ] || { echo "ERROR: $D already exists" >&2; exit 1; }
mkdir -p "$D"

echo "== checkout $COMMIT"
# LFS also holds the black-box bags; only the maps are needed on the chair.
GIT_LFS_SKIP_SMUDGE=1 git clone -q "$URL" "$D/source-linux"
cd "$D/source-linux"
git checkout -q --detach "$COMMIT"
git lfs pull --include="maps/**"
FULL="$(git rev-parse HEAD)"
echo "   $FULL $(git log -1 --format=%s)"

echo "== route2_intern assets"
REPO="$D/source-linux" . tools/route2_intern_env.sh
python3 - "$ROUTE" "$BAND" "$DRIVABLE_MASK" <<'EOF'
import json, sys
w = json.load(open(sys.argv[1]))
b = json.load(open(sys.argv[2]))
print("   route %d waypoints, band %d stations, mask %s"
      % (len(w["waypoints"]), len(b["stations"]), sys.argv[3].rsplit("/", 1)[-1]))
EOF

echo "== shared runtime from $(basename "$SHARED")"
for l in CUDA-PointPillars python tensorrt-lib; do
  ln -s "$(readlink -f "$SHARED/$l")" "$D/$l"
done

echo "== build"
mkdir -p "$D/ws/src" "$D/logs"
ln -s "$D/source-linux/src/static_livox_localization" "$D/ws/src/static_livox_localization"
# ROS's setup scripts read unset variables.
source /opt/ros/noetic/setup.bash
cd "$D/ws"
catkin config --extend "$EXTEND" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_FAST_GICP_CUDA=ON >/dev/null
catkin build static_livox_localization --no-status 2>&1 | tail -3

echo "== package tests"
cd "$D/source-linux/src/static_livox_localization"
TESTS="$(python3 -m pytest -q -p no:cacheprovider test 2>&1 | tail -1)" || true
echo "   $TESTS"
case "$TESTS" in
  *failed*|*error*) echo "ERROR: package tests did not pass; not marking verified" >&2; exit 1 ;;
esac

cat > "$D/OFFLINE_VERIFIED" <<EOF
commit=$FULL
source=$URL
map=$MAP_ID route=$(basename "$ROUTE")
build=catkin build static_livox_localization ENABLE_FAST_GICP_CUDA=ON succeeded
nuc_tests=$TESTS
prepared_at=$(date '+%Y-%m-%d %H:%M %Z')
EOF
echo "DEPLOY_READY $NAME  (switch: bash ~/use_deploy.sh $NAME)"
