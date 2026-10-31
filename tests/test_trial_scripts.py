"""What gets typed at the chair.

Three scripts, and each one's failure mode is different. trial_0727 has to
configure the run it claims to configure - a trial that quietly brings the
guards up measures nothing, because a band refusal and a lost fix are the
same stationary chair. go has to refuse rather than start, since anything it
cannot check is something that gets discovered while the chair is already
moving. stop has to work when everything else has failed, which means
checking nothing at all.
"""

from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
TOOLS = ROOT / "tools"
ROUTE2_KEYS = ("D5_to_A", "A_to_D5", "D5_to_D4", "D4_to_D5", "D4_to_A",
               "A_to_D4")


def script(name):
    return (TOOLS / name).read_text(encoding="utf-8")


def test_the_trial_switches_the_discretionary_guards_off():
    """That is the whole point of it: measure one thing without another
    guard ending the measurement first."""
    assert "export SAFETY_POLICIES=false" in script("trial_0727.sh")


def test_the_trial_leaves_object_avoidance_on():
    """~cluster_avoidance is deliberately not behind ~safety_policies, so the
    trial must not set it - and the bringup must not either."""
    assert "cluster_avoidance" not in script("trial_0727.sh")
    assert "cluster_avoidance" not in script("start_wheelchair_localization.sh")


def test_the_trial_does_not_drive():
    trial = script("trial_0727.sh")
    assert "waypoint_follower/start" not in trial
    assert "mode_cmd" not in trial


def test_go_refuses_when_the_follower_is_not_there():
    go = script("go.sh")
    assert "rosnode ping -c1 /waypoint_follower" in go
    assert "fail " in go


def test_go_refuses_on_a_silent_object_tracker():
    """It is the only guard still watching for people once the policies are
    off, and silence from it reads exactly like clear road."""
    go = script("go.sh")
    assert "/perception/objects_summary" in go


def test_go_refuses_unless_localization_is_tracking():
    go = script("go.sh")
    assert "localization_diagnostics" in go
    assert 'TRACKING' in go


def test_go_checks_everything_before_it_commands_anything():
    """A check after the first command is a check performed while the chair
    is already moving."""
    go = script("go.sh")
    last_check = max(go.index("/perception/objects_summary"),
                     go.index("localization_diagnostics"),
                     go.index("rosnode ping"))
    assert last_check < go.index("rostopic pub -1 /mode_cmd")
    assert last_check < go.index("waypoint_follower/start")


def test_stop_checks_nothing_and_tolerates_failure():
    """A stop that refuses because a precondition failed is not a stop."""
    stop = script("stop.sh")
    assert "set -e" not in stop
    assert stop.count("|| true") >= 2
    assert 'data: false' in stop


@pytest.mark.parametrize(
    "name",
    ["trial_0727.sh", "go.sh", "stop.sh", "start_route2_intern.sh", "mark.py"],
)
def test_each_script_is_deployed_and_checksum_verified(name):
    """They live in $HOME, outside the checkout, so a pull never touches
    them - the same way the bringup was found still running the previous
    week's route."""
    push = script("push_to_nuc.sh")
    assert name in push
    assert "did not install cleanly" in push


@pytest.mark.parametrize(
    "name",
    ["trial_0727.sh", "go.sh", "stop.sh", "start_route2_intern.sh", "mark.py"],
)
def test_each_script_is_executable(name):
    assert (TOOLS / name).stat().st_mode & 0o111, "%s is not executable" % name


def test_route2_start_overrides_the_whole_map_identity_and_does_not_drive():
    """The start script refuses a partial map override; a route2 bringup that
    left one variable on the v9 default would localize against the wrong map."""
    start = script("start_route2_intern.sh")
    env = script("route2_intern_env.sh")
    # The NUC's hybrid_current.sh sources the same file, so the app and this
    # wrapper cannot pick different halves of the set.
    assert '. "$SRC/tools/route2_intern_env.sh"' in start
    assert '. "$REPO/tools/route2_intern_env.sh"' in script("nuc_home/hybrid_current.sh")
    for var in ("MAP", "MAP_SHA256", "MAP_ID", "TRAJ", "TRAJ_SHA256",
                "ROUTE", "AUTO_INIT_ROUTE", "BAND", "DRIVABLE_MASK"):
        assert "export %s=" % var in env
    for text in (start.split("set -eo pipefail", 1)[1], env):
        assert "SAFETY_POLICIES" not in text
        assert "waypoint_follower/start" not in text
        assert "mode_cmd" not in text
    assert "git lfs pull" in env


def test_route2_start_points_at_assets_that_exist():
    for rel in ("maps/route2_intern_20261026/traj_lidar.txt",
                "routes/20261028_route2_D5_to_A_waypoints.json",
                "routes/20261028_route2_D5_to_A_safety_band.json",
                "routes/20261030_route2_A_to_D5_waypoints.json",
                "routes/20261030_route2_A_to_D5_safety_band.json",
                "routes/route_2d_map_route2_D5_to_A.yaml",
                "routes/route_2d_map_route2_intern_20261031.yaml",
                "routes/route_2d_map_route2_intern_20261031.pgm"):
        assert (ROOT / rel).is_file(), rel
    for key in ROUTE2_KEYS:
        for kind in ("waypoints", "safety_band"):
            for rel in ("routes/20261031_route2_%s_%s.json" % (key, kind),
                        "routes/route2_manual_20261031/%s_%s.json"
                        % (key, kind)):
                assert (ROOT / rel).is_file(), rel
    for ext in ("yaml", "pgm"):
        rel = "routes/route2_manual_20261031/common_drivable_mask." + ext
        assert (ROOT / rel).is_file(), rel


def test_route2_start_exports_the_key_to_the_bringup_it_execs():
    # hybrid_current.sh re-sources route2_intern_env.sh; a key that only
    # prefixed the `.` here is gone by then and D5_to_A comes up instead.
    start = script("start_route2_intern.sh")
    assert 'export ROUTE2_KEY="$KEY"' in start
    assert 'ROUTE2_KEY="$KEY" .' not in start


def test_route2_start_asks_from_and_to_only_at_a_terminal():
    start = script("start_route2_intern.sh").replace("\r\n", "\n")
    assert "STATIONS=(A D4 D5)" in start
    # `D4 D5` and `D4_to_D5` name the same route.
    assert 'KEY="${1^^}_to_${2^^}"' in start
    # The menu reads the terminal; with no terminal there is nobody to ask,
    # and the key falls back to D5_to_A as before.
    assert 'elif [ -t 0 ]; then\n  KEY="$(pick_key)" || exit 1\nelse\n' \
        '  KEY=D5_to_A\nfi' in start
    assert start.count("</dev/tty") >= 2
    # The last destination is only offered, after the key was accepted.
    body = start.split('. "$SRC/tools/route2_intern_env.sh" || exit $?', 1)
    assert 'echo "$DEST" > "$LAST_FILE"' in body[1]
    assert 'echo "$DEST" > "$LAST_FILE"' not in body[0]


def test_route2_check_measures_the_chair_against_the_route_origin():
    # Started at D4 on D5_to_A, the follower would join at D4 and drive D4->A.
    check = script("route2_intern_check.sh")
    running = check.split("running_checks() {", 1)[1]
    assert "/fast_lio_icp/pose/pose/pose/position" in running
    assert 'START_TOLERANCE_M="${START_TOLERANCE_M:-3.0}"' in check
    assert 'bad "chair is ${2:-?} m from the route origin' in running


def test_route2_check_takes_the_key_the_chair_was_started_with():
    # A fresh SSH shell has no ROUTE2_KEY, so `running` checked every
    # non-default key against D5_to_A and failed a correct bring-up.
    check = script("route2_intern_check.sh").replace("\r\n", "\n")
    assert 'KEY_ARG="${2:-}"' in check
    assert 'export ROUTE2_KEY="$KEY_ARG"' in check
    assert "running) resolve_key; deploy_checks; running_checks ;;" in check
    # Without it, the key comes from the follower's route, for every route
    # file route2_intern_env.sh can hand out.
    assert "rosparam get /waypoint_follower/route" in \
        check.split("resolve_key() {", 1)[1]
    assert "*/route2_manual_20261031/*_waypoints.json)" in check
    assert 'echo "${name%_waypoints.json}_smooth"' in check
    assert "20261028_route2_D5_to_A_waypoints.json) echo D5_to_A_raw" in check
    assert "20261030_route2_A_to_D5_waypoints.json) echo A_to_D5_raw" in check


def test_route2_keys_name_both_directions_on_one_mask():
    env = script("route2_intern_env.sh").replace("\r\n", "\n")
    # The six A/D4/D5 keys are the manual routes on their common mask...
    assert "    %s)\n" % "|".join(ROUTE2_KEYS) in env
    assert 'stem="route2_manual_20261031/$key"; ' \
        'mask="route2_manual_20261031/common_drivable_mask.yaml"' in env
    # ...the smoothed line is the same six with _smooth, on its own mask...
    assert "    %s)\n" % "|".join(k + "_smooth" for k in ROUTE2_KEYS) in env
    assert 'stem="20261031_route2_${key%_smooth}"; ' \
        'mask="route_2d_map_route2_intern_20261031.yaml"' in env
    # ...and the unsmoothed 10-28 pair stays on its own 10-28 mask.
    assert 'stem="20261028_route2_D5_to_A"' in env
    assert 'stem="20261030_route2_A_to_D5"' in env
    assert env.count('mask="route_2d_map_route2_D5_to_A.yaml"') == 2
    known = env.split("known:", 1)[1].split(")", 1)[0]
    for key in ROUTE2_KEYS + ("<key>_smooth", "D5_to_A_raw", "A_to_D5_raw"):
        assert key in known


def test_field_marks_reach_the_black_box():
    assert "/field_mark" in script("start_wheelchair_localization.sh")
    assert "get_num_connections" in script("mark.py")
