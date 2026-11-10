"""Three-site startup contract tests.  No ROS, map PCD or wheel hardware."""
from pathlib import Path

ROOT = Path(__file__).parents[1]
REPO = ROOT.parents[1]


def test_automatic_three_site_mode_is_bound_to_route2_only():
    env = (REPO / "tools" / "route2_intern_env.sh").read_text()
    assert "AUTO_INIT_SITE_ROUTES" in env
    assert "AUTO_INIT_GLOBAL_ONLY=false" in env
    for segment in ("A_to_D4_waypoints.json", "D4_to_D5_waypoints.json",
                    "D5_to_D4_waypoints.json"):
        assert segment in env
    start = (REPO / "tools" / "start_wheelchair_localization.sh").read_text()
    assert 'AUTO_INIT_SITE_ROUTES="${AUTO_INIT_SITE_ROUTES:-}"' in start
    assert 'auto_init_site_routes:="$AUTO_INIT_SITE_ROUTES"' in start
    assert 'three-site initialization requires route2_intern map' in start


def test_multi_site_scores_and_compares_all_three_before_verifying():
    script = (ROOT / "scripts" / "auto_initial_pose.py").read_text()
    assert 'zip(("A", "D4", "D5"), site_paths)' in script
    assert 'prefer_gpu=False' in script
    assert 'site_decision = decide_fix(refined_sites' in script
    assert 'candidate.score < min_site_score' in script
    assert 'candidate.source == "known_site:D5"' in script
    assert script.index('site_decision = decide_fix') < script.index(
        'candidates = load_trajectory_candidates')
    assert 'publish_initialization_receipt(winner.source)' in script
    assert 'len(verified_sites) == 1' in script
    assert 'len(verified_sites) > 1' in script
    assert 'last_consensus' in script
    assert 'return 8' in script  # Unstable odometry cannot be silently waived.


def test_global_fallback_and_original_global_score_gates_remain_intact():
    script = (ROOT / "scripts" / "auto_initial_pose.py").read_text()
    assert 'decision = decide_fix(refined, args.min_refined_score)' in script
    assert 'None, (decision.candidate,), args.min_score, 1' in script
    assert 'site_decision.reason' in script
    assert 'tracking_was_verified(' in script
    assert 'wait_for_stationary_stability(' in script


def test_source_receipts_allow_only_configured_sites():
    start = (REPO / "tools" / "start_wheelchair_localization.sh").read_text()
    assert 'known_site:A|known_site:D4|known_site:D5' in start
    assert '[ -n "$AUTO_INIT_SITE_ROUTES" ]' in start
    launch = (ROOT / "launch" / "moving_localization.launch").read_text()
    assert '<param name="site_routes" value="$(arg auto_init_site_routes)"/>' in launch
    ready = (REPO / "src/pid/scripts/pid_ready.sh").read_text()
    assert 'auto_init_site_routes:="${AUTO_INIT_SITE_ROUTES:-}"' in ready
    assert 'known_site:A|known_site:D4|known_site:D5' in ready
