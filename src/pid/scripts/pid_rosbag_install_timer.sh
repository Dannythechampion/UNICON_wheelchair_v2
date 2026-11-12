#!/usr/bin/env bash
# Install per-user timer for non-destructive ROS bag recovery.
# Run on NUC from the deployed src/pid/scripts directory.
set -Eeuo pipefail
HERE="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
ROS_SETUP="$(printenv ROS_SETUP || true)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/noetic/setup.bash}"
[[ -f "$HERE/pid_rosbag_auto.py" && -f "$HERE/pid_rosbag_organize.py" ]] || {
  echo "[FAIL] Missing organizer scripts in $HERE" >&2; exit 1;
}
[[ -r "$ROS_SETUP" ]] || { echo "[FAIL] ROS Noetic setup missing" >&2; exit 1; }
command -v systemctl >/dev/null || { echo "[FAIL] systemd unavailable" >&2; exit 1; }
mkdir -p "$HOME/.config/systemd/user"
SERVICE="$HOME/.config/systemd/user/pid-rosbag-auto.service"
TIMER="$HOME/.config/systemd/user/pid-rosbag-auto.timer"
python3 - "$SERVICE" "$TIMER" "$HERE" "$ROS_SETUP" <<'PY'
import pathlib, shlex, sys
service, timer, scripts, setup = map(pathlib.Path, sys.argv[1:])
command = "source %s; exec python3 %s" % (
    shlex.quote(str(setup)), shlex.quote(str(scripts / "pid_rosbag_auto.py")))
service.write_text(
    "[Unit]\nDescription=UNICON READY bag recovery and drive splitting\n"
    "[Service]\nType=oneshot\nExecStart=/bin/bash -lc %s\n"
    "TimeoutStartSec=1h\n" % shlex.quote(command))
timer.write_text(
    "[Unit]\nDescription=Retry unfinished UNICON READY bags\n"
    "[Timer]\nOnBootSec=3min\nOnUnitActiveSec=10min\nPersistent=true\n"
    "Unit=pid-rosbag-auto.service\n[Install]\nWantedBy=timers.target\n")
print("[OK] Installed user systemd units at", service.parent)
PY
systemctl --user daemon-reload
systemctl --user enable --now pid-rosbag-auto.timer
echo "[OK] Timer enabled. Original READY bags NEVER deleted."
echo "[INFO] Check: systemctl --user status pid-rosbag-auto.timer"
echo "[INFO] Logs: journalctl --user -u pid-rosbag-auto.service -n 80 --no-pager"
echo "[INFO] Optional unattended boot execution: sudo loginctl enable-linger \"$USER\""
