#!/usr/bin/env python3
"""The DWA control law, put behind the pursuit follower's guards.

Third profile, same contract as the other two. It subclasses
WaypointFollower and replaces only the part that turns a pose into a Twist;
the hold ladder, geofence, band containment, localisation health, cluster
liveness failsafe, manual-mode override, the parked-or-moving decision and
stop-on-shutdown are inherited and run unmodified. Nothing safety-bearing is
restated here - a copied guard drifts from the original silently, and always
toward the follower that does not stop.

The parked-or-moving decision was on that list by intention and not in fact
until 2026-08-11. Replacing step() dropped it, because it lived in the body
of the method rather than beside the guards that were carefully extracted,
and no copy drifted - the profile simply never asked. The planner was handed
the nearest threat whatever the tracker said about it, so where the pursuit
profile stands and waits for someone walking, this one picked an arc round
them at OBSTACLE_FLOOR_M. It is WaypointFollower.avoidance_for now, asked by
every profile.

WHAT THIS ONE IS FOR
--------------------
Pursuit and the MPC both follow a line well and avoid things badly. The
avoidance in this stack is take_a_way_round: a fixed +-0.6 m lateral offset
applied to the pursuit target. On 2026-08-04 that offset, taken from a
standstill where the lookahead has collapsed to MIN_LOOKAHEAD_M 0.9, is a
demand for atan(0.6 / 0.9) = 34 degrees, and it steered the chair at a wall
at WP585, WP910 and WP924 in one evening. Rolling candidate velocities out
and scoring them cannot produce that: a candidate is a (v, w) the chair can
hold, and one whose rollout leaves the corridor is discarded rather than
commanded.

The corridor is enforced as a rollout critic, not a costmap layer, and
dwa_core's docstring carries the measurement behind that choice.

WHAT IT SHARES WITH THE MPC PROFILE
-----------------------------------
The command ramp (mpc_command.advance_command). Both profiles hit the same
wall on 2026-08-05: integrating the solver's acceleration onto the MEASURED
velocity cannot start a chair whose wheels ignore anything under 0.30 m/s.
The ramp integrates onto the last command over the time that actually
elapsed, and dwa_core's speed sampling keeps the deadband out of the
candidate set in the first place. Two defences, because the failure was
silent both times.

STATE
-----
Driven once, on 2026-08-08, and it failed: two runs covered 44 m and 48 m of
the 380 m route in 1009 s and 726 s. The planner's score had no heading term,
which made it a bang-bang regulator, and standing still was a candidate that
beat every moving arc whenever the chair sat on the line - 180 s stuck in one
run, 77 s in the other. Both are fixed in dwa_core, whose docstrings carry the
numbers. Not re-driven since. PROFILE still defaults to pursuit, which drove
this route twice on 2026-07-31 with a person in the chair.

Two further changes on 2026-08-11, neither of them driven either, and both
found by reading rather than by a run: the parked-or-moving decision above,
and the object's shape instead of its nearest point (obstacle_points). Each
is written against a defect visible in the code and in the recorded data,
and neither is field evidence.

`OFF_BAND` is still a stop only a person can clear - every rollout point is
tested, so a chair a centimetre outside the corridor has no admissible
candidate at all, and the 08-08 analysis measured that at 23 %. A bounded
recovery for it is written and tested but deliberately NOT merged: it is the
only change in this stack that turns a stop into motion, and it is waiting
on one run of the route to be worth its risk. That run is what this profile
needs before anything else.
"""

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import dwa_core
import mpc_speed
from actuation_lag import RESIDUAL_TAU_S, SmithPredictor
from cluster_guard import (GO_ROUND, PERSON_BYPASS,
                           PERSON_BYPASS_CLEARANCE_M,
                           PERSON_BYPASS_SPEED_MPS, WAIT,
                           corridor_obstacle_points)
from mpc_anchor import DEFAULT_GAIN, StateAnchor
from mpc_command import MAX_COMMAND_GAP_S, advance_command, jerk_limited
from route_mask import RouteMask
from straight_launch import LAUNCH_SPEED_MPS, StraightLaunch, launch_speed
from waypoint_follower import (WaypointFollower, CONTROL_HZ, CREEP_SPEED,
                               GUARD_SLOW_EXTRA_M, MAX_ACCEL, MAX_DECEL,
                               MAX_SPEED, MAX_YAW_RATE, PLAN_AHEAD_M)

# The solver returns a velocity target, not an acceleration, so the ramp is
# fed the acceleration that would close the gap in one period - clamped to
# what the base can do. Deceleration is allowed to be brisker than
# acceleration for the same reason the pursuit follower allows it: stopping
# sooner is never the unsafe direction.
YAW_SLEW_RPS2 = 1.5
# Low-speed steering limits (dwa_core.yaw_range, 2026-11-11). Both default
# on; ~curvature_max 0 and ~yaw_accel 0 restore the behaviour before them
# (no cap; candidates over the full +-MAX_YAW_RATE and the ramp at
# YAW_SLEW_RPS2).
#   B1 CURVATURE_MAX: |w| <= 0.9 * the speed commanded this cycle.
#   B2 YAW_ACCEL: candidates within last w +- 0.8 rad/s^2 * dt, and the ramp
#      slews at the same 0.8 - the base's own p90 yaw acceleration on 11-09
#      was 0.66-0.90 rad/s^2, the clean runs' command slew p90 0.50-0.73,
#      the evening's 1.00.
CURVATURE_MAX = 0.9
YAW_ACCEL_RPS2 = 0.8

# Smith model error (problem 3, 2026-11-11). The predictor assumes the yaw
# rate it commanded is the yaw rate the chair makes. Under the wheels'
# deadband it is not: on 11-09 |model - IMU| p90 was 0.27-0.36 rad/s below
# 0.3 m/s in every run (about 7 deg over the 0.41 s horizon), and at
# 0.3-0.5 m/s 0.25-0.33 in the failed evening runs against 0.12-0.18 in the
# afternoon; at cruise 0.10-0.15. So the carried error (measured - model,
# low-passed) is applied in full under RESIDUAL_FULL_BELOW_MPS, tapered to
# nothing at RESIDUAL_ZERO_ABOVE_MPS - cruise keeps the prediction that was
# measured to be better without it (11-02 replay) - and the yaw rate it is
# measured against is the IMU's, as actuation_lag recommends: /wheel_status
# reads yaw 10-15 % low in 0.05 rad/s counts. "wheel" uses the wheel report
# instead, "off" is the behaviour before this.
SMITH_RESIDUAL_MODES = ("off", "wheel", "imu")
SMITH_RESIDUAL_DEFAULT = "imu"
SMITH_RESIDUAL_TAU_ON_S = 0.3
SMITH_IMU_TOPIC = "/livox/imu"
SMITH_IMU_FRESH_S = 0.2
RESIDUAL_FULL_BELOW_MPS = 0.30
RESIDUAL_ZERO_ABOVE_MPS = 0.50


def residual_weight(speed_mps, full_below=RESIDUAL_FULL_BELOW_MPS,
                    zero_above=RESIDUAL_ZERO_ABOVE_MPS):
    """1 under full_below, 0 over zero_above, linear between."""
    speed = abs(float(speed_mps))
    if speed <= full_below:
        return 1.0
    if speed >= zero_above:
        return 0.0
    return (zero_above - speed) / (zero_above - full_below)

# A gate stop that lasts a few cycles leaves the chair still rolling - the
# base takes 0.55 s to answer and the chair coasts - but the accepted /cmd_vel
# reads zero. Ramping from that zero is what turned 0.2-1 s dropouts into a
# fall from 0.8 to 0.1-0.3 m/s on 2026-11-04, and a slow chair given a normal
# yaw rate pivots on its inner wheel: the S-weave. So a stop resumes from the
# wheels' own speed while they are still turning. /wheel_status runs at
# 100 Hz (116,295 frames over 1,163 s in the 10-30 blackbox bag), so 0.2 s is
# twenty frames: older than that is a base that has gone quiet, and then the
# accepted zero stands as before. Under 0.05 m/s is one count of the 0.1 km/h
# report away from rest, and is rest.
WHEEL_SPEED_FRESH_S = 0.2
ROLLING_SPEED_MPS = 0.05

# Actuation lag, measured 2026-08-11 (see led_state). Overridable with
# ~latency_s for a vehicle this has not been measured on; at 0.0 the lead is
# the identity and the profile behaves exactly as it did before.
#
# Back to 0.0 on 2026-08-15. The 0.55 was measured while mpc_speed.MAX_SPEED
# was still 0.6, and the lead it produces is a distance, not a time:
# speed x 0.55, so 0.33 m at the speed it was measured at and 0.55 m at the
# 1.0 the cap was raised to afterwards. The 08-15 drive steered in S-curves
# that the 08-12 drive at 0.6 did not, and led_state's own warning is that a
# lead longer than the real lag over-steers. Unmeasured at this speed is
# what it now is, so it takes the value the docstring prescribes for that.
# Re-measure at 1.0 m/s (command angular.z against yaw rate differentiated
# from /fast_lio_icp/pose) before putting a non-zero number back.
# Back on, and for the first time it will do something. led_state leads the
# pose by speed x lag, and until 2026-08-23 the speed in the state was always
# zero - /Odometry carries no twist and nothing decoded the wheel report - so
# every value of this constant was identical to nought. The 08-15 note that
# set it to 0.0 blamed a weave on a lead that was never applied.
#
# The measurement stands and has now been corroborated twice: 2026-08-11
# cross-correlated commanded angular.z against the yaw rate differentiated
# from /fast_lio_icp/pose and found the peak at 0.55 s with gain 1.03 and
# R^2 0.90, and the steady-state gain measured off /wheel_status on 08-23,
# over 15 samples above 0.7 m/s where the command was held 0.8 s or longer,
# comes out at 1.03 as well. The base turns as asked; it just takes 0.55 s
# to get there, and at 0.8 m/s that is 0.44 m of travel the planner was
# correcting for a place the chair had already left.
LATENCY_S = 0.55

# How the lag is covered. "lead" is led_state above: the measured arc,
# extended by LATENCY_S. "smith" is actuation_lag.SmithPredictor: the
# commands already sent, run through the base model measured on 2026-11-02.
# The lead projects a turn the planner has already reversed for another
# 0.6 s; that is the over-steer behind the 11-02 S-weave near A and the
# 56-degree overshoot at wp 935. "off" plans from the measured pose.
# smith is the default from 2026-11-04, on the 11-02 bag replay (heading
# error through turn reversals p50 1.2 deg against 3.1-4.7 for the lead),
# before it has been driven; ~latency_mode lead puts the old lead back.
LATENCY_MODES = ("lead", "smith", "off")
LATENCY_MODE = "smith"

# One JSON line per planning cycle with what the lag compensation was given
# and what it produced, so a bag can say whether the vehicle's late reaction
# (0.3-0.4 s behind the model on the 11-07 drive) is Smith not leading or the
# planner pose arriving stale. Read-only; nothing in the stack subscribes to
# it. An absolute name, because the follower runs under different node names
# in the bring-ups (waypoint_follower, or the script's own name).
SMITH_STATE_TOPIC = "/dwa_follower/smith_state"

# How wide a slice of each object the planner is shown, either side of the
# centreline. Wider than the follower's CORRIDOR_HALF_WIDTH 0.45, which is
# the width the parked/moving DECISION is taken over: a rollout is free to
# step aside anywhere the band allows, so the geometry it is scored against
# has to cover where it may go and not only where the route runs. Bounded
# because every extra slice is another point in an O(candidates x steps x
# points) batch inside a 0.1 s period.
OBSTACLE_HALF_WIDTH_M = 1.0

# The gate refusing something this follower cannot see.
#
# There are two obstacle sources on this chair and they do not see the same
# world. safety_gate works on raw returns inside a height band and has no
# idea what anything is; the follower and the planner work on classified
# clusters and cannot see what the producer did not cluster. When something
# lands only in the first - a bush leaning into the corridor, a thin post,
# clutter the producer files as outside_band - the follower reports a clear
# corridor and keeps commanding 0.80 m/s while the gate zeroes every frame
# of it. Nothing resolves: the follower never registers a threat, so
# blocked_since never starts, and the chair stands there reading DWA v=0.80
# until somebody walks over to it. That shape was on the wire at wp 1218 on
# 2026-08-23.
#
# What this does NOT do is route around it. The gate can say a distance and
# a side and nothing else, and a source that cannot say what it saw must
# not be allowed to say what to do about it - the raw-scan guard removed on
# 2026-08-05 taught that at the cost of three stopped runs and a chair
# steered at a wall. So the deadlock becomes a named, recorded hold with
# the gate's own numbers on it, and a person decides.
GATE_STALL_S = 1.5
# Only the obstacle-shaped vetoes. CLOUD_STALE, INPUT_STALE and the rest
# are their own faults with their own handling, and folding them in here
# would relabel a dead sensor as an object in the road.
GATE_OBSTACLE_REASONS = ("OBSTACLE", "OBSTACLE_SWEEP")


def gate_stall(reason, blocked_for_s, stall_s=GATE_STALL_S):
    """True when the gate has been refusing an obstacle we cannot see.

    Pure, and deliberately ignorant of what the follower thinks: the caller
    reaches it only after its own WAIT has declined to fire, so by then the
    absence of a threat of our own is established rather than re-tested.
    """
    if reason not in GATE_OBSTACLE_REASONS:
        return False
    if blocked_for_s is None:
        return False
    return float(blocked_for_s) >= float(stall_s)


def approach_cap(base_cap, distance_m, stop_m, floor_mps):
    """Speed for closing on something the chair may have to go round.

    The ramp exists so the chair arrives at a parked object already slow
    enough to steer past it, rather than at cruise. It used to ramp down to
    CREEP_SPEED, which is below the speed the loaded wheels turn at, and that
    turned the approach into a stop: dwa_core.speed_samples returns nothing
    executable under its floor, the planner has no candidate to score, and
    the manoeuvre the ramp was preparing for never gets attempted.

    Measured on 2026-08-23 against a parked motorcycle at wp 1437-1441: the
    ramp handed down 0.21, 0.25, 0.28 and finally 0.15 m/s, and the chair
    stopped 1.3 m short and sat there for 2.4 minutes with the object
    correctly tracked as static and crossing the band the whole time.

    So the ramp stops at the floor. Below it there is no such thing as a
    slower approach, only a stop, and a stop is what WAIT and the planner's
    own obstacle critic are for - both of which still hold: WAIT halts before
    this is reached, rollouts inside OBSTACLE_FLOOR_M are rejected, and the
    gate stops outright for anything inside the braking envelope.
    """
    if distance_m >= stop_m + GUARD_SLOW_EXTRA_M:
        return base_cap
    ratio = max(0.0, (distance_m - stop_m) / GUARD_SLOW_EXTRA_M)
    ramped = CREEP_SPEED + ratio * (MAX_SPEED - CREEP_SPEED)
    return min(base_cap, max(floor_mps, ramped))


class DwaFollower(WaypointFollower):
    CONTROL_LAW = "dwa"
    latency_mode = LATENCY_MODE

    def __init__(self):
        WaypointFollower.__init__(self)
        self.latency_s = float(rospy.get_param("~latency_s", LATENCY_S))
        self.latency_mode = str(rospy.get_param("~latency_mode",
                                                LATENCY_MODE))
        if self.latency_mode not in LATENCY_MODES:
            raise ValueError("~latency_mode must be one of %s, got %r"
                             % (", ".join(LATENCY_MODES), self.latency_mode))
        # The model-error term is off unless asked for (see actuation_lag).
        # The model-error term (actuation_lag), measured where the model is
        # wrong: low speed, against the IMU's yaw rate (see
        # SMITH_RESIDUAL_MODES).
        self.smith_residual = str(rospy.get_param(
            "~smith_residual", SMITH_RESIDUAL_DEFAULT))
        if self.smith_residual not in SMITH_RESIDUAL_MODES:
            raise ValueError("~smith_residual must be one of %s, got %r" % (
                ", ".join(SMITH_RESIDUAL_MODES), self.smith_residual))
        residual_tau = float(rospy.get_param(
            "~smith_residual_tau_s",
            SMITH_RESIDUAL_TAU_ON_S if self.smith_residual != "off"
            else RESIDUAL_TAU_S))
        if self.smith_residual == "off":
            residual_tau = 0.0
        self.smith = SmithPredictor(residual_tau_s=residual_tau)
        self.imu_gz = 0.0
        self.imu_stamp = None
        if self.smith_residual == "imu":
            # Imported here so the desk tests' message stubs need not know it.
            from sensor_msgs.msg import Imu
            rospy.Subscriber(SMITH_IMU_TOPIC, Imu, self.on_imu, queue_size=50)
        rospy.loginfo("DWA latency compensation: %s%s", self.latency_mode,
                      " %.2f s" % self.latency_s
                      if self.latency_mode == "lead" else
                      " horizon %.2f s, model error %s" % (
                          self.smith.horizon_s,
                          "low-passed %.2f s" % self.smith.residual_tau_s
                          if self.smith.residual_tau_s > 0.0 else "off")
                      if self.latency_mode == "smith" else "")
        self.smith_state_pub = rospy.Publisher(
            SMITH_STATE_TOPIC, String, queue_size=10)
        self.planner = dwa_core.DwaPlanner(
            self.band, self.waypoints,
            distance_m=float(rospy.get_param("~sim_distance_m",
                                              dwa_core.SIM_DISTANCE_M)),
            grace=float(rospy.get_param("~band_grace", 0.0)),
            route_mask=RouteMask(rospy.get_param("~drivable_mask")))
        self.planner.obstacle_extension = str(rospy.get_param(
            "~obstacle_extension", dwa_core.OBSTACLE_EXTENSION))
        if self.planner.obstacle_extension not in \
                dwa_core.OBSTACLE_EXTENSION_MODES:
            raise ValueError(
                "~obstacle_extension must be one of %s, got %r" % (
                    ", ".join(dwa_core.OBSTACLE_EXTENSION_MODES),
                    self.planner.obstacle_extension))
        self.anchor = StateAnchor(
            gain=float(rospy.get_param("~anchor_gain", DEFAULT_GAIN)))
        self.last_command_stamp = None
        self.odom_v = 0.0
        self.odom_w = 0.0
        self.dwa_status = ""
        # Carried across cycles so the ramp has a slope to be limited against.
        self.command_accel = 0.0
        # Launch from rest (straight_launch): ~launch_floor_mps 0.0 restores
        # the plain ramp, ~straight_launch false lets the planner steer from
        # the first cycle as before.
        self.launch_floor_mps = float(rospy.get_param(
            "~launch_floor_mps", LAUNCH_SPEED_MPS))
        self.launch = StraightLaunch(enabled=bool(rospy.get_param(
            "~straight_launch", False)))
        self.curvature_max = float(rospy.get_param(
            "~curvature_max", CURVATURE_MAX))
        self.yaw_accel = float(rospy.get_param(
            "~yaw_accel", YAW_ACCEL_RPS2))
        if not (0.0 <= self.curvature_max <= 5.0) or                 not (0.0 <= self.yaw_accel <= 5.0):
            raise ValueError("~curvature_max / ~yaw_accel out of range: "
                             "%r / %r" % (self.curvature_max, self.yaw_accel))
        rospy.loginfo("DWA steering limits: curvature %s, yaw accel %s",
                      "%.2f 1/m" % self.curvature_max
                      if self.curvature_max > 0.0 else "off",
                      "%.2f rad/s^2 (window + ramp)" % self.yaw_accel
                      if self.yaw_accel > 0.0 else
                      "off (ramp %.1f rad/s^2)" % YAW_SLEW_RPS2)
        rospy.loginfo("DWA launch: floor %.2f m/s, straight launch %s",
                      self.launch_floor_mps,
                      "on" if self.launch.enabled else "off")
        self.gate_reason = ""
        self.gate_blocked_since = None
        self.gate_detail = ""
        rospy.Subscriber("/safety_gate/status", String,
                         self.on_gate_status, queue_size=1)
        # Keep the planner's command state tied to what the downstream
        # safety stack actually released, not merely what DWA requested.
        # In the person-bypass graph the requested command is published on
        # /cmd_vel_planned while /cmd_vel is the final accepted command.
        self.accepted_cmd_topic = str(rospy.get_param(
            "~accepted_cmd_topic", "/cmd_vel"))
        rospy.Subscriber(self.accepted_cmd_topic, Twist,
                         self.on_accepted_command, queue_size=1)
        rospy.loginfo(
            "DWA profile: sim %.2f m, %d speeds x %d yaw rates, band and "
            "drivable mask as hard rejects, obstacle tail %s to %.1f m, "
            "threat corridor %s",
            self.planner.distance_m,
            len(dwa_core.speed_samples()), len(dwa_core.yaw_samples()),
            self.planner.obstacle_extension, dwa_core.OBSTACLE_PREVIEW_M,
            self.threat_corridor)

    def on_gate_status(self, message):
        """Track how long the gate has been refusing, and why."""
        try:
            report = json.loads(message.data)
        except (ValueError, TypeError):
            return
        reason = str(report.get("reason") or "")
        if reason != self.gate_reason:
            self.gate_reason = reason
            self.gate_blocked_since = rospy.Time.now() if reason else None
        nearest = report.get("zone_nearest_m")
        envelope = report.get("envelope_m")
        self.gate_detail = "%s at %s m, envelope %s m" % (
            reason or "clear",
            "?" if nearest is None else nearest,
            "?" if envelope is None else envelope)

    def gate_blocked_for(self, now):
        if self.gate_blocked_since is None:
            return None
        return (now - self.gate_blocked_since).to_sec()

    def on_accepted_command(self, message):
        """Resynchronise the DWA ramp with the command that reached base.

        A lower non-zero accepted command can simply be normal downstream
        lag, so retain the acceleration slope.  A final stop is different:
        carrying acceleration through it recreates the stop-go sawtooth on
        the next planning cycle, and must reset the ramp.
        """
        accepted_speed = float(message.linear.x)
        self.current_speed = accepted_speed
        self.last_yaw_rate = float(message.angular.z)
        if accepted_speed <= 0.02:
            self.command_accel = 0.0
            # Any stop that reached the base makes the next move a launch
            # (straight_launch), coasting wheels included.
            launch = getattr(self, "launch", None)
            if launch is not None:
                launch.note_stop()
            # The slope is dropped, the speed is not: resume from what the
            # wheels are doing, not from the zero the gate passed down.
            rolling = self.rolling_wheel_speed()
            if rolling is not None:
                self.current_speed = rolling
        # What reached the base, gate stops included, is the predictor's
        # input - not what the planner asked for.
        self.smith.on_command(rospy.Time.now().to_sec(), accepted_speed,
                              self.last_yaw_rate)

    def rolling_wheel_speed(self):
        """The wheels' forward speed if they are still turning, else None.

        None when the base report is stale or the chair is at rest, so the
        accepted command stands exactly as it did before this existed.
        """
        stamp = getattr(self, "wheel_status_stamp", None)
        if stamp is None:
            return None
        age = (rospy.Time.now() - stamp).to_sec()
        if age < 0.0 or age > WHEEL_SPEED_FRESH_S:
            return None
        measured = float(getattr(self, "measured_speed", 0.0))
        if not math.isfinite(measured) or measured <= ROLLING_SPEED_MPS:
            return None
        return min(measured, dwa_core.MAX_SPEED)

    def command_for_target(self, target_v, target_w, elapsed,
                           hold_yaw=False, limits=True):
        """The raw command the ramp will publish for one planner target.

        Candidate safety has to judge this command rather than the solver's
        target.  On the first cycle of a turn the yaw slew limiter can make
        those two very different; validating the target and publishing the
        ramped command would recreate the planner/gate disagreement one step
        later.

        From rest the speed goes straight to the launch floor, and with
        hold_yaw a moving target is commanded straight (straight_launch).
        """
        step = max(float(elapsed), 1e-3)
        wanted_accel = np.clip(
            (float(target_v) - self.current_speed) / step,
            -MAX_DECEL, MAX_ACCEL)
        command_accel = jerk_limited(
            wanted_accel, self.command_accel, step)
        yaw_accel = float(getattr(self, "yaw_accel", 0.0)) if limits else 0.0
        slew = yaw_accel if yaw_accel > 0.0 else YAW_SLEW_RPS2
        accel = np.array([
            command_accel,
            np.clip((float(target_w) - self.last_yaw_rate) / step,
                    -slew, slew)])
        speed, yaw_rate = advance_command(
            self.current_speed, self.last_yaw_rate, accel, elapsed,
            dwa_core.MAX_SPEED, MAX_YAW_RATE)
        floor = launch_speed(self.current_speed, target_v,
                             getattr(self, "launch_floor_mps", 0.0))
        if floor is not None and speed < floor:
            # Through the deadband in one step; the ramp (and its jerk
            # limit) resumes from the floor, not from a carried slope.
            speed, command_accel = floor, 0.0
        curvature_max = float(getattr(self, "curvature_max", 0.0)) \
            if limits else 0.0
        if curvature_max > 0.0:
            # B1 on what is actually sent this cycle; cutting |w| down to
            # the cap is allowed at once (dwa_core.yaw_range).
            cap = min(MAX_YAW_RATE, curvature_max * max(0.0, speed))
            yaw_rate = float(np.clip(yaw_rate, -cap, cap))
        if hold_yaw and float(target_v) >= dwa_core.TURN_FLOOR_SPEED:
            yaw_rate = 0.0
        return float(speed), float(yaw_rate), float(command_accel)

    def yaw_range_for(self, elapsed, hold_yaw=False):
        """v -> (lo, hi) for the planner, or None when B1 and B2 are off.

        Built from command_for_target itself, so the speed B1 caps on is the
        speed the ramp will send for that target this cycle (D included).
        """
        curvature_max = float(getattr(self, "curvature_max", 0.0))
        yaw_accel = float(getattr(self, "yaw_accel", 0.0))
        if curvature_max <= 0.0 and yaw_accel <= 0.0:
            return None
        # The window is what the ramp can reach in the same 1 s horizon the
        # speed samples use (dwa_core.yaw_range explains why not one cycle).
        step = dwa_core.VELOCITY_WINDOW_S
        speeds = {}
        # Taken now: the window is round the command already sent, and must
        # not move if the closure is asked again after this cycle's publish.
        last_w = float(self.last_yaw_rate)

        def bounds(v):
            if v not in speeds:
                speeds[v] = self.command_for_target(v, last_w, elapsed)[0]
            return dwa_core.yaw_range(
                speeds[v], last_w, step,
                curvature_max=curvature_max, yaw_accel=yaw_accel,
                limit=MAX_YAW_RATE)
        return bounds

    def log_launch_transition(self, was_active, held_s, left, right,
                              target_w):
        """One line per launch start and end, with the wheels' numbers."""
        launch = self.launch
        if launch.active and not was_active:
            rospy.loginfo(
                "LAUNCH start: wheels %.2f / %.2f m/s, planner w %+.2f held "
                "at 0", left, right, target_w)
        elif was_active and not launch.active:
            rospy.loginfo(
                "LAUNCH end (%s) after %.2f s: wheels %.2f / %.2f m/s",
                launch.last_exit or "?",
                -1.0 if held_s is None else held_s, left, right)

    def launch_inputs(self, now):
        """(left, right, fresh) from the base's own wheel report."""
        stamp = getattr(self, "wheel_status_stamp", None)
        fresh = False
        if stamp is not None:
            age = (now - stamp).to_sec()
            fresh = 0.0 <= age <= WHEEL_SPEED_FRESH_S
        return (float(getattr(self, "measured_left", 0.0)),
                float(getattr(self, "measured_right", 0.0)), fresh)

    def planner_candidate_veto(self, _now, _decision, _command_for_target):
        """Optional downstream-geometry veto, supplied by guarded profiles."""
        return None

    def planner_excluded_track_ids(self):
        """Tracks omitted only from DWA geometry, never from raw safety."""
        return ()

    def send_stop(self):
        # The jerk limit shapes driving, never braking. Dropping the carried
        # acceleration here is what keeps a stop as abrupt as it was before.
        self.command_accel = 0.0
        # A hold ends any launch; the next one starts from rest afresh.
        launch = getattr(self, "launch", None)
        if launch is not None:
            launch.reset("hold")
            launch.note_stop()
        WaypointFollower.send_stop(self)

    def on_imu(self, message):
        """The IMU's yaw rate, for the Smith model-error term."""
        self.imu_gz = float(message.angular_velocity.z)
        self.imu_stamp = rospy.Time.now()

    def residual_inputs(self, now_s):
        """(measured yaw rate, residual weight) for the Smith predictor.

        The yaw rate is the IMU's in "imu" mode while it is fresh, else the
        wheel report's (with a throttled warning: a dead IMU must not read
        as a chair that stopped turning). The weight is residual_weight on
        the wheels' speed, or 0 when the term is off.
        """
        mode = getattr(self, "smith_residual", "off")
        yaw_rate = float(getattr(self, "measured_yaw_rate", 0.0))
        self.residual_src = "wheel"
        if mode == "imu":
            stamp = getattr(self, "imu_stamp", None)
            age = None if stamp is None else float(now_s) - stamp.to_sec()
            if age is not None and -0.05 <= age <= SMITH_IMU_FRESH_S:
                yaw_rate = float(self.imu_gz)
                self.residual_src = "imu"
            else:
                rospy.logwarn_throttle(
                    10.0, "Smith residual: IMU %s, using the wheel yaw rate",
                    "never received" if age is None else "%.2f s old" % age)
        if mode == "off":
            self.residual_src = "off"
            weight = 0.0
        else:
            weight = residual_weight(getattr(self, "measured_speed", 0.0))
        self.residual_weight_now = weight
        return yaw_rate, weight

    def on_odom(self, message):
        """Wheel odometry, kept for the anchor as well as for the base."""
        WaypointFollower.on_odom(self, message)
        self.odom_v = float(message.twist.twist.linear.x)
        self.odom_w = float(message.twist.twist.angular.z)

    def obstacle_points(self, state):
        """The objects ahead, as the returns the rollouts must clear.

        Where they actually are, not straight ahead: placing every threat on
        the heading is what turned a wall beside the chair into one in front
        of it, and with the corridor 0.3 m wide at that station nothing could
        clear OBSTACLE_FLOOR_M and the profile held for 211 cycles on
        2026-08-09.

        And the whole of them, not one point. The nearest return of an object
        is where a stopping radius is measured from, and it was the only
        thing passed here until 2026-08-11 - so a wall spanning the corridor
        became a single point, and an arc could be admitted for clearing that
        one point by 0.41 m while driving through the metres of wall either
        side of it. cluster_guard.object_points publishes each lateral slice
        the object occupies, which is the same measurement, ungeneralised.
        """
        if not self.clusters_enabled or self.cluster_summary is None:
            return ()
        blocks, points = corridor_obstacle_points(
            self.cluster_summary, OBSTACLE_HALF_WIDTH_M,
            max_distance_m=PLAN_AHEAD_M,
            exclude_track_ids=self.planner_excluded_track_ids(),
            # This cycle's frame, set by corridor_threat just before: what
            # stands round the bend, and on the chair's own line, as well as
            # the straight strip.
            route_frame=getattr(self, "threat_route_frame", None))
        if not blocks or not points:
            return ()
        heading = np.array([math.cos(state[2]), math.sin(state[2])])
        left = np.array([-heading[1], heading[0]])
        return [state[:2] + heading * forward + left * lateral
                for forward, lateral in points]

    def led_state(self, state, now_s=0.0):
        """The state advanced by the actuation lag, along the arc it is on.

        With ~latency_mode smith it is advanced instead by the commands
        already sent (actuation_lag), and now_s is the predictor's clock.

        Only the pose is led; the speed and yaw rate stay as measured,
        because those are what the ramp integrates from. A lead longer than
        the real lag over-steers exactly as badly as no lead under-steers,
        so this is a measured number and not a tuning knob - and at zero it
        is the identity, which is what an unmeasured vehicle should use.
        """
        if self.latency_mode == "off":
            return state
        if self.latency_mode == "smith":
            yaw_rate, weight = self.residual_inputs(now_s)
            return np.array(self.smith.predict(
                state, self.measured_speed, yaw_rate, now_s,
                residual_weight=weight), dtype=float)
        lead = self.latency_s
        if lead <= 0.0:
            return state
        led = np.array(state, dtype=float)
        led[0] += state[3] * math.cos(state[2]) * lead
        led[1] += state[3] * math.sin(state[2]) * lead
        led[2] += state[4] * lead
        return led

    def step(self):
        now = rospy.Time.now()
        # Before the guards: route_locked is what arms OFF_ROUTE and
        # OFF_BAND, and it is set here.
        if self.pose_xy is not None:
            self.advance_progress()
        if self.handled_before_driving(now):
            self.anchor.reset("held")
            self.last_command_stamp = None
            return

        # /Odometry carries no twist - FAST-LIO leaves it at zero, and all
        # 13,395 samples of the 08-23 run are exactly 0.000 - so the anchor
        # had been folding in a velocity of nought and the rollouts started
        # from a chair that was never moving. The base reports what the
        # wheels are doing at 100 Hz; that is the measurement.
        state = self.anchor.update(self.pose_xy, self.pose_yaw,
                                   self.measured_speed, self.measured_yaw_rate,
                                   now.to_sec())
        # The speed policy still applies: the corridor ahead, the slope and
        # a DEGRADED fix all cap what any candidate may be worth choosing.
        v_ref, stop_reason = mpc_speed.shaped_reference(
            self.band, state[:2], 1, pitch_rad=self.pose_pitch,
            degraded=(self.tracking_state == "DEGRADED"))
        if stop_reason:
            self.publish_state("HOLD:SLOWER_THAN_FLOOR", "HOLD")
            self.send_stop()
            self.last_command_stamp = None
            return

        # Parked or moving, answered before the planner is asked anything.
        # Rolling velocities out and scoring them is a way of GOING ROUND
        # something, and going round is only ever the answer for an object
        # the tracker has watched stand still. Anything moving, or not yet
        # watched long enough to say, is waited out where it stands - a
        # sidestep is a manoeuvre into where they are about to be. Until
        # 2026-08-11 this profile never asked: it handed the nearest threat
        # to the planner whichever it was, and the planner would clear a
        # walking person by OBSTACLE_FLOOR_M and drive past.
        threat = self.corridor_threat(0.0)
        stop_m = self.stop_radius_for(threat)
        decision = self.avoidance_for(
            now, threat, self.threat_blocks(threat, stop_m))
        if decision == WAIT:
            self.publish_state("HOLD:DWA_WAIT", "HOLD:DWA_WAIT")
            self.dwa_status = "WAIT"
            self.send_stop()
            self.last_command_stamp = None
            return
        if decision == PERSON_BYPASS and self.tracking_state != "TRACKING":
            self.publish_state("HOLD:PERSON_BYPASS_TRACKING", "HOLD")
            self.send_stop()
            self.last_command_stamp = None
            return

        # Past our own WAIT, so whatever the gate is holding, it is not
        # something the cluster producer gave us. Say so and stop asking.
        if gate_stall(self.gate_reason, self.gate_blocked_for(now)):
            if self.dwa_status != "GATE_STALL":
                rospy.logwarn(
                    "gate is refusing an obstacle the follower cannot see: "
                    "%s - the cluster producer reports a clear corridor",
                    self.gate_detail)
            self.publish_state("HOLD:GATE_STALL " + self.gate_detail,
                               "HOLD:GATE_STALL")
            self.dwa_status = "GATE_STALL"
            self.send_stop()
            self.last_command_stamp = None
            return

        # Approach at the pursuit profile's pace, whatever the decision. A
        # planner that only knows stop-or-cruise arrives at what it is about
        # to wait for at full speed and stops there.
        cap = float(v_ref[0])
        if threat is not None:
            cap = approach_cap(cap, threat.distance_m, stop_m,
                               dwa_core.TURN_FLOOR_SPEED)
        if decision == PERSON_BYPASS:
            cap = min(cap, PERSON_BYPASS_SPEED_MPS)

        # Geometry only when going round it. Handing the planner an object it
        # is not allowed to go round would let it sidestep anyway.
        obstacles = self.obstacle_points(state) if decision in (
            GO_ROUND, PERSON_BYPASS) else ()
        # Plan from where the chair will be when the command lands, not from
        # where it is. The gap was measured on 2026-08-11 by cross-correlating
        # commanded angular.z against the yaw rate differentiated from
        # /fast_lio_icp/pose: the correlation peaks at 0.55 s with gain 1.03
        # and R^2 0.90, and the identical figure comes off /Odometry, so it is
        # upstream of the ICP correction. Unfixed, that lag costs 0.33 m of
        # travel at 0.6 m/s and 0.55 m at 1.0 - the planner correcting for
        # where the chair no longer is, which is what lateral hunting is.
        anchored = state
        state = self.led_state(state, now.to_sec())
        self.publish_smith_state(anchored, state, now)
        elapsed = 1.0 / CONTROL_HZ
        if self.last_command_stamp is not None:
            elapsed = (now - self.last_command_stamp).to_sec()
        if elapsed > MAX_COMMAND_GAP_S:
            rospy.logwarn_throttle(
                5.0, "control loop gap %.2f s - resyncing command to measured",
                elapsed)
            self.current_speed = float(state[3])
            self.last_yaw_rate = float(state[4])
        # Straight launch: decided before planning, so candidate safety
        # judges the straight command that will actually be sent. Never
        # while going round something - there the first metre is the turn.
        launch = getattr(self, "launch", None)
        left, right, wheels_fresh = self.launch_inputs(now)
        launch_allowed = decision not in (GO_ROUND, PERSON_BYPASS)
        hold_yaw = launch is not None and launch.eligible(
            self.current_speed, left, right, wheels_fresh, launch_allowed)
        was_active = launch is not None and launch.active
        held_s = None if launch is None else launch.held_for(now.to_sec())

        def plan_with(hold, limits=True):
            command_for_target = lambda v, w: self.command_for_target(
                v, w, elapsed, hold_yaw=hold, limits=limits)
            candidate_veto = self.planner_candidate_veto(
                now, decision, command_for_target)
            extra = {}
            yaw_range_for = self.yaw_range_for(elapsed, hold) \
                if limits else None
            if yaw_range_for is not None:
                extra["yaw_range_for"] = yaw_range_for
            return command_for_target, self.planner.plan(
                state, obstacles, speed_cap=cap,
                last_yaw_rate=self.last_yaw_rate,
                last_speed=self.current_speed,
                obstacle_floor_m=(
                    PERSON_BYPASS_CLEARANCE_M
                    if decision == PERSON_BYPASS
                    else dwa_core.OBSTACLE_FLOOR_M),
                candidate_veto=candidate_veto, **extra)

        command_for_target, (target_v, target_w, status) = plan_with(hold_yaw)
        if hold_yaw and status != "OK":
            # Straight was refused downstream; a straight launch is not worth
            # a stop. Plan again as before this existed.
            # Disarmed for this launch, so the next cycles plan once.
            launch.reset("no straight candidate")
            launch.armed = False
            hold_yaw = False
            command_for_target, (target_v, target_w, status) = \
                plan_with(False)
        relaxed = False
        if status != "OK" and self.yaw_range_for(elapsed) is not None:
            # B1/B2 are comfort limits, never a reason to stop: if nothing
            # inside them is admissible (a chair left skewed off the route
            # needs a tighter or quicker turn than they allow), plan this
            # cycle as before they existed.
            limited_status = status
            command_for_target, (target_v, target_w, status) = plan_with(
                hold_yaw, limits=False)
            relaxed = status == "OK"
            if relaxed and getattr(self, "steer_relaxed_reason", "") != \
                    limited_status:
                rospy.logwarn("DWA %s inside the steering limits; this "
                              "cycle planned without them", limited_status)
            self.steer_relaxed_reason = limited_status if relaxed else ""
        else:
            self.steer_relaxed_reason = ""
        if hold_yaw:
            hold_yaw = launch.update(
                now.to_sec(), self.current_speed, target_v, left, right,
                wheels_fresh, launch_allowed)
            if not hold_yaw:
                command_for_target = lambda v, w: self.command_for_target(
                    v, w, elapsed, limits=not relaxed)
        elif launch is not None:
            launch.reset("not eligible")
        if launch is not None:
            self.log_launch_transition(was_active, held_s, left, right,
                                       target_w)
        if status != "OK":
            if status != self.dwa_status:
                if status == "SPEED_BELOW_FLOOR":
                    rospy.logwarn(
                        "DWA %s at wp %d/%d: cap %.2f m/s is under the "
                        "%.2f m/s the wheels turn for (v_ref %.2f%s)",
                        status, self.nearest_index, len(self.waypoints),
                        cap, dwa_core.TURN_FLOOR_SPEED, float(v_ref[0]),
                        ", threat guard" if threat is not None else "")
                else:
                    rospy.logwarn("DWA %s at wp %d/%d", status,
                                  self.nearest_index, len(self.waypoints))
            self.dwa_status = status
            self.publish_state("HOLD:DWA_" + status, "HOLD:DWA_" + status)
            self.send_stop()
            self.last_command_stamp = None
            return
        self.dwa_status = status

        self.last_command_stamp = now
        speed, yaw_rate, self.command_accel = command_for_target(
            target_v, target_w)

        command = Twist()
        command.linear.x = speed
        command.angular.z = yaw_rate if speed > 0.02 else 0.0
        self.cmd_pub.publish(command)
        self.current_speed = speed
        self.last_yaw_rate = yaw_rate
        self.publish_state(
            "DWA wp=%d/%d v=%.2f w=%+.2f target %.2f/%+.2f%s%s" % (
                self.nearest_index, len(self.waypoints), speed, yaw_rate,
                target_v, target_w,
                (" LAUNCH" if hold_yaw else "")
                + (" STEER_RELAXED" if relaxed else ""),
                "" if self.policies else " POLICIES_OFF"), "DWA:OK")

    def smith_state_record(self, anchored, led, now):
        """What the lag compensation saw and did this cycle, as a dict.

        anchored is the planner state before the lead (StateAnchor output),
        led the state the planner actually plans from. The model_* fields are
        the Smith base model with delay brought up to now - compare them with
        meas_* for the model error, and lead_m/lead_yaw_deg with the turn
        the chair then makes for whether the lead was long enough.
        """
        def num(value, digits=4):
            value = float(value)
            return round(value, digits) if math.isfinite(value) else None

        def age(stamp):
            if stamp is None or (hasattr(stamp, "is_zero")
                                 and stamp.is_zero()):
                return None
            return num((now - stamp).to_sec(), 3)

        now_s = now.to_sec()
        dyaw = math.atan2(math.sin(led[2] - anchored[2]),
                          math.cos(led[2] - anchored[2]))
        record = {
            "t": round(now_s, 3),
            "mode": self.latency_mode,
            "pose_age_s": age(getattr(self, "pose_stamp", None)),
            "wheel_age_s": age(getattr(self, "wheel_status_stamp", None)),
            "anchor": [num(v) for v in anchored[:5]],
            "led": [num(v) for v in led[:3]],
            "lead_m": num(math.hypot(led[0] - anchored[0],
                                     led[1] - anchored[1]), 3),
            "lead_yaw_deg": num(math.degrees(dyaw), 2),
            "meas_v": num(getattr(self, "measured_speed", float("nan"))),
            "meas_w": num(getattr(self, "measured_yaw_rate", float("nan"))),
            "wheel_l": num(getattr(self, "measured_left", float("nan"))),
            "wheel_r": num(getattr(self, "measured_right", float("nan"))),
        }
        launch = getattr(self, "launch", None)
        record.update({
            # Which of the launch switches this drive ran with, every
            # message, so a drive log can never be read against the wrong
            # configuration.
            "launch_floor": num(getattr(self, "launch_floor_mps", 0.0), 3),
            "straight_launch": bool(launch is not None and launch.enabled),
            "launch": bool(launch is not None and launch.active),
            "launch_armed": bool(launch is not None and launch.armed),
            "launch_exit": "" if launch is None else launch.last_exit,
            "curvature_max": num(getattr(self, "curvature_max", 0.0), 3),
            "yaw_accel": num(getattr(self, "yaw_accel", 0.0), 3),
            "steer_relaxed": getattr(self, "steer_relaxed_reason", ""),
            "residual_mode": getattr(self, "smith_residual", "off"),
            "residual_src": getattr(self, "residual_src", ""),
            "residual_weight": num(getattr(self, "residual_weight_now", 0.0),
                                   3),
            "imu_gz": num(getattr(self, "imu_gz", float("nan"))),
        })
        smith = getattr(self, "smith", None)
        if self.latency_mode == "smith" and smith is not None:
            held = smith.command_at(now_s)
            record.update({
                "horizon_s": num(smith.horizon_s, 3),
                "model_v": num(smith.speed.output),
                "model_w": num(smith.yaw.output),
                "resid_v": num(smith.residual[0]),
                "resid_w": num(smith.residual[1]),
                "held_v": num(held[0]),
                "held_w": num(held[1]),
                "n_cmds": len(smith.stamps),
                "last_cmd_age_s": num(now_s - smith.stamps[-1], 3)
                if smith.stamps else None,
            })
        elif self.latency_mode == "lead":
            record["horizon_s"] = num(self.latency_s, 3)
        return record

    def publish_smith_state(self, anchored, led, now):
        """Publish smith_state_record; never lets logging stop the drive."""
        publisher = getattr(self, "smith_state_pub", None)
        if publisher is None:
            return
        try:
            publisher.publish(String(data=json.dumps(
                self.smith_state_record(anchored, led, now),
                separators=(",", ":"))))
        except Exception as exc:  # diagnostics only
            rospy.logwarn_throttle(30.0, "smith_state publish failed: %r",
                                   exc)

    def publish_state(self, text, state=None):
        """Publish every cycle, log only on a change of state.

        The line carries v and w, which move every cycle, so comparing the
        whole line to decide whether to log would log at 10 Hz.
        """
        self.status_pub.publish(String(data=text))
        state = state or text
        if state != self.status:
            rospy.loginfo("state: %s", text)
            self.status = state


if __name__ == "__main__":
    try:
        DwaFollower().run()
    except rospy.ROSInterruptException:
        pass
