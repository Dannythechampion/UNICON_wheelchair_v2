"""A Smith predictor for the base's actuation lag.

The base answers a wheel command late and slowly. Measured on the three
2026-11-02 afternoon runs (A_to_D4, D4_to_D5, D5_to_A), fitting a dead time
plus a first-order lag from the accepted /cmd_vel to the base's own
/wheel_status report:

    yaw rate  dead 0.28 s, time constant 0.40 s, gain 1.0
    speed     dead 0.16 s, time constant 0.80 s, gain 1.0

Nothing on the NUC adds to it - /cmd_vel to /wheel_cmd is 0.00 s and the
UART write takes 0.05 ms - and the IMU and FAST-LIO odometry lag the command
by the same 0.6 s as the wheel report, so it is the base, not localisation.
The follower's own chain (DWA output to /cmd_vel through the person-bypass,
terrain, gate and tip guards) adds a further 0.12-0.14 s.

dwa_follower.led_state used to cover this by extending the arc the chair is
on by LATENCY_S. That arc is drawn from the MEASURED yaw rate, which is the
answer to commands sent 0.6 s ago: a turn the planner has already reversed
keeps being projected for another 0.6 s, and the planner piles on more
command to fight a turn that is already being undone. This predictor knows
what was sent.

THE FORM USED
-------------
The plan starts from the measured pose and is carried one dead time ahead
on the model's own output, driven by every command already sent:

    prediction = measured pose + integral of model_without_delay(u)

The pose is re-anchored to the localisation every cycle, so a model error
cannot accumulate in it; only the motion over the 0.41 s horizon is modelled.

The classic Smith predictor adds the model's error carried forward,
(measured - model_with_delay(u)), to that motion. It is here, low-passed,
behind residual_tau_s, and OFF by default. On the 11-02 replay it did not
help: heading error at the horizon was p50 0.70 deg without it against 0.80
deg with it on D5_to_A, and 1.17-1.28 against 1.25-1.63 deg through turn
reversals. The measurement it pulls toward is the reason: /wheel_status
reads the yaw rate 10-15 % low against the IMU, and it is quantised to
0.05 rad/s of yaw rate per count. Turn it on for a slope or a load that
biases the base, preferably measured off the IMU rather than the wheels.

What neither form can fix is a wrong dead time. Too long over-leads exactly
as a too-long LATENCY_S did; so the horizon is the measured one, not a knob.
"""

import bisect
import math

# Model step. /cmd_vel arrives at about 50 Hz.
DT_S = 0.02

YAW_DEAD_S = 0.28
YAW_TAU_S = 0.40
YAW_GAIN = 1.0
SPEED_DEAD_S = 0.16
SPEED_TAU_S = 0.80
SPEED_GAIN = 1.0

# DWA output to the accepted /cmd_vel, through the downstream guards.
CHAIN_S = 0.13

# Low-pass on (measured - model) when the error term is on; 0 turns it off.
# 0.4 s averages out a count of wheel quantisation and still follows a
# slope coming on.
RESIDUAL_TAU_S = 0.0

# Commands older than this cannot reach the model any more; a model that
# has not been advanced for this long restarts from the measurement.
HISTORY_S = 2.0


class LagChannel:
    """One velocity channel: a dead time, then a first-order lag."""

    def __init__(self, dead_s, tau_s, gain):
        self.dead_s = float(dead_s)
        self.gain = float(gain)
        self.alpha = 1.0 - math.exp(-DT_S / float(tau_s))
        self.output = 0.0

    def step(self, command):
        self.output += self.alpha * (self.gain * command - self.output)
        return self.output


class SmithPredictor:
    """Where the chair will be when the next command starts to act."""

    def __init__(self, chain_s=CHAIN_S, residual_tau_s=RESIDUAL_TAU_S,
                 speed=(SPEED_DEAD_S, SPEED_TAU_S, SPEED_GAIN),
                 yaw=(YAW_DEAD_S, YAW_TAU_S, YAW_GAIN)):
        self.chain_s = float(chain_s)
        self.residual_tau_s = float(residual_tau_s)
        self.speed_params = tuple(speed)
        self.yaw_params = tuple(yaw)
        self.stamps = []
        self.commands = []
        self.reset()

    @property
    def horizon_s(self):
        """From publishing a command to the base starting to turn on it."""
        return self.chain_s + self.yaw_params[0]

    def reset(self):
        """Forget the model state; the next predict restarts from measured."""
        self.speed = LagChannel(*self.speed_params)
        self.yaw = LagChannel(*self.yaw_params)
        self.model_s = None
        self.residual = [0.0, 0.0]

    def on_command(self, stamp_s, speed, yaw_rate):
        """Record a command that reached the base (the accepted /cmd_vel)."""
        stamp_s = float(stamp_s)
        if self.stamps and stamp_s < self.stamps[-1]:
            # A clock that went backwards (bag restart, sim reset) leaves
            # the history meaningless.
            self.stamps, self.commands = [], []
            self.reset()
        self.stamps.append(stamp_s)
        self.commands.append((float(speed), float(yaw_rate)))
        cut = bisect.bisect_left(self.stamps, stamp_s - HISTORY_S)
        if cut:
            del self.stamps[:cut]
            del self.commands[:cut]

    def command_at(self, stamp_s):
        """The command the base was holding at stamp_s (zero before any)."""
        index = bisect.bisect_right(self.stamps, stamp_s) - 1
        if index < 0:
            return 0.0, 0.0
        return self.commands[index]

    def advance(self, speed, yaw, start_s, end_s):
        """Step both channels from start_s to end_s on the recorded input."""
        t = start_s
        while t < end_s - 1e-9:
            t += DT_S
            speed.step(self.command_at(t - speed.dead_s)[0])
            yaw.step(self.command_at(t - yaw.dead_s)[1])
        return t

    def predict(self, state, measured_speed, measured_yaw_rate, now_s,
                residual_weight=1.0):
        """state (x, y, yaw, v, w) -> the pose one horizon ahead.

        residual_weight scales how much of the carried model error (when the
        error term is on) is applied over the horizon; the error itself is
        tracked at full rate either way, so it is warm when the weight rises.
        The follower schedules it on speed: full where the model is wrong
        (under the wheels' deadband), none at cruise where it is right.

        Speed and yaw rate are returned as measured, not as predicted: they
        are what the command ramp resynchronises from, and a predicted value
        there would ramp from a velocity the base has not reached.
        """
        now_s = float(now_s)
        measured = (float(measured_speed), float(measured_yaw_rate))
        if self.model_s is None or now_s - self.model_s > HISTORY_S \
                or now_s < self.model_s:
            self.reset()
            self.speed.output, self.yaw.output = measured
            self.model_s = now_s
        else:
            # The model with delay, brought up to now: what the base should
            # be doing on the commands that have had time to land.
            elapsed = now_s - self.model_s
            self.model_s = self.advance(self.speed, self.yaw,
                                        self.model_s, now_s)
            if self.residual_tau_s > 0.0:
                blend = 1.0 - math.exp(-elapsed / self.residual_tau_s)
                for i, model in enumerate((self.speed.output,
                                           self.yaw.output)):
                    self.residual[i] += blend * (
                        measured[i] - model - self.residual[i])

        # The model without delay: the same channels carried on over the
        # horizon on the commands already sent (plus the carried error, when
        # that term is on; it is zero otherwise).
        speed = LagChannel(*self.speed_params)
        yaw_channel = LagChannel(*self.yaw_params)
        speed.output, yaw_channel.output = self.speed.output, self.yaw.output
        x, y, heading = float(state[0]), float(state[1]), float(state[2])
        weight = min(max(float(residual_weight), 0.0), 1.0)
        carried_v = weight * self.residual[0]
        carried_w = weight * self.residual[1]
        t, end_s = self.model_s, self.model_s + self.horizon_s
        while t < end_s - 1e-9:
            t += DT_S
            v = speed.step(self.command_at(t - speed.dead_s)[0]) \
                + carried_v
            w = yaw_channel.step(self.command_at(t - yaw_channel.dead_s)[1]) \
                + carried_w
            mid = heading + 0.5 * w * DT_S
            x += v * math.cos(mid) * DT_S
            y += v * math.sin(mid) * DT_S
            heading += w * DT_S
        led = [x, y, heading] + [float(s) for s in state[3:]]
        return led
