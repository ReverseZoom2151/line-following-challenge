# Tuning and calibration

This is a bench procedure for the first time the rewritten firmware meets a
real robot. It has not itself been carried out: no hardware has been available
since the rewrite, and the firmware is verified only by an arduino-cli compile
for the ATmega32U4 and by host-side unit tests over the pure logic. Every
number below is a starting point, not a measurement.

Work through the steps in order. Each one assumes the previous one passed. If
a step fails, stop and fix it rather than compensating for it later in the
chain: a polarity error corrected by a sign flip in the gains will produce a
robot that appears to work and then fails unpredictably.

## Where the constants live

All tunable values are in `line_following_robot/config.h`. Nothing in this
document asks you to edit logic. The constants referenced here are:

| Constant | Ships as | Controls |
|---|---|---|
| `BASE_SPEED_PWM` | 30.0 | Forward bias applied to both wheels |
| `MAX_PWM` | 255.0 | Output clamp on each channel |
| `PID_KP` | 40.0 | Proportional gain on line position error |
| `PID_KI` | 0.0 | Integral gain |
| `PID_KD` | 0.0 | Derivative gain |
| `PID_INTEGRAL_LIMIT` | 50.0 | Anti-windup clamp on the integral term |
| `LINE_PRESENT_THRESHOLD` | 200 | Activation floor for "a line is here" |
| `JUNCTION_THRESHOLD` | 600 | Far-sensor activation that counts as a junction |
| `NORMALISED_MAX` | 1000 | Full scale after calibration |
| `SENSOR_TIMEOUT_US` | 2500 | Discharge give-up time, and the budget for the whole five-sensor pass |
| `CALIBRATION_MS` | 3000 | Duration of the calibration sweep |
| `TURN_TIMEOUT_MS` | 1200 | Maximum time spent in a turn state |
| `TURN_SETTLE_MS` | 150 | How long a turn ignores the line it started on |
| `REDISCOVER_TIMEOUT_MS` | 2000 | Maximum time spent hunting for a lost line |

### Why the integral and derivative gains ship at zero

`PID_KI` and `PID_KD` default to 0.0 deliberately. With both at zero the
controller reduces to a pure proportional response, and `PID_KP = 40.0` was
chosen to reproduce the original coursework controller exactly. That controller
computed `LeftPWM = BiasPWM + MaxTurnPWM * W` with `W` in the range -2 to +2;
the rewritten `linePosition()` returns -1 to +1, so a gain of `2 * MaxTurnPWM`,
that is 40.0, gives identical output at full deflection.

This matters because the proportional configuration is the last one known to
have driven a real robot. Shipping non-zero integral or derivative gains that
have never been tested on hardware would mean the default behaviour is
unvalidated in a way the old firmware was not. Turn them on deliberately,
during the procedure below, once you can watch what they do.

## 1. Safety first: wheels off the ground

For every first run after a firmware change, put the robot on a bench block or
stand so both wheels spin freely in the air. Keep it there for steps 2 to 4.

The failure modes that matter most here (reversed motor polarity, a gain high
enough to saturate both channels, a state machine that never leaves a turn) all
present as the robot driving hard in an unexpected direction. On a bench that is
information; on a table edge it is a repair. Have the power switch within reach
and keep the USB cable clear of the wheels.

## 2. Verify motor polarity and direction

Before anything reads a sensor, confirm the motors do what they are told.
Drive `driveStraight(BASE_SPEED_PWM)` and watch both wheels.

- Both wheels must rotate in the forward direction. If both run backwards, the
  direction sense is inverted globally; if one runs backwards, that channel's
  direction pin or motor leads are reversed.
- `spinLeft()` must rotate the robot anticlockwise viewed from above, and
  `spinRight()` clockwise. Confirm this by which wheel leads, not by guessing.
- `stop()` must halt both wheels and return. If the robot becomes
  unresponsive and needs a reset to recover, you are running an old build.

Correct any polarity problem in the pin configuration or the wiring. Do not
proceed with an inverted channel: every later step assumes a positive
correction steers one specific way.

## 3. Verify the calibration sweep

Calibration runs for `CALIBRATION_MS` on entry to the `Calibrate` state and
records a per-sensor minimum and maximum. It is only meaningful if, during that
window, every sensor sees both a black line and a white surface.

- Sweep the robot across the line for the full calibration window so that all
  five sensors, including DN1 and DN5, pass over black and over white. A single
  slow sweep in each direction is usually enough. Rotating in place over the
  line works too, provided the far sensors actually cross it.
- Afterwards, inspect the recorded minima and maxima for each sensor. Call
  `sensors.reportCalibration()` once from `setup()` after the sweep, or from a
  serial command, and read the result over the serial monitor at 9600 baud. It
  prints one comma-separated row per sensor: index, minimum, maximum, span.
  `calibrationMin(i)` and `calibrationMax(i)` return the same values if you
  would rather format them yourself. Do not call either from `loop()`:
  transmitting takes long enough to blind the robot between readings.
- Sanity checks: the maximum must be meaningfully above the minimum for every
  sensor (a near-equal pair means that sensor never saw one of the two
  surfaces); no maximum should sit at `SENSOR_TIMEOUT_US`, which means the
  sensor timed out rather than discharging and the surface is darker or further
  away than the timeout allows; the five sensors should be broadly comparable,
  since one wildly different sensor usually means dirt, damage, or a ride
  height problem on that side.
- With the robot centred on the line, the normalised middle reading should be
  near `NORMALISED_MAX` and the outer readings low. Off the line entirely,
  total activation should fall below `LINE_PRESENT_THRESHOLD`.

If a sweep produces poor ranges, redo it rather than adjusting thresholds to
compensate. `LINE_PRESENT_THRESHOLD` and `JUNCTION_THRESHOLD` are expressed in
normalised units precisely so they need not change per surface. Only revisit
them once calibration itself is clean, and then only in small steps: raise
`LINE_PRESENT_THRESHOLD` if the robot claims to see a line on a blank surface,
lower it if it drops the line while still visibly over it.

## 4. Derive PID_KP

Keep the wheels off the ground for the first check, then move to a straight
section of track for the real work.

1. Start at the shipped default, `PID_KP = 40.0`, with `PID_KI` and `PID_KD`
   at 0.0. This is the original proportional response.
2. Place the robot on a straight line and let it follow. Note whether it holds
   the line, drifts off, or weaves.
3. If the response is sluggish (the robot corrects too late and cuts corners or
   loses the line on gentle bends), increase `PID_KP`. Move in steps of roughly
   20 percent and re-test on the same section each time.
4. Keep increasing until the robot visibly oscillates around the line: a
   steady, sustained left-right weave on a straight section rather than an
   occasional wobble. Record that value.
5. Back off by roughly 30 percent from the oscillation point. That is the
   working proportional gain.

If the robot oscillates at the shipped 40.0, work downwards instead: halve the
gain until the weave stops, then come back up until it reappears, and again
take 30 percent below that point.

Tune `BASE_SPEED_PWM` and `PID_KP` together, not independently. A gain tuned at
one speed will not hold at another, because the correction has less distance to
act over as the robot goes faster. Fix a speed you are happy with first, tune
the gain to it, and re-tune if you change the speed later.

## 5. Add PID_KD, then consider PID_KI

Derivative gain damps the oscillation that step 4 deliberately provoked and
then backed away from. It acts on the rate of change of the error, so it
opposes fast swings without affecting steady tracking.

1. With `PID_KP` set, introduce a small `PID_KD`. A tenth of the proportional
   gain is a reasonable first guess; increase from there.
2. Increase `PID_KD` until the residual weave settles and the robot tracks the
   line cleanly through bends.
3. Stop as soon as the motion becomes jittery or the robot feels twitchy over
   small surface imperfections. Excessive derivative gain amplifies sensor
   noise, and the symptom is high-frequency judder rather than a slow weave.
4. With the damping in place you can often raise `PID_KP` slightly and re-test.
   Do this at most once or twice, and re-check the oscillation point after.

Only then consider `PID_KI`, and only if a genuine steady-state offset
persists: the robot tracks stably but consistently to one side of the line and
never centres. That is the one problem integral gain solves. If the robot
centres and merely overshoots, the answer is `PID_KD` or a lower `PID_KP`, not
integral gain.

If you do need it, start very small (an order of magnitude below `PID_KD`) and
increase slowly. Watch for wind-up on long turns, where the accumulated error
keeps steering after the robot has already recovered.
`PID_INTEGRAL_LIMIT` bounds this, so if raising `PID_KI` causes overshoot after
sustained curves, lower the limit before lowering the gain. A line follower
usually does not need integral action at all; leaving `PID_KI` at 0.0 is a
legitimate final answer.

## 6. Time the turn and rediscover behaviour

The remaining constants are durations, and they can only be set by watching the
robot at the speed you settled on in steps 4 and 5.

**Turns.** `TURN_TIMEOUT_MS` (1200 ms) is a safety bound, not the expected turn
duration. A turn ignores the line for its first `TURN_SETTLE_MS` (150 ms),
because the line it started on is still under the sensors and would otherwise
end the turn immediately; after that window it exits on the first reacquisition.
In normal operation the timeout should never fire.

- Time how long a 90 degree corner actually takes at your base speed. Do this
  from the outside: start a stopwatch as the robot commits to the turn and stop
  it as it straightens up.
- Set `TURN_TIMEOUT_MS` to comfortably more than that, roughly double, so that
  a normal turn always completes on line reacquisition.
- Check `TURN_SETTLE_MS` against that measured time. It must be short enough
  that the robot has not already swung onto the new line before the window
  closes, or the turn will overshoot every corner by a fixed amount. If turns
  overshoot consistently and reducing the turn speed does not help, this is the
  constant to suspect.
- If turns end on the timeout instead of on reacquisition, the robot drops into
  `Rediscover` rather than resuming line following, which looks like a hesitation
  after every corner. Either the timeout is too short, the settle window is
  swallowing the reacquisition, or the centre sensors are not reaching
  `LINE_PRESENT_THRESHOLD`, which is what the exit condition tests through
  `onLine()`. `JUNCTION_THRESHOLD` governs entering a turn, not leaving one.
- If the robot spins past the line and keeps going, the timeout is too long
  relative to the spin speed. Reduce the turn speed rather than the timeout.

**Rediscover.** `REDISCOVER_TIMEOUT_MS` (2000 ms) bounds how long the robot
hunts for a line it has lost before halting.

- Deliberately remove the line, by lifting the robot off the track mid-run or
  using a track with a gap, and time how long the rediscover behaviour takes to
  find it again when it succeeds.
- Set the timeout above that, with margin, but not so high that a robot which
  has genuinely left the track wanders for a long time before stopping.
- Verify the halted state on purpose: let the rediscover timeout expire and
  confirm the motors stop and stay stopped, and that the robot does not need a
  power cycle to recover.

**Junctions.** Finally, run the full track and watch the crossroads and corner
classifications specifically. `JUNCTION_THRESHOLD` (600 normalised units) sets
how much far-sensor activation counts as a junction. Raise it if ordinary bends
are being read as corners; lower it if real corners are missed. Change it only
after calibration is known good, since it is meaningless against a bad
normalisation.

## Recording what you find

Every number in `config.h` is currently an untested default. When a value has
been set on real hardware, note in the commit message what was observed and at
what base speed, so the next person can tell a measured constant from an
inherited one. Update the validation section of the README as items move from
untested to confirmed.
