<h1 align="center">Line Following Robot</h1>

<p align="center"><strong>Calibrated sensing, PID steering and an event-driven state machine for the Pololu 3Pi+ 32U4</strong></p>

<div align="center">

https://github.com/user-attachments/assets/c94fdcc3-b232-45a5-b280-1699de687c68

</div>

Firmware for the EMATM0053 Robotic Systems line following challenge: a Pololu
3Pi+ 32U4 that joins a line, follows it through corners and crossroads, and
recovers when the line is lost. The recording above was captured from the
original coursework firmware and predates the rewrite described below. It is
kept for context, not as evidence that the current code behaves the same way.

The firmware has been rewritten around three changes: sensor readings are
calibrated and normalised rather than compared against a fixed raw threshold,
steering is driven by a PID controller instead of a bare proportional term, and
the navigation state machine is event-driven with no blocking delays in the
control path. The pin map and the mechanical setup are unchanged.

## What is implemented

- Calibrated reflectance sensing: a single-pass read of all five sensors taken
  at one instant, per-sensor minimum and maximum captured during a calibration
  sweep, and normalisation to a fixed 0 to 1000 scale.
- Bounded sensor timing: every discharge wait is capped by
  `SENSOR_TIMEOUT_US`, so no read loop can spin indefinitely, and the emitter
  is always switched off before the read returns.
- Weighted line position over the five normalised readings, returned in the
  range -1.0 to +1.0, with a total-activation floor that doubles as the
  divide-by-zero guard and as the "no line present" signal.
- A PID controller with anti-windup on the integral term, output clamping, no
  derivative spike on the first sample after a reset, and a guard against a
  zero millisecond timestep.
- A navigator built as an explicit finite state machine with entry actions per
  state, non-blocking turns that exit on line reacquisition or timeout, and an
  idempotent halted state that stops the motors without trapping in a loop.
- Motor primitives that clamp to the PWM limit and return immediately. Nothing
  in the motor layer blocks, and `stop()` no longer requires a hardware reset
  to leave.
- Host-side C++ unit tests that build the firmware headers against an Arduino
  stub with a virtual clock and virtual pins, plus a watchdog that fails a test
  rather than hanging the suite when a read loop has no timeout.
- Continuous integration that compiles the sketch for the target board with
  arduino-cli and runs the host test suite on every push to `main` and on
  every pull request against it.

## What still requires validation

No robot hardware has been available at any point during this rewrite, and the
firmware has never been run on one. There are exactly two forms of verification
behind the claims above, and neither of them involves a moving robot:

1. The sketch compiles for the ATmega32U4 via arduino-cli against the
   `pololu-a-star:avr:a-star32U4` board core.
2. The host test suite compiles the pure logic (line position, PID, state
   transitions) with `g++` and executes it against the stub, on the developer
   machine and in CI.

Everything that only physical hardware can settle is therefore open. The
calibration sweep has never seen a real black line or a real white surface.
Motor polarity has not been confirmed on an assembled robot. The PID gains have
never been evaluated against real inertia, wheel slip or track geometry, and the
turn and rediscover timings are inherited numbers, not measured ones. Sensor
read timing has been reasoned about but not observed on target. Treat the
shipped constants as a starting point and work through [TUNING.md](TUNING.md)
with the wheels off the ground before trusting the robot on a track.

The integral and derivative gains ship at 0.0 for this reason: with both at
zero the controller reduces exactly to the proportional response of the
original coursework firmware, which is the last configuration known to have
driven a real robot.

## Hardware

Pololu 3Pi+ 32U4 (ATmega32U4), FQBN `pololu-a-star:avr:a-star32U4`. The pin
assignments are inherited from the coursework platform and are not
configurable in any meaningful sense:

| Signal | Pin | Notes |
|---|---|---|
| EMIT (IR emitter) | 11 | INPUT is off, OUTPUT and HIGH is on |
| DN1 (far left) | 12 | |
| DN2 (left) | A0 | |
| DN3 (middle) | A2 | |
| DN4 (right) | A3 | |
| DN5 (far right) | A4 | |
| Left motor PWM | 10 | |
| Left motor direction | 16 | Forward is LOW, reverse is HIGH |
| Right motor PWM | 9 | |
| Right motor direction | 15 | |

## Build and upload

The sketch builds with arduino-cli. The Pololu A-Star core depends on
`arduino:avr` for its toolchain, so both cores must be installed, in this
order:

```bash
arduino-cli config init --overwrite
arduino-cli config add board_manager.additional_urls \
  https://files.pololu.com/arduino/package_pololu_index.json
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli core install pololu-a-star:avr

arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4 line_following_robot
arduino-cli upload --fqbn pololu-a-star:avr:a-star32U4 \
  -p <PORT> line_following_robot
```

The Arduino IDE works equally well once the Pololu board index has been added
under Preferences. Replace `<PORT>` with the port the robot enumerates on
(`arduino-cli board list` will show it).

## Core workflow

Each pass of `loop()` takes one snapshot of all five sensors and hands it to
the navigator with the current millisecond timestamp. The navigator owns all
behaviour; the sketch contains wiring only.

```text
read all five sensors (single pass, one instant)
  → clamp raw microseconds, normalise against calibration
  → weighted line position in [-1, +1] plus line-present flag
  → navigator state machine
  → PID correction about the line position error
  → left/right PWM
```

The state machine:

```text
        Calibrate
            |  sweep complete
            v
        JoinLine
            |  line acquired (debounced)
            v
   +--> FollowLine <-------------------------------+
   |      |   |   |                                |
   |      |   |   +--> Crossroads -----------------+
   |      |   |          (both far sensors active) |
   |      |   |          far sensors clear ........+
   |      |   |
   |      |   +------->  TurnRight ----------------+
   |      |                (far right active)      |
   |      |                                        |
   |      +--------->    TurnLeft -----------------+
   |                       (far left active)       |
   |                                               |
   |                     line reacquired ..........+
   |                     turn timeout ....\
   |  line lost                            \
   v                                        v
Rediscover <--------------------------------+
   |      |
   |      +--  line reacquired  ------------------->  FollowLine
   |
   |  rediscover timeout
   v
 Halted  (motors stopped, idempotent, no trap loop)
```

Turns are timed rather than blocking: entering a turn state records the start
time, each `update()` ticks it forward, and the state exits when the line is
reacquired or `TURN_TIMEOUT_MS` expires. Sensors keep being read throughout,
which was not true of the original firmware.

Two details of that turn behaviour are worth stating, because neither is
obvious from the state names. A turn ignores the line for the first
`TURN_SETTLE_MS`, since the line it started on is still under the sensors and
would otherwise end the turn immediately. And a turn that times out exits to
`Rediscover` rather than `FollowLine`: having swung round without finding a
line, it has not found one, and resuming line following would be a claim the
robot cannot support.

## Tuning and calibration

Every tunable constant lives in `line_following_robot/config.h`. Nothing is
tuned by editing logic. See [TUNING.md](TUNING.md) for the bench procedure,
which assumes the wheels are off the ground and works up from motor polarity
through calibration to the PID gains.

Setting `BENCH_MODE` to 1 in that file builds a diagnostic firmware instead of
the normal one: it opens the serial port, drives each motor primitive in turn
so polarity can be checked by eye, runs a calibration sweep and prints the
per-sensor bounds it recorded. It blocks throughout and never enters the state
machine, so it belongs on a stand rather than on a course. The normal build
carries none of it.

## Repository layout

```text
line_following_robot/
  line_following_robot.ino   setup() and loop() wiring only, no logic
  config.h                   pins and tuning constants, no code
  linesensors.h              LineSensor_c, SensorSnapshot
  motors.h                   Motors_c
  pid.h                      PID_c
  navigator.h                Navigator_c, NavState, Junction
test/
  arduino_stub.h             virtual clock, virtual pins, read watchdog
  test_harness.h             CHECK, CHECK_EQ, CHECK_NEAR
  test_motors.cpp            clamping, direction, stop
  test_sensors.cpp           single-pass read, calibration, line position
  test_pid.cpp               anti-windup, clamping, derivative behaviour
  test_navigator.cpp         state transitions and turn exits
  run_tests.sh               compiles and runs every test/test_*.cpp
.github/workflows/ci.yml     sketch compile and host test suite
README.md
TUNING.md                    bench procedure for deriving the constants
LICENSE                      MIT
.gitignore
```

The firmware classes are header-only, matching the style of the original
coursework code.

## Tests

```bash
bash test/run_tests.sh
```

The runner compiles each `test/test_*.cpp` with
`-std=c++17 -Wall -Wextra` and both source directories on the include path,
using `$CXX` if set and `g++` otherwise, runs each binary, and
exits non-zero if any binary fails or fails to compile. The Arduino stub
advances a virtual clock by one microsecond per `digitalRead()` and trips a
watchdog after a large number of reads, so an unbounded firmware read loop
fails an assertion instead of hanging the suite.

These tests cover the pure logic only. They say nothing about motor behaviour,
sensor response or anything else that depends on the physical robot.

## Contributing

Run `bash test/run_tests.sh` and compile the sketch for
`pololu-a-star:avr:a-star32U4` before submitting changes. Keep claims tied to
what has actually been executed: if a change has only been compiled and unit
tested, say so, and leave the validation section above honest.

## License and acknowledgments

Released under the [MIT License](LICENSE).

- Dr Paul O'Dowd and the University of Bristol EMATM0053 Robotic Systems
  coursework team.
- Pololu Corporation for the 3Pi+ mobile robot and its support materials.
