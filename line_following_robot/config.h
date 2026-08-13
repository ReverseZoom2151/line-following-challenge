#ifndef CONFIG_H
#define CONFIG_H

// Central place for every pin and every tuning constant. No code lives here,
// so a value can be changed and the robot reflashed without reading any of
// the logic. The pin map matches the Pololu 3Pi+ 32U4 wiring and must not be
// altered.

#include <stdint.h>

// ---------------------------------------------------------------- pin map

static const uint8_t EMIT_PIN = 11;  // INPUT = emitters off, OUTPUT+HIGH = on

static const uint8_t LS_LEFT_PIN     = 12;  // DN1, far left
static const uint8_t LS_MIDLEFT_PIN  = A0;  // DN2
static const uint8_t LS_MIDDLE_PIN   = A2;  // DN3
static const uint8_t LS_MIDRIGHT_PIN = A3;  // DN4
static const uint8_t LS_RIGHT_PIN    = A4;  // DN5, far right

static const uint8_t L_PWM_PIN = 10;
static const uint8_t L_DIR_PIN = 16;
static const uint8_t R_PWM_PIN = 9;
static const uint8_t R_DIR_PIN = 15;

#define FWD LOW
#define REV HIGH

// ------------------------------------------------------------- sensing

static const uint8_t  NUM_SENSORS            = 5;
static const uint16_t SENSOR_TIMEOUT_US      = 2500;  // discharge give-up
static const uint16_t NORMALISED_MAX         = 1000;  // calibrated full scale
static const uint16_t LINE_PRESENT_THRESHOLD = 200;   // normalised units
static const uint16_t JUNCTION_THRESHOLD     = 600;   // far sensor on line

// Narrowest light-to-dark span, in microseconds, that counts as a usable
// calibration. Below this the sensor never saw both surfaces, the calibration
// is rejected outright, and readings fall back to the raw timeout range.
static const uint16_t MIN_CALIBRATION_SPAN = 20;

// ------------------------------------------------------------- driving

static const float BASE_SPEED_PWM = 30.0f;  // was BiasPWM
static const float MAX_PWM        = 255.0f;

// Kp derivation: the original controller was
//   LeftPWM = BiasPWM + MaxTurnPWM * W,  W in [-2, +2]
// linePosition() returns [-1, +1], so Kp = 2 * MaxTurnPWM = 40 reproduces the
// original proportional response exactly at full deflection.
static const float PID_KP = 40.0f;
static const float PID_KI = 0.0f;  // start at zero, tune on hardware
static const float PID_KD = 0.0f;  // start at zero, tune on hardware
static const float PID_INTEGRAL_LIMIT = 50.0f;

// Ki and Kd default to zero deliberately, so the controller degrades exactly
// to the proportional behaviour that is known to work on the course.

// ------------------------------------------------------------- timings

// A turn ignores the line it started on for this long, so it cannot exit
// immediately on the sensors it entered with.
static const uint32_t TURN_SETTLE_MS        = 150;
static const uint32_t TURN_TIMEOUT_MS       = 1200;
static const uint32_t REDISCOVER_TIMEOUT_MS = 2000;
static const uint32_t CALIBRATION_MS        = 3000;

// TURN_SETTLE_MS must stay well below TURN_TIMEOUT_MS, or a turn can only
// ever end on the timeout and never on reacquiring the line.

// ------------------------------------------------------------- confirmation

// A junction must be seen this many times running before it is acted on, so
// one noisy frame on a far sensor cannot throw the robot into a turn.
static const uint8_t JUNCTION_CONFIRM_SAMPLES = 2;

// Joining the line needs this many sightings, each separated by at least the
// debounce interval: the start box is crossed at an angle and one brush of
// the line is not enough to commit to following it.
static const uint8_t  JOIN_HITS_REQUIRED = 2;
static const uint32_t JOIN_DEBOUNCE_MS   = 200;

// ------------------------------------------------------------- bench mode

// Set to 1 to build the diagnostic firmware that the procedure in TUNING.md
// assumes: it opens the serial port, walks the motors through a polarity
// check, runs a calibration sweep and prints the resulting per-sensor bounds,
// then stays stopped. Set back to 0 for normal operation.
//
// The normal build carries none of it. Note that the cost is not memory: the
// 32U4 links its USB serial stack either way, so the bench build measures
// 6986 bytes of flash and 240 bytes of RAM against 8670 and 243 for the
// normal one, the difference being the navigator rather than Serial. The
// reason to gate it is behavioural. Bench mode drives the motors on a fixed
// script and blocks while it does so, which is safe on a stand and dangerous
// on a course.
//
// It is the one place in this firmware where blocking is acceptable: a
// scripted sequence for a human to watch, not a control loop.
#define BENCH_MODE 0

static const long     BENCH_SERIAL_BAUD = 9600;
static const uint32_t BENCH_MOVE_MS     = 2000;  // how long each motor test runs

#endif
