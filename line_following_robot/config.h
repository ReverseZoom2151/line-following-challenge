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

#endif
