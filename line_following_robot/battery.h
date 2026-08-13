#ifndef BATTERY_H
#define BATTERY_H

#include "config.h"

// Battery voltage sensing and demand compensation.
//
// Why this exists: the pack drains from roughly 6V fresh to roughly 4V flat.
// The motor driver hands the motors a fraction of the pack voltage, so the
// same PWM number produces steadily less torque as the pack empties. Gains
// tuned on a fresh battery are therefore wrong on a tired one, and the usual
// symptom is a robot that followed the course perfectly in the morning and
// slides straight through the corners in the afternoon.
//
// Compensation scales the demand by nominal/measured, so the torque the
// controller asks for stays roughly constant as the pack drains.
//
// HONESTY NOTE: none of the numbers below have been checked against real
// hardware. The divider ratio, the nominal voltage and the low-battery
// threshold are all taken from documentation or from reasonable assumption
// and must be confirmed on the first bench session.
//
// The constants are declared here for now so this header stands alone. They
// belong in config.h alongside every other tuning value once this is wired
// into the sketch.

// ---------------------------------------------------------------- pin map

// The 3Pi+ 32U4 senses the battery through a resistor divider on A1. Note
// that A1 is not used by anything else in this firmware, so unlike the
// encoder pins there is no conflict to resolve.
static const uint8_t BATTERY_SENSE_PIN = A1;

// --------------------------------------------------------------- scaling

// The ADC reference and full-scale count for the 32U4 running at 5V.
static const long BATTERY_ADC_REFERENCE_MV = 5000L;
static const long BATTERY_ADC_FULL_SCALE   = 1023L;

// The divider ratio, expressed as a fraction so the arithmetic stays in
// integers: the pin sees 2/3 of the pack voltage, so the pack voltage is
// 3/2 of the pin voltage. This gives Pololu's own expression,
//   analogRead(A1) * 5000L * 3 / 2 / 1023
// and a full-scale reading of 7500 mV.
//
// The ratio is taken from Pololu's documentation for the 3Pi+ 32U4 and has
// NOT been verified. Put a multimeter across the pack on the first bench
// session, compare it against readMillivolts(), and correct these two
// numbers if they disagree. Everything else in this file is built on them,
// so an error here silently corrupts the compensation factor too.
static const long BATTERY_DIVIDER_NUMERATOR   = 3L;
static const long BATTERY_DIVIDER_DENOMINATOR = 2L;

// ------------------------------------------------------------- thresholds

// The pack voltage the PID gains are assumed to have been tuned at. This is
// the reference the compensation works back towards, so it must match the
// voltage the robot was actually tuned on, not a datasheet figure. 5000 mV
// is a guess at a half-drained alkaline pack and should be replaced with the
// measured voltage from the tuning session.
static const uint16_t BATTERY_NOMINAL_MV = 5000;

// Below this the pack is tired enough that behaviour is no longer
// trustworthy even with compensation, because the motors are approaching the
// point where more PWM buys no more torque. A warning, not a shutdown.
static const uint16_t BATTERY_LOW_MV = 4000;

// Anything below this is not a battery. A disconnected divider, an ADC that
// never got configured, or a pin held low all read near zero, and dividing
// by such a value would produce an enormous compensation factor.
static const uint16_t BATTERY_MIN_VALID_MV = 1000;

// Hard bounds on the compensation factor. This is the safety property of
// this whole file: an unclamped nominal/measured on a misread or
// disconnected ADC commands full power to both motors, which on a bench with
// the robot in someone's hand is how fingers get hurt. The clamp means the
// worst a bad reading can do is double the demand, which the PWM clamp in
// motors.h then limits again.
static const float BATTERY_COMP_MIN = 0.5f;
static const float BATTERY_COMP_MAX = 2.0f;

// ------------------------------------------------------------- smoothing

// A single ADC sample taken while the motors are surging is noisy: the pack
// sags under load and recovers, so consecutive samples can differ by several
// hundred millivolts. The running average is over this many samples, which
// at loop rate is a fraction of a second and far shorter than the timescale
// on which a battery actually drains.
static const uint8_t BATTERY_SMOOTHING_SAMPLES = 8;

class Battery_c {

  private:

    uint16_t samples[BATTERY_SMOOTHING_SAMPLES];
    uint8_t  next_slot = 0;
    uint32_t running_total = 0;  // kept incrementally, so no loop per read
    uint16_t smoothed_mv = 0;

    // One unsmoothed conversion, in millivolts.
    uint16_t sampleOnce() const {

      long counts = (long)analogRead(BATTERY_SENSE_PIN);

      if (counts < 0) counts = 0;  // an ADC never returns negative, but a
                                   // stub or a future refactor might

      long mv = counts * BATTERY_ADC_REFERENCE_MV
                * BATTERY_DIVIDER_NUMERATOR
                / BATTERY_DIVIDER_DENOMINATOR
                / BATTERY_ADC_FULL_SCALE;

      return (uint16_t)mv;

    }

    // Folds one fresh sample into the running average and returns it.
    uint16_t pushSample(uint16_t mv) {

      running_total -= samples[next_slot];
      samples[next_slot] = mv;
      running_total += mv;

      next_slot++;
      if (next_slot >= BATTERY_SMOOTHING_SAMPLES) next_slot = 0;

      smoothed_mv = (uint16_t)(running_total / BATTERY_SMOOTHING_SAMPLES);
      return smoothed_mv;

    }

  public:

    Battery_c() {

      for (uint8_t i = 0; i < BATTERY_SMOOTHING_SAMPLES; i++) samples[i] = 0;

    }

    // Primes the average with the pack's current voltage. Without this the
    // first several readings are dragged down by the zeroes the buffer
    // started with, and the robot would briefly believe the battery was flat
    // and over-compensate at exactly the moment it starts moving.
    void begin() {

      pinMode(BATTERY_SENSE_PIN, INPUT);

      uint16_t mv = sampleOnce();

      running_total = 0;
      for (uint8_t i = 0; i < BATTERY_SMOOTHING_SAMPLES; i++) {
        samples[i] = mv;
        running_total += mv;
      }

      next_slot = 0;
      smoothed_mv = mv;

    }

    // Takes one conversion, folds it into the average, and returns the
    // smoothed value. Call this once per control iteration. It is cheap, but
    // it is not free: analogRead blocks for about 100 us on the 32U4.
    uint16_t readMillivolts() {

      return pushSample(sampleOnce());

    }

    // The last smoothed value, without taking a new conversion. Useful when
    // more than one decision in an iteration needs the voltage, so the loop
    // does not pay for the ADC twice.
    uint16_t millivolts() const { return smoothed_mv; }

    // Multiply the motor demand by this to hold torque roughly constant as
    // the pack drains. Always inside [BATTERY_COMP_MIN, BATTERY_COMP_MAX],
    // and always exactly 1.0 when the reading is not believable, so a broken
    // divider degrades to no compensation rather than to full power.
    float compensationFactor() {

      // Guards the division and rejects garbage in one test. Zero is the
      // obvious case; anything under BATTERY_MIN_VALID_MV is equally not a
      // battery and is treated the same way.
      if (smoothed_mv < BATTERY_MIN_VALID_MV) return 1.0f;

      float factor = (float)BATTERY_NOMINAL_MV / (float)smoothed_mv;

      if (factor < BATTERY_COMP_MIN) factor = BATTERY_COMP_MIN;
      else if (factor > BATTERY_COMP_MAX) factor = BATTERY_COMP_MAX;

      return factor;

    }

    // A warning that the pack should be swapped. Note that a reading of zero
    // also reports low, deliberately: whether the pack is flat or the sense
    // line has fallen off, the right response from a human is the same.
    bool isLow() const {

      return smoothed_mv < BATTERY_LOW_MV;

    }

};

#endif
