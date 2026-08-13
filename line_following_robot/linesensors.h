#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "config.h"

// One reading of all five sensors, taken at a single instant. Passing this
// around means every decision in a control iteration is made from the same
// snapshot, rather than from readings taken milliseconds apart.
struct SensorSnapshot {

  uint16_t raw[NUM_SENSORS];         // discharge time in us, clamped to SENSOR_TIMEOUT_US
  uint16_t normalised[NUM_SENSORS];  // 0..NORMALISED_MAX, valid once calibrated
  bool     timedOut[NUM_SENSORS];    // sensor never discharged within the budget
  uint32_t timestampMicros;          // when the discharge window opened

};

class LineSensor_c {

  private:

    uint8_t ls_pins[NUM_SENSORS] = { LS_LEFT_PIN, LS_MIDLEFT_PIN, LS_MIDDLE_PIN, LS_MIDRIGHT_PIN, LS_RIGHT_PIN }; // stores pin numbers for convenient access

    // Per-sensor calibration. The five sensors do not respond identically:
    // component tolerance and ride height mean the same surface reads
    // differently on each, so a single shared threshold biases the steering.
    uint16_t min_raw[NUM_SENSORS];
    uint16_t max_raw[NUM_SENSORS];
    bool calibrated = false;

    // A span narrower than this means the sensor never saw both surfaces, so
    // the calibration is not trustworthy.
    static const uint16_t MIN_CALIBRATION_SPAN = 20;

    // Maps one raw reading onto 0..NORMALISED_MAX, where larger means darker.
    // Falls back to the full timeout range until a calibration exists, so an
    // uncalibrated robot still steers rather than reading all zeroes.
    uint16_t normaliseOne(uint8_t i, uint16_t raw) const {

      uint16_t lo = calibrated ? min_raw[i] : 0;
      uint16_t hi = calibrated ? max_raw[i] : SENSOR_TIMEOUT_US;

      if (hi <= lo) return 0;  // divide-by-zero guard, never a NaN
      if (raw <= lo) return 0;
      if (raw >= hi) return NORMALISED_MAX;

      return (uint16_t)(((uint32_t)(raw - lo) * NORMALISED_MAX) / (uint32_t)(hi - lo));

    }

  public:

    LineSensor_c() {}

    void begin() {

      pinMode(EMIT_PIN, INPUT);
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        pinMode(ls_pins[i], INPUT);
      }

    }

    // Charges all five sensors, releases them together, then polls until each
    // has discharged or the budget expires. One emitter pulse, one instant,
    // and a bounded worst case of SENSOR_TIMEOUT_US for the whole set rather
    // than per sensor. Deliberately contains no Serial output: this is a
    // timing-critical loop and printing would distort every reading.
    void readAll(SensorSnapshot &out) {

      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);

      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        pinMode(ls_pins[i], OUTPUT);
        digitalWrite(ls_pins[i], HIGH);
      }

      delayMicroseconds(10);  // charges the capacitors

      bool done[NUM_SENSORS];

      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        out.raw[i] = 0;
        out.normalised[i] = 0;
        out.timedOut[i] = false;
        done[i] = false;
        pinMode(ls_pins[i], INPUT);  // releases, discharge begins
      }

      uint32_t start_time = micros();
      out.timestampMicros = start_time;

      uint8_t remaining = NUM_SENSORS;

      while (remaining > 0) {

        uint32_t elapsed = micros() - start_time;

        if (elapsed > SENSOR_TIMEOUT_US) elapsed = SENSOR_TIMEOUT_US;

        for (uint8_t i = 0; i < NUM_SENSORS; i++) {

          if (done[i]) continue;

          if (digitalRead(ls_pins[i]) == LOW) {

            out.raw[i] = (uint16_t)elapsed;
            done[i] = true;
            remaining--;

          } else if (elapsed >= SENSOR_TIMEOUT_US) {

            // never discharged: treat as the darkest reading the budget allows
            out.raw[i] = SENSOR_TIMEOUT_US;
            out.timedOut[i] = true;
            done[i] = true;
            remaining--;

          }

        }

      }

      pinMode(EMIT_PIN, INPUT);  // emitters always off on the way out

      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        out.normalised[i] = normaliseOne(i, out.raw[i]);
      }

    }

    // --------------------------------------------------------- calibration

    // Drive the robot across both the line and the background between
    // beginCalibration() and endCalibration(), feeding every snapshot to
    // updateCalibration().
    void beginCalibration() {

      calibrated = false;

      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        min_raw[i] = SENSOR_TIMEOUT_US;
        max_raw[i] = 0;
      }

    }

    void updateCalibration(const SensorSnapshot &s) {

      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (s.raw[i] < min_raw[i]) min_raw[i] = s.raw[i];
        if (s.raw[i] > max_raw[i]) max_raw[i] = s.raw[i];
      }

    }

    void endCalibration() {

      // only accepts the calibration if every sensor saw a usable spread,
      // otherwise the fallback range stays in use
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {

        if (max_raw[i] < min_raw[i] + MIN_CALIBRATION_SPAN) {
          calibrated = false;
          return;
        }

      }

      calibrated = true;

    }

    bool isCalibrated() const { return calibrated; }

    // reads a line sensor with error checking
    unsigned long readLineSensor(int number) {

      // prevents memory errors 
      if (number < 0 || number >= NUM_SENSORS) {

        // 0 is the safe error value here: it is an unsigned function, so -1
        // would wrap to the type maximum and read as a solid line detection
        return 0;

      }

      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);
      pinMode(ls_pins[number], OUTPUT);
      digitalWrite(ls_pins[number], HIGH);
      delayMicroseconds(10); 

      pinMode(ls_pins[number], INPUT); // only switches to input for measurement

      // gives up after SENSOR_TIMEOUT_US so a sensor that never discharges
      // (black surface, or a broken connection) cannot stall the robot
      unsigned long start_time = micros();

      while (digitalRead(ls_pins[number]) == HIGH) {

        if ((micros() - start_time) > SENSOR_TIMEOUT_US) break;

      }

      unsigned long end_time = micros();

      pinMode(EMIT_PIN, INPUT); 

      unsigned long elapsed_time = end_time - start_time;
      
      return elapsed_time; 

    } 

};

#endif
