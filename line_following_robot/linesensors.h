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

    // The bounds are initialised here, not just in beginCalibration(). The
    // accessors and reportCalibration() read them whether or not a sweep has
    // run, and reading uninitialised memory to print it over serial would be
    // a confusing way to start a bench session.
    LineSensor_c() {
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        min_raw[i] = 0;
        max_raw[i] = 0;
      }
    }

    // Maps a raw discharge time onto the calibrated 0..NORMALISED_MAX scale.
    // Exposed alongside the calibration bounds so a bench session can check
    // what a given reading will actually become, rather than inferring it.
    uint16_t normalise(uint8_t i, uint16_t raw) const {
      return (i < NUM_SENSORS) ? normaliseOne(i, raw) : 0;
    }

    // Calibration bounds, exposed so the sweep can actually be checked on a
    // bench. Without these the tuning procedure asks the operator to inspect
    // numbers that nothing can print. A sensor whose minimum and maximum sit
    // close together never saw both the line and the surface beside it, which
    // is the single most common reason a calibrated robot steers badly.
    uint16_t calibrationMin(uint8_t i) const {
      return (i < NUM_SENSORS) ? min_raw[i] : 0;
    }

    uint16_t calibrationMax(uint8_t i) const {
      return (i < NUM_SENSORS) ? max_raw[i] : 0;
    }

    // Prints one line per sensor: index, minimum, maximum, span. Intended for
    // a bench session over the serial monitor, never from the control loop:
    // it is slow and would blind the robot while it transmits.
    void reportCalibration() const {

      Serial.println(F("sensor,min_us,max_us,span_us"));

      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.print(i);
        Serial.print(',');
        Serial.print(min_raw[i]);
        Serial.print(',');
        Serial.print(max_raw[i]);
        Serial.print(',');
        Serial.println((uint16_t)(max_raw[i] - min_raw[i]));
      }

      if (!calibrated) Serial.println(F("WARNING: calibration was rejected"));

    }

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

    // Restores bounds recorded on a previous run, so a good sweep survives a
    // power cycle. The bounds are put through exactly the same span check a
    // fresh sweep faces: a stored record is not more trustworthy than a live
    // one just because it came out of EEPROM, and a narrow span produces a
    // robot that cannot tell floor from line whatever its origin.
    //
    // Returns false and leaves the sensor uncalibrated if the bounds are not
    // usable, so a caller can fall back to sweeping again.
    bool applyCalibration(const uint16_t *min_in, const uint16_t *max_in, uint8_t count) {

      if (min_in == 0 || max_in == 0 || count != NUM_SENSORS) return false;

      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        min_raw[i] = min_in[i];
        max_raw[i] = max_in[i];
      }

      endCalibration();
      return calibrated;

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

    // ------------------------------------------------------ interpretation

    // Weighted mean of all five sensors, mapped to [-1, +1], where negative
    // means the line lies left of centre. Using all five rather than only
    // DN2 and DN4 keeps the estimate meaningful once the line drifts out
    // towards a far sensor, instead of saturating.
    float linePosition(const SensorSnapshot &s, bool &lineFound) const {

      static const int8_t weights[NUM_SENSORS] = { -2, -1, 0, 1, 2 };

      int32_t total = 0;
      int32_t weighted = 0;

      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        total += (int32_t)s.normalised[i];
        weighted += (int32_t)weights[i] * (int32_t)s.normalised[i];
      }

      // The presence test is on the STRONGEST sensor, not on the sum. Testing
      // the sum meant five sensors reading a little each could add up to a
      // line that was not there: on an uncalibrated robot, bare floor
      // normalises to roughly 72 per sensor, and 5 x 72 = 360 cleared a
      // threshold of 200. The robot then believed it was centred on a line
      // while looking at an empty floor. Using the maximum also makes this
      // agree with onLine(), which tests sensors individually and was never
      // fooled; the two disagreeing is what made the failure silent.
      //
      // This is still the divide-by-zero guard: total cannot be below the
      // maximum, so a passing maximum guarantees a non-zero total.
      if (activation(s) < LINE_PRESENT_THRESHOLD) {
        lineFound = false;
        return 0.0f;
      }

      lineFound = true;

      // weighted / total lies in [-2, +2]; halving maps it onto [-1, +1]
      return ((float)weighted / (float)total) * 0.5f;

    }

    // Strongest single response in the snapshot.
    uint16_t activation(const SensorSnapshot &s) const {

      uint16_t peak = 0;

      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (s.normalised[i] > peak) peak = s.normalised[i];
      }

      return peak;

    }

    // True when the line sits under the central three sensors, DN2 to DN4.
    bool onLine(const SensorSnapshot &s) const {

      for (uint8_t i = 1; i < NUM_SENSORS - 1; i++) {
        if (s.normalised[i] >= LINE_PRESENT_THRESHOLD) return true;
      }

      return false;

    }

    bool farLeftActive(const SensorSnapshot &s) const {
      return s.normalised[0] >= JUNCTION_THRESHOLD;
    }

    bool farRightActive(const SensorSnapshot &s) const {
      return s.normalised[NUM_SENSORS - 1] >= JUNCTION_THRESHOLD;
    }

};

#endif
