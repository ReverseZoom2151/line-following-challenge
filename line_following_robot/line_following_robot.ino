// Line following robot for the Pololu 3Pi+ 32U4.
//
// The sketch is wiring only. Every decision belongs to the navigator, and all
// pins and tuning constants live in config.h.
//
// On power-up the robot offers a short window to choose what to do, so
// selecting a mode no longer means editing a file and reflashing:
//
//   A, or nothing        run the course
//   B                    bench diagnostics, for the procedure in TUNING.md
//   C                    discard the stored calibration and sweep again
//
// A calibration accepted on a previous run is restored from EEPROM, so a good
// sweep survives a power cycle and the robot goes straight to looking for the
// line.

#include "config.h"
#include "linesensors.h"
#include "motors.h"
#include "navigator.h"
#include "battery.h"
#include "buttons.h"
#include "encoders.h"
#include "calibration_store.h"

LineSensor_c line_sensors;
Motors_c motors;
Navigator_c navigator;
Battery_c battery;
Buttons_c buttons;
Encoders_c encoders;
CalibrationStore_c calibration_store;

// True once the bench sequence has run, so loop() holds the robot stopped
// instead of falling through into the state machine.
bool bench_finished = false;

// Tracks the transition out of Calibrate, so a fresh sweep is written to
// EEPROM exactly once, at the moment it is accepted.
bool calibration_saved = false;
NavState previous_state = NavState::Calibrate;

// How often the battery is sampled. The ADC read is cheap but not free, and
// a pack does not drain measurably between two passes of a loop running at
// sensor speed.
uint32_t last_battery_ms = 0;

// ---------------------------------------------------------------- helpers

// Restores a stored calibration. Returns false if there is no record, if it
// fails validation, or if the bounds it holds are too narrow to be usable,
// in which case the caller should sweep instead.
bool restoreCalibration() {

  uint16_t min_raw[NUM_SENSORS];
  uint16_t max_raw[NUM_SENSORS];

  if (!calibration_store.load(min_raw, max_raw, NUM_SENSORS)) return false;

  return line_sensors.applyCalibration(min_raw, max_raw, NUM_SENSORS);

}

// Writes the calibration the robot just swept to EEPROM. Only called when the
// sensor accepted it, so a rejected sweep never overwrites a good record.
void storeCalibration() {

  uint16_t min_raw[NUM_SENSORS];
  uint16_t max_raw[NUM_SENSORS];

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    min_raw[i] = line_sensors.calibrationMin(i);
    max_raw[i] = line_sensors.calibrationMax(i);
  }

  calibration_store.save(min_raw, max_raw, NUM_SENSORS);

}

// Offers the mode choice for BOOT_SELECT_MS. Polls rather than blocking on a
// press: the window has to close on its own, or a robot started away from a
// keyboard would wait forever.
uint8_t chooseBootMode() {

  const uint32_t opened = millis();

  while (millis() - opened < BOOT_SELECT_MS) {

    buttons.update(millis());

    if (buttons.wasPressed(BUTTON_B)) return BUTTON_B;
    if (buttons.wasPressed(BUTTON_C)) return BUTTON_C;
    if (buttons.wasPressed(BUTTON_A)) return BUTTON_A;

  }

  return BUTTON_NONE;

}

// Scripted bench sequence: motor polarity, encoder counts, then a calibration
// sweep and a report of what it recorded. Blocking throughout, because a
// person is watching each step and needs time to see it. This is the one
// place in the firmware where blocking is acceptable.
void runBenchCheck() {

  Serial.println(F("=== bench check ==="));
  Serial.print(F("battery mV: "));
  Serial.println(battery.readMillivolts());

  if (battery.isLow()) Serial.println(F("WARNING: battery is low"));

  // ---- step 2 of TUNING.md: motor polarity, and what the encoders saw
  encoders.reset();
  Serial.println(F("driveStraight: both wheels should turn FORWARDS"));
  motors.driveStraight(BASE_SPEED_PWM);
  delay(BENCH_MOVE_MS);
  motors.stop();
  reportEncoders();
  delay(500);

  encoders.reset();
  Serial.println(F("spinLeft: robot should turn ANTICLOCKWISE from above"));
  motors.spinLeft(BASE_SPEED_PWM);
  delay(BENCH_MOVE_MS);
  motors.stop();
  reportEncoders();
  delay(500);

  encoders.reset();
  Serial.println(F("spinRight: robot should turn CLOCKWISE from above"));
  motors.spinRight(BASE_SPEED_PWM);
  delay(BENCH_MOVE_MS);
  motors.stop();
  reportEncoders();
  delay(500);

  // ---- step 3 of TUNING.md: calibration sweep
  Serial.println(F("calibration: sweep all five sensors across the line now"));
  line_sensors.beginCalibration();

  const uint32_t started = millis();
  while (millis() - started < CALIBRATION_MS) {
    SensorSnapshot snapshot;
    line_sensors.readAll(snapshot);
    line_sensors.updateCalibration(snapshot);
  }

  line_sensors.endCalibration();
  line_sensors.reportCalibration();

  if (line_sensors.isCalibrated()) {
    storeCalibration();
    Serial.println(F("calibration stored to EEPROM"));
  } else {
    Serial.println(F("calibration NOT stored: the sweep was rejected"));
  }

  Serial.println(F("=== bench check complete, motors stopped ==="));

}

// Prints both encoder counts. Expect the right channel to read zero on real
// hardware: see the pin conflict documented at the top of encoders.h. This
// exists so that conflict shows up as a number on a bench rather than as a
// robot that turns the wrong amount on a course.
void reportEncoders() {

  Serial.print(F("  encoders L,R: "));
  Serial.print(encoders.leftCount());
  Serial.print(',');
  Serial.println(encoders.rightCount());

}

// ---------------------------------------------------------------- sketch

void setup() {

  motors.begin();
  line_sensors.begin();
  buttons.begin();
  battery.begin();
  encoders.begin();

  Serial.begin(BENCH_SERIAL_BAUD);

  // The 32U4 enumerates its USB serial port after boot, so an immediate print
  // is lost. Wait for the host to open the port, but not forever: a robot run
  // from a battery has no host and must not hang here.
  const uint32_t waited = millis();
  while (!Serial && (millis() - waited) < SERIAL_WAIT_MS) {}

  const uint8_t chosen = chooseBootMode();

  if (chosen == BUTTON_B) {
    runBenchCheck();
    bench_finished = true;
    return;
  }

  if (chosen == BUTTON_C) {
    // Forget the stored record so this run sweeps and writes a new one.
    calibration_store.clear();
  }

  const bool restored = (chosen != BUTTON_C) && restoreCalibration();

  if (restored) {
    // Nothing to save: the record it came from is already in EEPROM.
    calibration_saved = true;
    navigator.beginCalibrated(&line_sensors, &motors);
  } else {
    navigator.begin(&line_sensors, &motors);
  }

  previous_state = navigator.state();

}

void loop() {

  if (bench_finished) {
    motors.stop();
    return;
  }

  buttons.update(millis());

  // Scale the motor demand for the state of the pack, so a gain tuned on a
  // fresh battery still means the same thing on a flat one.
  if (millis() - last_battery_ms >= BATTERY_SAMPLE_MS) {
    battery.readMillivolts();
    motors.setCompensation(battery.compensationFactor());
    last_battery_ms = millis();
  }

  // one snapshot of all five sensors, taken at one instant, so every
  // decision the navigator makes this iteration agrees with every other
  SensorSnapshot snapshot;
  line_sensors.readAll(snapshot);

  navigator.update(millis(), snapshot);

  // Persist a sweep the moment it is accepted, once. Doing it here rather
  // than inside the navigator keeps the state machine ignorant of storage.
  const NavState now_state = navigator.state();

  if (!calibration_saved && previous_state == NavState::Calibrate &&
      now_state != NavState::Calibrate) {

    if (line_sensors.isCalibrated()) storeCalibration();
    calibration_saved = true;

  }

  previous_state = now_state;

}
