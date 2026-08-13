// Line following robot for the Pololu 3Pi+ 32U4.
//
// The sketch is wiring only: one sensor read per iteration, handed to the
// navigator, which owns every decision. All pins and tuning constants live in
// config.h, and nothing here blocks, so the control loop runs as fast as the
// sensors can be read.
//
// Setting BENCH_MODE to 1 in config.h builds the diagnostic firmware instead,
// which is what the procedure in TUNING.md expects. See runBenchCheck below.

#include "config.h"
#include "linesensors.h"
#include "motors.h"
#include "navigator.h"

LineSensor_c line_sensors;
Motors_c motors;
Navigator_c navigator;

#if BENCH_MODE

// Scripted bench sequence: motor polarity, then a calibration sweep, then a
// report of what the sweep actually recorded. Blocking throughout, because a
// person is watching each step and needs time to see it. Never compiled into
// the normal firmware.
void runBenchCheck() {

  Serial.println(F("=== bench check ==="));

  // ---- step 2 of TUNING.md: motor polarity
  Serial.println(F("driveStraight: both wheels should turn FORWARDS"));
  motors.driveStraight(BASE_SPEED_PWM);
  delay(BENCH_MOVE_MS);
  motors.stop();
  delay(500);

  Serial.println(F("spinLeft: robot should turn ANTICLOCKWISE from above"));
  motors.spinLeft(BASE_SPEED_PWM);
  delay(BENCH_MOVE_MS);
  motors.stop();
  delay(500);

  Serial.println(F("spinRight: robot should turn CLOCKWISE from above"));
  motors.spinRight(BASE_SPEED_PWM);
  delay(BENCH_MOVE_MS);
  motors.stop();
  delay(500);

  // ---- step 3 of TUNING.md: calibration sweep
  Serial.println(F("calibration: sweep all five sensors across the line now"));
  line_sensors.beginCalibration();

  uint32_t started = millis();
  while (millis() - started < CALIBRATION_MS) {
    SensorSnapshot snapshot;
    line_sensors.readAll(snapshot);
    line_sensors.updateCalibration(snapshot);
  }

  line_sensors.endCalibration();
  line_sensors.reportCalibration();

  Serial.println(F("=== bench check complete, motors stopped ==="));

}

#endif

void setup() {

  motors.begin();
  line_sensors.begin();

#if BENCH_MODE

  Serial.begin(BENCH_SERIAL_BAUD);

  // The 32U4 enumerates its USB serial port after boot, so an immediate print
  // is lost. Wait for the host to open the port, but not forever: a robot run
  // from a battery has no host and must not hang here.
  uint32_t waited = millis();
  while (!Serial && (millis() - waited) < 3000) {}

  runBenchCheck();

#else

  navigator.begin(&line_sensors, &motors);

#endif

}

void loop() {

#if BENCH_MODE

  // The bench sequence has already run to completion in setup(). Hold the
  // motors stopped rather than falling through to the state machine.
  motors.stop();

#else

  // one snapshot of all five sensors, taken at one instant, so every
  // decision the navigator makes this iteration agrees with every other
  SensorSnapshot snapshot;
  line_sensors.readAll(snapshot);

  navigator.update(millis(), snapshot);

#endif

}
