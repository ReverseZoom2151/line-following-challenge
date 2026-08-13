// Line following robot for the Pololu 3Pi+ 32U4.
//
// The sketch is wiring only: one sensor read per iteration, handed to the
// navigator, which owns every decision. All pins and tuning constants live in
// config.h, and nothing here blocks, so the control loop runs as fast as the
// sensors can be read.

#include "config.h"
#include "linesensors.h"
#include "motors.h"
#include "navigator.h"

LineSensor_c line_sensors;
Motors_c motors;
Navigator_c navigator;

void setup() {

  motors.begin();
  line_sensors.begin();
  navigator.begin(&line_sensors, &motors);

}

void loop() {

  // one snapshot of all five sensors, taken at one instant, so every
  // decision the navigator makes this iteration agrees with every other
  SensorSnapshot snapshot;
  line_sensors.readAll(snapshot);

  navigator.update(millis(), snapshot);

}
