// Host tests for linesensors.h.
//
// Two of these document defects rather than desired behaviour. They are
// written as passing assertions against what the firmware does today, so the
// suite is green now and the assertion can be inverted in the commit that
// fixes the defect. That makes each fix demonstrably a change in behaviour
// rather than a change nobody can observe.

#include "arduino_stub.h"
#include "test_harness.h"

#include "linesensors.h"

// The five sensor pins in DN1..DN5 order, for tests that sweep all of them.
static int sensorPin(int i) {
  static const int pins[NUM_SENSORS] = { LS_LEFT_PIN, LS_MIDLEFT_PIN,
                                         LS_MIDDLE_PIN, LS_MIDRIGHT_PIN,
                                         LS_RIGHT_PIN };
  return pins[i];
}

int main() {
  printf("sensors\n");

  {
    TEST("begin leaves the emitter off and the sensors as inputs");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();

    CHECK_EQ(mockPins[EMIT_PIN].mode, INPUT);
    CHECK_EQ(mockPins[LS_LEFT_PIN].mode, INPUT);
    CHECK_EQ(mockPins[LS_MIDLEFT_PIN].mode, INPUT);
    CHECK_EQ(mockPins[LS_MIDDLE_PIN].mode, INPUT);
    CHECK_EQ(mockPins[LS_MIDRIGHT_PIN].mode, INPUT);
    CHECK_EQ(mockPins[LS_RIGHT_PIN].mode, INPUT);
  }

  {
    TEST("a normal read returns the discharge time");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();
    mockSetDischarge(LS_MIDDLE_PIN, 500);

    unsigned long reading = sensors.readLineSensor(2);

    CHECK_EQ(reading, 500);
    CHECK(!mockWatchdogTripped);
  }

  {
    TEST("a darker surface reads as a longer discharge time");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();
    mockSetDischarge(LS_MIDLEFT_PIN, 200);
    mockSetDischarge(LS_MIDRIGHT_PIN, 1400);

    unsigned long bright = sensors.readLineSensor(1);
    unsigned long dark = sensors.readLineSensor(3);

    CHECK(dark > bright);
    CHECK_EQ(bright, 200);
    CHECK_EQ(dark, 1400);
  }

  {
    TEST("the emitter is switched back off after a read");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();
    mockSetDischarge(LS_LEFT_PIN, 300);

    sensors.readLineSensor(0);

    CHECK_EQ(mockPins[EMIT_PIN].mode, INPUT);
  }

  {
    // A sensor over a black surface, or a broken connection, never pulls the
    // pin low. The read must give up rather than spin until the robot is
    // reset. The stub's watchdog standing untripped is the proof.
    TEST("a sensor that never discharges times out instead of hanging");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();
    mockSetNeverDischarges(LS_MIDDLE_PIN);

    unsigned long reading = sensors.readLineSensor(2);

    CHECK(!mockWatchdogTripped);
    CHECK(reading >= 2500);
    CHECK(reading <= 2600);
  }

  {
    // An out of range index must report as "nothing seen", not as the
    // unsigned wrap of -1, which exceeded every detection threshold.
    TEST("an out of range index returns zero, below every threshold");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();

    CHECK_EQ(sensors.readLineSensor(9), 0);
    CHECK_EQ(sensors.readLineSensor(-1), 0);
    CHECK(sensors.readLineSensor(5) < 1000);
  }

  {
    TEST("readAll returns all five sensors from one pass");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();
    mockSetDischarge(LS_LEFT_PIN, 400);
    mockSetDischarge(LS_MIDLEFT_PIN, 500);
    mockSetDischarge(LS_MIDDLE_PIN, 600);
    mockSetDischarge(LS_MIDRIGHT_PIN, 700);
    mockSetDischarge(LS_RIGHT_PIN, 800);

    SensorSnapshot s;
    uint32_t before = micros();
    sensors.readAll(s);
    uint32_t cost = micros() - before;

    CHECK(!mockWatchdogTripped);
    CHECK_NEAR(s.raw[0], 400, 20);
    CHECK_NEAR(s.raw[1], 500, 20);
    CHECK_NEAR(s.raw[2], 600, 20);
    CHECK_NEAR(s.raw[3], 700, 20);
    CHECK_NEAR(s.raw[4], 800, 20);

    for (int i = 0; i < NUM_SENSORS; i++) CHECK(!s.timedOut[i]);

    // one pass costs about the slowest sensor, not the sum of all five,
    // which is what reading them one at a time used to cost
    CHECK(cost < 1500);
    CHECK(s.timestampMicros >= before);
  }

  {
    TEST("readAll flags a sensor that never discharges and keeps the rest");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();
    mockSetDischarge(LS_LEFT_PIN, 300);
    mockSetNeverDischarges(LS_MIDDLE_PIN);
    mockSetDischarge(LS_RIGHT_PIN, 350);

    SensorSnapshot s;
    sensors.readAll(s);

    CHECK(!mockWatchdogTripped);
    CHECK(s.timedOut[2]);
    CHECK_EQ(s.raw[2], SENSOR_TIMEOUT_US);
    CHECK(!s.timedOut[0]);
    CHECK(!s.timedOut[4]);
    CHECK_NEAR(s.raw[0], 300, 20);
    CHECK_NEAR(s.raw[4], 350, 20);
  }

  {
    TEST("readAll switches the emitter back off");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();
    mockSetDischarge(LS_MIDDLE_PIN, 200);

    SensorSnapshot s;
    sensors.readAll(s);

    CHECK_EQ(mockPins[EMIT_PIN].mode, INPUT);
  }

  {
    TEST("normalisation falls back to the full range before calibration");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();
    mockSetDischarge(LS_MIDDLE_PIN, 1250);  // half of SENSOR_TIMEOUT_US

    SensorSnapshot s;
    sensors.readAll(s);

    CHECK(!sensors.isCalibrated());
    CHECK_NEAR(s.normalised[2], 500, 20);
  }

  {
    TEST("calibration rescales each sensor onto its own span");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();

    // sweeps a light surface then a dark one across all five
    sensors.beginCalibration();
    for (int i = 0; i < NUM_SENSORS; i++) mockSetDischarge(sensorPin(i), 200);
    SensorSnapshot s;
    sensors.readAll(s);
    sensors.updateCalibration(s);
    for (int i = 0; i < NUM_SENSORS; i++) mockSetDischarge(sensorPin(i), 1200);
    sensors.readAll(s);
    sensors.updateCalibration(s);
    sensors.endCalibration();

    CHECK(sensors.isCalibrated());

    // mid-span reads as mid-scale, and the extremes saturate cleanly
    for (int i = 0; i < NUM_SENSORS; i++) mockSetDischarge(sensorPin(i), 700);
    sensors.readAll(s);
    CHECK_NEAR(s.normalised[0], 500, 40);
    CHECK_NEAR(s.normalised[4], 500, 40);

    for (int i = 0; i < NUM_SENSORS; i++) mockSetDischarge(sensorPin(i), 100);
    sensors.readAll(s);
    CHECK_EQ(s.normalised[2], 0);

    for (int i = 0; i < NUM_SENSORS; i++) mockSetDischarge(sensorPin(i), 2000);
    sensors.readAll(s);
    CHECK_EQ(s.normalised[2], NORMALISED_MAX);
  }

  {
    TEST("a degenerate calibration is rejected and does not divide by zero");
    mockReset();
    LineSensor_c sensors;
    sensors.begin();

    // every sample identical: the sensors never saw both surfaces
    sensors.beginCalibration();
    for (int i = 0; i < NUM_SENSORS; i++) mockSetDischarge(sensorPin(i), 600);
    SensorSnapshot s;
    sensors.readAll(s);
    sensors.updateCalibration(s);
    sensors.readAll(s);
    sensors.updateCalibration(s);
    sensors.endCalibration();

    CHECK(!sensors.isCalibrated());

    sensors.readAll(s);
    for (int i = 0; i < NUM_SENSORS; i++) {
      CHECK(s.normalised[i] <= NORMALISED_MAX);
    }
  }

  return testSummary("sensors");
}
