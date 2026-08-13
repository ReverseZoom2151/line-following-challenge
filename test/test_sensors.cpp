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

// Builds a snapshot directly from normalised values, so the interpretation
// tests are not entangled with pin timing.
static SensorSnapshot snap(uint16_t a, uint16_t b, uint16_t c, uint16_t d,
                           uint16_t e) {
  SensorSnapshot s = {};
  const uint16_t v[NUM_SENSORS] = { a, b, c, d, e };
  for (int i = 0; i < NUM_SENSORS; i++) {
    s.normalised[i] = v[i];
    s.raw[i] = v[i];
    s.timedOut[i] = false;
  }
  s.timestampMicros = 0;
  return s;
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

  {
    TEST("linePosition is centred, signed and bounded");
    LineSensor_c sensors;
    bool found = false;

    SensorSnapshot centred = snap(0, 0, 900, 0, 0);
    CHECK_NEAR(sensors.linePosition(centred, found), 0.0, 0.001);
    CHECK(found);

    // symmetric straddle of the centre is still centred
    SensorSnapshot straddle = snap(0, 500, 800, 500, 0);
    CHECK_NEAR(sensors.linePosition(straddle, found), 0.0, 0.001);

    SensorSnapshot left = snap(0, 900, 200, 0, 0);
    float lp = sensors.linePosition(left, found);
    CHECK(found);
    CHECK(lp < 0.0);
    CHECK(lp >= -1.0);

    SensorSnapshot right = snap(0, 0, 200, 900, 0);
    float rp = sensors.linePosition(right, found);
    CHECK(rp > 0.0);
    CHECK(rp <= 1.0);

    // mirrored inputs give mirrored outputs
    CHECK_NEAR(lp, -rp, 0.001);

    // the extremes saturate exactly at the ends of the range
    SensorSnapshot hardLeft = snap(1000, 0, 0, 0, 0);
    CHECK_NEAR(sensors.linePosition(hardLeft, found), -1.0, 0.001);
    SensorSnapshot hardRight = snap(0, 0, 0, 0, 1000);
    CHECK_NEAR(sensors.linePosition(hardRight, found), 1.0, 0.001);
  }

  {
    TEST("linePosition reports no line rather than dividing by zero");
    LineSensor_c sensors;
    bool found = true;

    SensorSnapshot blank = snap(0, 0, 0, 0, 0);
    float p = sensors.linePosition(blank, found);

    CHECK(!found);
    CHECK_NEAR(p, 0.0, 0.001);
    CHECK(p == p);  // not NaN

    // just under the presence threshold also counts as no line
    found = true;
    SensorSnapshot faint = snap(10, 10, 10, 10, 10);
    CHECK_NEAR(sensors.linePosition(faint, found), 0.0, 0.001);
    CHECK(!found);
  }

  {
    TEST("activation, onLine and the far sensor helpers");
    LineSensor_c sensors;

    SensorSnapshot s = snap(700, 100, 950, 100, 50);
    CHECK_EQ(sensors.activation(s), 950);
    CHECK(sensors.onLine(s));
    CHECK(sensors.farLeftActive(s));
    CHECK(!sensors.farRightActive(s));

    // a far sensor alone is a junction cue, not the line being followed
    SensorSnapshot edge = snap(0, 0, 0, 0, 800);
    CHECK(!sensors.onLine(edge));
    CHECK(sensors.farRightActive(edge));
    CHECK(!sensors.farLeftActive(edge));

    SensorSnapshot cross = snap(900, 300, 900, 300, 900);
    CHECK(sensors.farLeftActive(cross));
    CHECK(sensors.farRightActive(cross));
    CHECK(sensors.onLine(cross));
  }

  return testSummary("sensors");
}
