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

int main() {
  printf("sensors\n");

  {
    TEST("setup leaves the emitter off and the sensors as inputs");
    mockReset();
    LineSensor_c sensors;
    sensors.setupAllLineSensors();

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
    sensors.setupAllLineSensors();
    mockSetDischarge(LS_MIDDLE_PIN, 500);

    unsigned long reading = sensors.readLineSensor(2);

    CHECK_EQ(reading, 500);
    CHECK(!mockWatchdogTripped);
  }

  {
    TEST("a darker surface reads as a longer discharge time");
    mockReset();
    LineSensor_c sensors;
    sensors.setupAllLineSensors();
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
    sensors.setupAllLineSensors();
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
    sensors.setupAllLineSensors();
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
    sensors.setupAllLineSensors();

    CHECK_EQ(sensors.readLineSensor(9), 0);
    CHECK_EQ(sensors.readLineSensor(-1), 0);
    CHECK(sensors.readLineSensor(5) < 1000);
  }

  return testSummary("sensors");
}
