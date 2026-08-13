// Host tests for battery.h.
//
// The property that matters most here is the clamp on the compensation
// factor. Compensation multiplies the motor demand, so an unclamped factor
// from a disconnected or misread ADC is a robot that goes to full power for
// no reason. That is tested explicitly, and swept across the whole ADC range
// rather than spot checked, because a safety property should not depend on
// which values someone happened to think of.
//
// Note that nothing here proves the divider ratio is right. No test on a
// host can: it only proves the arithmetic matches the ratio that was written
// down. The ratio itself needs a multimeter.

#include "arduino_stub.h"
#include "test_harness.h"

#include "battery.h"

// The ADC count that corresponds to a given pack voltage, inverting the
// conversion in battery.h. Used so the tests read in volts rather than in
// arbitrary counts.
static int countsFor(long millivolts) {
  return (int)(millivolts * BATTERY_ADC_FULL_SCALE * BATTERY_DIVIDER_DENOMINATOR
               / (BATTERY_ADC_REFERENCE_MV * BATTERY_DIVIDER_NUMERATOR));
}

// Runs enough conversions to flush the smoothing buffer, so the average
// reflects only the value currently on the pin.
static uint16_t settle(Battery_c &battery) {
  uint16_t mv = 0;
  for (uint8_t i = 0; i < BATTERY_SMOOTHING_SAMPLES; i++) mv = battery.readMillivolts();
  return mv;
}

int main() {
  printf("battery\n");

  {
    TEST("full scale reads the top of the divider range");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, 1023);

    Battery_c battery;
    battery.begin();

    // 1023 * 5000 * 3 / 2 / 1023 = 7500 mV
    CHECK_EQ(battery.readMillivolts(), 7500);
  }

  {
    TEST("a nominal pack reads its nominal voltage and needs no compensation");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, countsFor(BATTERY_NOMINAL_MV));

    Battery_c battery;
    battery.begin();
    settle(battery);

    CHECK_EQ(battery.millivolts(), BATTERY_NOMINAL_MV);
    CHECK_NEAR(battery.compensationFactor(), 1.0f, 0.01f);
    CHECK(!battery.isLow());
  }

  {
    TEST("a drained pack raises the demand and reports low");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, countsFor(3600));

    Battery_c battery;
    battery.begin();
    settle(battery);

    CHECK(battery.isLow());
    // 5000 / 3600 is about 1.39: the same torque needs about 39 per cent
    // more PWM than it did on a nominal pack.
    CHECK_NEAR(battery.compensationFactor(), 1.39f, 0.02f);
  }

  {
    // The whole point of the file. Without the clamp this is 5000/0 or a
    // factor of thirty, and the robot takes off across the room.
    TEST("a zero reading does not divide by zero and does not compensate");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, 0);

    Battery_c battery;
    battery.begin();
    settle(battery);

    CHECK_EQ(battery.millivolts(), 0);
    CHECK_NEAR(battery.compensationFactor(), 1.0f, 0.0001f);
    CHECK(battery.isLow());
  }

  {
    TEST("a reading below the plausible minimum is rejected, not amplified");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, countsFor(BATTERY_MIN_VALID_MV - 200));

    Battery_c battery;
    battery.begin();
    settle(battery);

    CHECK(battery.millivolts() < BATTERY_MIN_VALID_MV);
    CHECK_NEAR(battery.compensationFactor(), 1.0f, 0.0001f);
  }

  {
    TEST("an implausibly low but non-zero pack is clamped at the upper bound");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, countsFor(1500));

    Battery_c battery;
    battery.begin();
    settle(battery);

    // 5000 / 1500 is 3.33, which would triple every demand.
    CHECK_NEAR(battery.compensationFactor(), BATTERY_COMP_MAX, 0.0001f);
  }

  {
    // The lower clamp cannot be reached through the divider as configured:
    // full scale is 7500 mV and 5000/7500 is 0.67, comfortably inside the
    // bound. It is kept because the divider constants are unverified, and a
    // correction to them could put a real reading past it. This test records
    // that the factor stays inside the bound at the extreme the hardware can
    // actually produce.
    TEST("an over-voltage reading reduces the demand without going below the bound");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, 1023);

    Battery_c battery;
    battery.begin();
    settle(battery);

    float factor = battery.compensationFactor();
    CHECK_NEAR(factor, 0.667f, 0.01f);
    CHECK(factor >= BATTERY_COMP_MIN);
  }

  {
    // Swept rather than spot checked. Every reading the ADC is physically
    // capable of returning, including the nonsense ones, must produce a
    // factor that is safe to multiply a motor demand by.
    TEST("no ADC reading whatsoever produces a factor outside the clamp");
    mockReset();

    bool all_within = true;

    for (int counts = 0; counts <= 1023; counts++) {
      mockSetAnalogRead(BATTERY_SENSE_PIN, counts);

      Battery_c battery;
      battery.begin();
      settle(battery);

      float factor = battery.compensationFactor();
      if (factor < BATTERY_COMP_MIN || factor > BATTERY_COMP_MAX) all_within = false;
    }

    CHECK(all_within);
  }

  {
    TEST("begin primes the average so the first reading is not dragged from zero");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, countsFor(5000));

    Battery_c battery;
    battery.begin();

    // Without priming this would be 5000/8, a sixth of the real voltage, and
    // the robot would believe the pack was flat at the moment it starts.
    CHECK_EQ(battery.millivolts(), 5000);
    CHECK_EQ(battery.readMillivolts(), 5000);
  }

  {
    TEST("one noisy sample moves the average only a fraction of the way");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, countsFor(5000));

    Battery_c battery;
    battery.begin();

    uint16_t before = battery.millivolts();

    // A sag to nearly nothing, as the motors surge for one iteration.
    mockSetAnalogRead(BATTERY_SENSE_PIN, 0);
    uint16_t after_one = battery.readMillivolts();

    // Seven of the eight slots still hold the real voltage, so the average
    // drops by about an eighth rather than collapsing.
    CHECK(after_one < before);
    CHECK(after_one > (uint16_t)(before * 3 / 4));

    // and a sustained change does eventually get through
    uint16_t after_all = settle(battery);
    CHECK_EQ(after_all, 0);
  }

  {
    TEST("a sustained drop is followed, and recovery is followed back");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, countsFor(6000));

    Battery_c battery;
    battery.begin();
    CHECK(!battery.isLow());

    mockSetAnalogRead(BATTERY_SENSE_PIN, countsFor(3800));
    settle(battery);
    CHECK(battery.isLow());

    mockSetAnalogRead(BATTERY_SENSE_PIN, countsFor(6000));
    settle(battery);
    CHECK(!battery.isLow());
  }

  {
    TEST("millivolts does not take a new conversion of its own");
    mockReset();
    mockSetAnalogRead(BATTERY_SENSE_PIN, countsFor(5000));

    Battery_c battery;
    battery.begin();

    // Changing the pin without reading must not change the reported value.
    mockSetAnalogRead(BATTERY_SENSE_PIN, 0);
    CHECK_EQ(battery.millivolts(), 5000);
    CHECK_EQ(battery.millivolts(), 5000);
  }

  return testSummary("battery");
}
