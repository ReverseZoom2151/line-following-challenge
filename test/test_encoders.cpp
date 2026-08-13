// Host tests for encoders.h.
//
// Nothing on a host fires an interrupt, so these call the ISR entry points
// directly. That is the only way to exercise the counting logic without a
// robot, and it is enough to pin down the arithmetic: direction decoding,
// counts to millimetres, and millimetres to a heading change.
//
// What these tests cannot show: that the pin numbers are right, that the
// gear ratio is 75:1 on this particular board, that the track width is 96 mm,
// or that the right encoder can be read at all given the conflict documented
// at the top of encoders.h. All of that needs the hardware.

#include "arduino_stub.h"
#include "test_harness.h"

#include "encoders.h"

// After begin() the channel pins are INPUT_PULLUP, so the stub reports
// whatever was last written to them. That is what lets a test pose the two
// channels in a chosen phase relationship and then fire the handler.
static void setChannelB(uint8_t pin, int level) {
  digitalWrite(pin, level);
}

int main() {
  printf("encoders\n");

  {
    TEST("begin configures both channels of both encoders and attaches two handlers");
    mockReset();
    mockAttachedInterrupts = 0;

    Encoders_c encoders;
    encoders.begin();

    CHECK_EQ(mockPins[ENCODER_LEFT_A_PIN].mode, INPUT_PULLUP);
    CHECK_EQ(mockPins[ENCODER_LEFT_B_PIN].mode, INPUT_PULLUP);
    CHECK_EQ(mockPins[ENCODER_RIGHT_A_PIN].mode, INPUT_PULLUP);
    CHECK_EQ(mockPins[ENCODER_RIGHT_B_PIN].mode, INPUT_PULLUP);
    CHECK_EQ(mockAttachedInterrupts, 2);

    CHECK_EQ(encoders.leftCount(), 0);
    CHECK_EQ(encoders.rightCount(), 0);
  }

  {
    TEST("channels in phase count one way, out of phase the other");
    mockReset();
    Encoders_c encoders;
    encoders.begin();

    // B agrees with A: forward, by the convention in encoders.h.
    setChannelB(ENCODER_LEFT_B_PIN, LOW);
    for (int i = 0; i < 10; i++) Encoders_c::handleLeftA();
    CHECK_EQ(encoders.leftCount(), 10);

    // B disagrees with A: the wheel is being turned the other way, and the
    // count must come back down rather than keep climbing. A counter that
    // only ever increments makes a robot rocking on the spot look like a
    // robot driving forwards.
    setChannelB(ENCODER_LEFT_B_PIN, HIGH);
    for (int i = 0; i < 4; i++) Encoders_c::handleLeftA();
    CHECK_EQ(encoders.leftCount(), 6);

    // and back to where it started
    for (int i = 0; i < 6; i++) Encoders_c::handleLeftA();
    CHECK_EQ(encoders.leftCount(), 0);
  }

  {
    TEST("the two wheels count independently");
    mockReset();
    Encoders_c encoders;
    encoders.begin();

    for (int i = 0; i < 7; i++) Encoders_c::handleLeftA();
    for (int i = 0; i < 3; i++) Encoders_c::handleRightA();

    CHECK_EQ(encoders.leftCount(), 7);
    CHECK_EQ(encoders.rightCount(), 3);
  }

  {
    TEST("reset zeroes both counts");
    mockReset();
    Encoders_c encoders;
    encoders.begin();

    for (int i = 0; i < 5; i++) Encoders_c::handleLeftA();
    for (int i = 0; i < 5; i++) Encoders_c::handleRightA();
    CHECK_EQ(encoders.leftCount(), 5);

    encoders.reset();
    CHECK_EQ(encoders.leftCount(), 0);
    CHECK_EQ(encoders.rightCount(), 0);
    CHECK_NEAR(encoders.leftDistanceMm(), 0.0f, 0.001f);
    CHECK_NEAR(encoders.headingChangeDegrees(), 0.0f, 0.001f);
  }

  {
    TEST("one wheel revolution is one wheel circumference");
    mockReset();
    Encoders_c encoders;
    encoders.begin();

    for (int i = 0; i < (int)ENCODER_COUNTS_PER_WHEEL_REV; i++) Encoders_c::handleLeftA();

    // 12 CPR, a quarter of the edges counted, 75:1 gearbox: 225 counts.
    CHECK_EQ((long)ENCODER_COUNTS_PER_WHEEL_REV, 225);
    CHECK_NEAR(encoders.leftDistanceMm(), WHEEL_CIRCUMFERENCE_MM, 0.01f);
    CHECK_NEAR(encoders.leftDistanceMm(), 100.53f, 0.01f);
  }

  {
    TEST("distance is signed, so reversing unwinds it");
    mockReset();
    Encoders_c encoders;
    encoders.begin();

    setChannelB(ENCODER_LEFT_B_PIN, HIGH);  // reverse
    for (int i = 0; i < 225; i++) Encoders_c::handleLeftA();

    CHECK_EQ(encoders.leftCount(), -225);
    CHECK_NEAR(encoders.leftDistanceMm(), -100.53f, 0.01f);
  }

  {
    TEST("equal counts on both wheels is a straight line, not a turn");
    mockReset();
    Encoders_c encoders;
    encoders.begin();

    for (int i = 0; i < 225; i++) {
      Encoders_c::handleLeftA();
      Encoders_c::handleRightA();
    }

    CHECK_NEAR(encoders.leftDistanceMm(), encoders.rightDistanceMm(), 0.001f);
    CHECK_NEAR(encoders.headingChangeDegrees(), 0.0f, 0.001f);
  }

  {
    // This is what replaces TURN_TIMEOUT_MS. A spin on the spot drives the
    // wheels in opposite directions, and the angle turned falls out of the
    // two counts and the track width instead of out of a stopwatch.
    TEST("a spin on the spot yields a measured heading change");
    mockReset();
    Encoders_c encoders;
    encoders.begin();

    // right wheel forward, left wheel backward: anticlockwise, a left turn
    setChannelB(ENCODER_RIGHT_B_PIN, LOW);
    setChannelB(ENCODER_LEFT_B_PIN, HIGH);
    for (int i = 0; i < 225; i++) {
      Encoders_c::handleLeftA();
      Encoders_c::handleRightA();
    }

    CHECK_EQ(encoders.leftCount(), -225);
    CHECK_EQ(encoders.rightCount(), 225);

    // 2 * 100.53 mm of arc over a 96 mm track is 2.094 rad, or 120 degrees.
    CHECK_NEAR(encoders.headingChangeDegrees(), 120.0f, 0.1f);
  }

  {
    TEST("the sign of the heading change distinguishes left from right");
    mockReset();
    Encoders_c encoders;
    encoders.begin();

    // left wheel forward only: the robot pivots clockwise, to the right
    for (int i = 0; i < 100; i++) Encoders_c::handleLeftA();
    float clockwise = encoders.headingChangeDegrees();
    CHECK(clockwise < 0.0f);

    encoders.reset();

    // right wheel forward only: anticlockwise, to the left
    for (int i = 0; i < 100; i++) Encoders_c::handleRightA();
    float anticlockwise = encoders.headingChangeDegrees();
    CHECK(anticlockwise > 0.0f);

    CHECK_NEAR(anticlockwise, -clockwise, 0.001f);
  }

  {
    // A long is four bytes on an eight-bit MCU, so reading one is not atomic
    // and an interrupt landing mid-read returns a value the wheel never had.
    // Nothing fires interrupts here, so what a host test can prove is that
    // the guard is actually taken, and that it is balanced: a missing
    // interrupts() would leave the robot deaf to every interrupt afterwards,
    // which is far worse than a torn count.
    TEST("counter reads are guarded and the guard is balanced");
    mockReset();
    Encoders_c encoders;
    encoders.begin();

    mockInterruptGuardDepth = 0;
    mockInterruptGuardEntries = 0;

    (void)encoders.leftCount();
    CHECK_EQ(mockInterruptGuardEntries, 1);
    CHECK_EQ(mockInterruptGuardDepth, 0);

    (void)encoders.rightCount();
    encoders.reset();
    CHECK_EQ(mockInterruptGuardEntries, 3);
    CHECK_EQ(mockInterruptGuardDepth, 0);
  }

  {
    // The counts live at namespace scope because the ISR entry points are
    // static and have no object to reach through. That makes them shared
    // state, which is worth recording explicitly: constructing a second
    // Encoders_c does not give a fresh pair of counters.
    TEST("counts are shared state, not per instance");
    mockReset();
    Encoders_c first;
    first.begin();

    for (int i = 0; i < 12; i++) Encoders_c::handleLeftA();

    Encoders_c second;
    CHECK_EQ(second.leftCount(), 12);

    second.reset();
    CHECK_EQ(first.leftCount(), 0);
  }

  return testSummary("encoders");
}
