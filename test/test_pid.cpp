// Host tests for pid.h.
//
// The controller ships with Ki and Kd at zero, so most of what is exercised
// here is dormant in the default build. It is tested anyway: the point of the
// gains being in config.h is that someone can raise them on the day they have
// a robot in front of them, and these tests are what says the terms behave
// when they do.

#include "arduino_stub.h"
#include "test_harness.h"

#include "pid.h"

int main() {
  printf("pid\n");

  {
    TEST("with Ki and Kd at zero it is exactly proportional");
    PID_c pid(40.0f, 0.0f, 0.0f, -255.0f, 255.0f, 50.0f);

    CHECK_NEAR(pid.update(0.5f, 0), 20.0, 0.001);
    CHECK_NEAR(pid.update(0.5f, 10), 20.0, 0.001);
    CHECK_NEAR(pid.update(-0.25f, 20), -10.0, 0.001);
    CHECK_NEAR(pid.update(0.0f, 30), 0.0, 0.001);
  }

  {
    TEST("output is clamped to the configured range");
    PID_c pid(40.0f, 0.0f, 0.0f, -30.0f, 30.0f, 50.0f);

    CHECK_NEAR(pid.update(5.0f, 0), 30.0, 0.001);
    CHECK_NEAR(pid.update(-5.0f, 10), -30.0, 0.001);
  }

  {
    TEST("the first sample produces no derivative spike");
    // A large Kd with a large first error would kick hard if the controller
    // treated the absent previous sample as zero.
    PID_c pid(1.0f, 0.0f, 100.0f, -1000.0f, 1000.0f, 50.0f);

    float first = pid.update(1.0f, 0);
    CHECK_NEAR(first, 1.0, 0.001);  // P term only

    // and a steady error still differentiates to zero
    CHECK_NEAR(pid.update(1.0f, 10), 1.0, 0.001);
  }

  {
    TEST("the derivative term tracks the rate of change");
    PID_c pid(0.0f, 0.0f, 1.0f, -1000.0f, 1000.0f, 50.0f);

    pid.update(0.0f, 0);
    // 0.1 of error over 10ms is a rate of 10 per second
    CHECK_NEAR(pid.update(0.1f, 10), 10.0, 0.001);
    // and the same error again is a rate of zero
    CHECK_NEAR(pid.update(0.1f, 20), 0.0, 0.001);
    // falling error differentiates negative
    CHECK(pid.update(0.0f, 30) < 0.0);
  }

  {
    TEST("dt of zero does not divide by zero");
    PID_c pid(2.0f, 1.0f, 100.0f, -1000.0f, 1000.0f, 50.0f);

    pid.update(0.5f, 100);
    float same = pid.update(0.5f, 100);   // same millisecond
    float again = pid.update(-0.5f, 100); // same millisecond, error flipped

    CHECK(same == same);    // not NaN
    CHECK(again == again);
    CHECK_NEAR(same, 1.0, 0.001);    // P only, nothing integrated or differentiated
    CHECK_NEAR(again, -1.0, 0.001);

    // time moving on again resumes normal behaviour
    float later = pid.update(-0.5f, 110);
    CHECK(later == later);
  }

  {
    TEST("the integral is clamped, so it cannot wind up");
    PID_c pid(0.0f, 1.0f, 0.0f, -1000.0f, 1000.0f, 50.0f);

    // a steady error of 10 for 10 seconds would integrate to 100 unclamped
    uint32_t t = 0;
    float out = 0.0f;
    for (int i = 0; i < 100; i++) {
      t += 100;
      out = pid.update(10.0f, t);
    }

    CHECK_NEAR(out, 50.0, 0.001);  // pinned at the limit, not 100

    // and it unwinds promptly once the error reverses, rather than holding
    // the output saturated for as long as it was saturated on the way in
    t += 1000;
    float afterOneSecond = pid.update(-10.0f, t);
    CHECK_NEAR(afterOneSecond, 40.0, 0.001);

    // the clamp is symmetric
    for (int i = 0; i < 100; i++) {
      t += 100;
      out = pid.update(-10.0f, t);
    }
    CHECK_NEAR(out, -50.0, 0.001);
  }

  {
    TEST("reset clears the integral and the derivative history");
    PID_c pid(0.0f, 1.0f, 100.0f, -1000.0f, 1000.0f, 50.0f);

    uint32_t t = 0;
    for (int i = 0; i < 20; i++) {
      t += 100;
      pid.update(10.0f, t);
    }
    CHECK(pid.update(10.0f, t + 100) > 10.0);  // integral is loaded

    pid.reset();

    // first sample after a reset: no integral, and no derivative kick
    CHECK_NEAR(pid.update(10.0f, t + 200), 0.0, 0.001);
  }

  {
    TEST("setGains takes effect on the next update");
    PID_c pid(1.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, 50.0f);

    CHECK_NEAR(pid.update(2.0f, 0), 2.0, 0.001);
    pid.setGains(10.0f, 0.0f, 0.0f);
    CHECK_NEAR(pid.update(2.0f, 10), 20.0, 0.001);
  }

  {
    TEST("the configured gains reproduce the original steering response");
    // config.h derives Kp from the original BiasPWM/MaxTurnPWM controller:
    // full deflection must still command 40 of PWM either way.
    PID_c pid(PID_KP, PID_KI, PID_KD, -MAX_PWM, MAX_PWM, PID_INTEGRAL_LIMIT);

    CHECK_NEAR(pid.update(1.0f, 0), 40.0, 0.001);
    CHECK_NEAR(pid.update(-1.0f, 10), -40.0, 0.001);
    CHECK_NEAR(pid.update(0.0f, 20), 0.0, 0.001);
  }

  return testSummary("pid");
}
