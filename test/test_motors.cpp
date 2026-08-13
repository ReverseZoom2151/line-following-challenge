// Host tests for motors.h.
//
// These pin down the behaviour that is already correct: PWM clamping, the
// direction pin following the sign of the demand, and the magnitude written
// being the absolute value of the demand.

#include "arduino_stub.h"
#include "test_harness.h"

#include "motors.h"

int main() {
  printf("motors\n");

  {
    TEST("forward demand drives both channels forward");
    mockReset();
    Motors_c motors;
    motors.begin();
    motors.setMotorPower(30.0f, 30.0f);

    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 30);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 30);
  }

  {
    TEST("negative demand flips direction and writes magnitude");
    mockReset();
    Motors_c motors;
    motors.begin();
    motors.setMotorPower(-40.0f, 40.0f);

    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, REV);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 40);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 40);
  }

  {
    TEST("demand beyond the PWM range is clamped");
    mockReset();
    Motors_c motors;
    motors.begin();
    motors.setMotorPower(1000.0f, -1000.0f);

    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 255);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 255);
    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, REV);
  }

  {
    TEST("zero demand counts as forward and writes zero duty");
    mockReset();
    Motors_c motors;
    motors.begin();
    motors.setMotorPower(0.0f, 0.0f);

    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 0);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 0);
    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, FWD);
  }

  {
    TEST("begin configures all four motor pins as outputs");
    mockReset();
    Motors_c motors;
    motors.begin();

    CHECK_EQ(mockPins[L_PWM_PIN].mode, OUTPUT);
    CHECK_EQ(mockPins[R_PWM_PIN].mode, OUTPUT);
    CHECK_EQ(mockPins[L_DIR_PIN].mode, OUTPUT);
    CHECK_EQ(mockPins[R_DIR_PIN].mode, OUTPUT);
  }

  {
    // stop() used to trap in while(1) and needed a hard reset to escape.
    // Reaching the assertions below at all proves it now returns.
    TEST("stop zeroes both channels, returns, and is repeatable");
    mockReset();
    Motors_c motors;
    motors.begin();
    motors.setMotorPower(80.0f, 80.0f);

    motors.stop();
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 0);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 0);

    motors.stop();
    motors.stop();
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 0);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 0);

    // and the robot can be driven again afterwards, without a reset
    motors.setMotorPower(30.0f, 30.0f);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 30);
  }

  {
    TEST("driveStraight sets both directions forward");
    mockReset();
    Motors_c motors;
    motors.begin();
    motors.driveStraight(30);

    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 30);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 30);
  }

  {
    TEST("spinLeft and spinRight oppose the two channels");
    mockReset();
    Motors_c motors;
    motors.begin();

    motors.spinLeft(30);
    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, REV);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 30);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 30);

    motors.spinRight(30);
    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, FWD);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, REV);
  }

  {
    // Regression: these three used to write analogWrite directly, so an
    // out-of-range demand truncated into an 8-bit register instead of being
    // clamped. A demand of 300 became 44, a slow crawl rather than full speed.
    TEST("the movement primitives clamp rather than wrap");
    mockReset();
    Motors_c motors;
    motors.begin();

    motors.driveStraight(300);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 255);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 255);

    motors.spinLeft(300);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 255);
    CHECK_EQ(mockPins[L_DIR_PIN].digitalValue, REV);

    motors.spinRight(300);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 255);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, REV);
  }

  {
    TEST("compensation scales the demand and defaults to no change");
    mockReset();
    Motors_c motors;
    motors.begin();

    // A robot that never sets a factor behaves exactly as it always did.
    CHECK_NEAR(motors.compensationFactor(), 1.0f, 0.0001f);
    motors.setMotorPower(30.0f, 30.0f);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 30);

    // A flat pack needs more PWM for the same torque.
    motors.setCompensation(1.5f);
    motors.setMotorPower(30.0f, 30.0f);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 45);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 45);
  }

  {
    // Safety property: an implausible factor must not reach the motors. The
    // safe direction on a bad battery reading is no compensation, not more
    // power, because the reading may be bad precisely because something is
    // disconnected.
    TEST("an implausible compensation factor is refused, not applied");
    mockReset();
    Motors_c motors;
    motors.begin();

    const float bad[] = { 0.0f, -1.0f, 5.0f, 100.0f, -0.5f };

    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
      motors.setCompensation(bad[i]);
      CHECK_NEAR(motors.compensationFactor(), 1.0f, 0.0001f);
    }

    motors.setCompensation(30.0f);
    motors.setMotorPower(30.0f, 30.0f);
    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 30);  // not 900, and not wrapped
  }

  {
    TEST("compensation cannot push the output past the PWM clamp");
    mockReset();
    Motors_c motors;
    motors.begin();

    motors.setCompensation(2.0f);
    motors.setMotorPower(200.0f, -200.0f);

    CHECK_EQ(mockPins[L_PWM_PIN].analogValue, 255);
    CHECK_EQ(mockPins[R_PWM_PIN].analogValue, 255);
    CHECK_EQ(mockPins[R_DIR_PIN].digitalValue, REV);
  }

  return testSummary("motors");
}
