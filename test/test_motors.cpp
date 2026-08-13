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

  return testSummary("motors");
}
