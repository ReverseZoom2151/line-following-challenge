#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

class Motors_c {

  public:

    Motors_c() {

    }

    void begin() {

      // sets all the motor pins as outputs
      pinMode(L_PWM_PIN, OUTPUT);
      pinMode(R_PWM_PIN, OUTPUT);
      pinMode(L_DIR_PIN, OUTPUT);
      pinMode(R_DIR_PIN, OUTPUT);

    }

    void setMotorPower(float left_pwm, float right_pwm) {

      // range limiting: demands outside +/-MAX_PWM are clamped, not wrapped

      // left motor
      if (left_pwm > MAX_PWM)  left_pwm = MAX_PWM;
      else if (left_pwm < -MAX_PWM) left_pwm = -MAX_PWM;  // clamp values  

      digitalWrite(L_DIR_PIN, (left_pwm >= 0) ? FWD : REV); 
      analogWrite(L_PWM_PIN, abs(left_pwm)); // absolute value  

      // right motor (same logic applied)
      if (right_pwm > MAX_PWM) right_pwm = MAX_PWM;  
      else if (right_pwm < -MAX_PWM) right_pwm = -MAX_PWM;

      digitalWrite(R_DIR_PIN, (right_pwm >= 0) ? FWD : REV); 
      analogWrite(R_PWM_PIN, abs(right_pwm));

    }

    // zeroes both channels and returns immediately, so the caller keeps
    // control and can decide to stay halted, or to move again
    void stop() {

      analogWrite(L_PWM_PIN, 0);
      analogWrite(R_PWM_PIN, 0);

    }

    // The movement primitives below all route through setMotorPower rather
    // than writing the PWM pins themselves, so that a demand outside
    // +/-MAX_PWM is clamped in exactly one place. Writing analogWrite
    // directly would truncate into an 8-bit register instead: a demand of
    // 300 would wrap to 44 and the robot would quietly crawl.
    void driveStraight(const float speed) {

      setMotorPower(speed, speed);

    }

    // Spins on the spot. Like every other method here it commands the
    // motors and returns at once: how long to spin for is the caller's
    // decision, taken between sensor readings rather than inside a delay.
    void spinLeft(const float speed) {

      setMotorPower(-speed, speed);

    }

    void spinRight(const float speed) {

      setMotorPower(speed, -speed);

    }

};

#endif
