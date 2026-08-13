#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

class Motors_c {

  public:

    Motors_c() {

    }

    void initialise() {

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

    void driveStraight(const float speed) {

      // sets directions for forward movement
      digitalWrite(L_DIR_PIN, FWD);
      digitalWrite(R_DIR_PIN, FWD);

      // drives both motors at the specified speed
      analogWrite(L_PWM_PIN, speed); 
      analogWrite(R_PWM_PIN, speed);
      
    }

    // Spins on the spot. Like every other method here it commands the
    // motors and returns at once: how long to spin for is the caller's
    // decision, taken between sensor readings rather than inside a delay.
    void spinLeft(const float speed) {

      digitalWrite(L_DIR_PIN, REV);
      digitalWrite(R_DIR_PIN, FWD);

      analogWrite(L_PWM_PIN, speed);
      analogWrite(R_PWM_PIN, speed);

    }

    void spinRight(const float speed) {

      digitalWrite(L_DIR_PIN, FWD);
      digitalWrite(R_DIR_PIN, REV);

      analogWrite(L_PWM_PIN, speed);
      analogWrite(R_PWM_PIN, speed);

    }

};

#endif
