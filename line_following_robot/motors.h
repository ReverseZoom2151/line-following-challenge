#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

class Motors_c {

  public:

    Motors_c() {

    }

  private:

    float compensation = 1.0f;

  public:

    void begin() {

      // sets all the motor pins as outputs
      pinMode(L_PWM_PIN, OUTPUT);
      pinMode(R_PWM_PIN, OUTPUT);
      pinMode(L_DIR_PIN, OUTPUT);
      pinMode(R_DIR_PIN, OUTPUT);

    }

    // Battery compensation. As the pack drains, the same PWM produces less
    // torque, so gains tuned on a fresh pack are wrong on a flat one. Scaling
    // the demand by nominal/measured keeps behaviour constant. Defaults to
    // 1.0, which is exactly no compensation, so a robot that never calls this
    // behaves as it always did.
    //
    // The factor is applied before clamping, so compensation can never push
    // the output past MAX_PWM.
    void setCompensation(float factor) {

      // A caller passing something absurd must not command full power. The
      // safe direction on a bad reading is no compensation at all.
      if (!(factor > 0.0f) || factor < BATTERY_COMP_MIN || factor > BATTERY_COMP_MAX) {
        compensation = 1.0f;
        return;
      }

      compensation = factor;

    }

    float compensationFactor() const { return compensation; }

    void setMotorPower(float left_pwm, float right_pwm) {

      left_pwm *= compensation;
      right_pwm *= compensation;

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
