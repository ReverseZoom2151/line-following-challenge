#ifndef MOTORS_H
#define MOTORS_H
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15
#define FWD LOW  
#define REV HIGH 

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

      // error handling & range limiting (assume 0-150 is optimal):
      // const int MAX_PWM = 150;  
      const int MAX_PWM = 255;  
      const int MIN_PWM = 0; 

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

    void driveStraight(int speed) {

      // sets directions for forward movement
      digitalWrite(L_DIR_PIN, FWD);
      digitalWrite(R_DIR_PIN, FWD);

      // drives both motors at the specified speed
      analogWrite(L_PWM_PIN, speed); 
      analogWrite(R_PWM_PIN, speed);
      
    }

    void spin180(const float speed) {

      digitalWrite(L_DIR_PIN, REV);  
      digitalWrite(R_DIR_PIN, FWD);

      analogWrite(L_PWM_PIN, speed); 
      analogWrite(R_PWM_PIN, speed);

      delay(1000);

    }

    void spinLeft(const float speed, int pause) {

      digitalWrite(L_DIR_PIN, REV);  
      digitalWrite(R_DIR_PIN, FWD);

      analogWrite(L_PWM_PIN, speed); 
      analogWrite(R_PWM_PIN, speed);

      delay(pause);

    }

    void spinRight(const float speed, int pause) {

      digitalWrite(L_DIR_PIN, FWD);  
      digitalWrite(R_DIR_PIN, REV);

      analogWrite(L_PWM_PIN, speed); 
      analogWrite(R_PWM_PIN, speed);

      delay(pause);

    }

};

#endif
