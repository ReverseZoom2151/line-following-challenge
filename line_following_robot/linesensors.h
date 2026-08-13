#ifndef LINESENSOR_H
#define LINESENSOR_H
#define EMIT_PIN    11  
#define LS_LEFT_PIN 12  
#define LS_MIDLEFT_PIN A0   
#define LS_MIDDLE_PIN A2   
#define LS_MIDRIGHT_PIN A3  
#define LS_RIGHT_PIN A4   
#define LINE_SENSOR_UPDATE 100
#define MOTOR_UPDATE 2000 

class LineSensor_c {

  private: 

    int ls_pins[5] = { LS_LEFT_PIN, LS_MIDLEFT_PIN, LS_MIDDLE_PIN, LS_MIDRIGHT_PIN, LS_RIGHT_PIN }; // stores pin numbers for convenient access

  public:

    LineSensor_c() {}

    void setupAllLineSensors() {

      pinMode(EMIT_PIN, INPUT);  
      for (int i = 0; i < 5; i++) {
        pinMode(ls_pins[i], INPUT);
      }

    }

    // reads a line sensor with error checking
    unsigned long readLineSensor(int number) {

      // prevents memory errors 
      if (number < 0 || number > 4) {

        Serial.println("Error: sensor number out of range");
        return -1; // or some other clear error indication
     
      }

      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);
      pinMode(ls_pins[number], OUTPUT);
      digitalWrite(ls_pins[number], HIGH);
      delayMicroseconds(10); 

      pinMode(ls_pins[number], INPUT); // only switches to input for measurement

      // gives up after SENSOR_TIMEOUT_US so a sensor that never discharges
      // (black surface, or a broken connection) cannot stall the robot
      const unsigned long SENSOR_TIMEOUT_US = 2500;

      unsigned long start_time = micros();

      while (digitalRead(ls_pins[number]) == HIGH) {

        if ((micros() - start_time) > SENSOR_TIMEOUT_US) break;

      }

      unsigned long end_time = micros();

      pinMode(EMIT_PIN, INPUT); 

      unsigned long elapsed_time = end_time - start_time;
      
      return elapsed_time; 

    } 

};

#endif
