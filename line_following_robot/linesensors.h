#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "config.h"

class LineSensor_c {

  private: 

    uint8_t ls_pins[NUM_SENSORS] = { LS_LEFT_PIN, LS_MIDLEFT_PIN, LS_MIDDLE_PIN, LS_MIDRIGHT_PIN, LS_RIGHT_PIN }; // stores pin numbers for convenient access

  public:

    LineSensor_c() {}

    void setupAllLineSensors() {

      pinMode(EMIT_PIN, INPUT);  
      for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        pinMode(ls_pins[i], INPUT);
      }

    }

    // reads a line sensor with error checking
    unsigned long readLineSensor(int number) {

      // prevents memory errors 
      if (number < 0 || number >= NUM_SENSORS) {

        // 0 is the safe error value here: it is an unsigned function, so -1
        // would wrap to the type maximum and read as a solid line detection
        return 0;

      }

      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);
      pinMode(ls_pins[number], OUTPUT);
      digitalWrite(ls_pins[number], HIGH);
      delayMicroseconds(10); 

      pinMode(ls_pins[number], INPUT); // only switches to input for measurement

      // gives up after SENSOR_TIMEOUT_US so a sensor that never discharges
      // (black surface, or a broken connection) cannot stall the robot
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
