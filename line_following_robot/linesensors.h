#ifndef LINESENSOR_H
#define LINESENSOR_H
#define MAX_SAMPLES 10
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
    float results[MAX_SAMPLES];
    unsigned long sensorReadings[5]; // array to store readings

  public:

    LineSensor_c() {}

    void setupOneLineSensor() {

      pinMode(EMIT_PIN, INPUT);  // IR LEDs off 
      pinMode(LS_LEFT_PIN, INPUT); 

    }

    void setupAllLineSensors() {

      pinMode(EMIT_PIN, INPUT);  
      for (int i = 0; i < 5; i++) {
        pinMode(ls_pins[i], INPUT);
      }

    }

    void algorithm_1() {

      unsigned long elapsed_time;

      // 1. enables IR LEDs
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);  

      // 2. sets sensor pin as OUTPUT to charge capacitor
      pinMode(LS_LEFT_PIN, OUTPUT);
      digitalWrite(LS_LEFT_PIN, HIGH); 
      delayMicroseconds(10); 

      // 3. switches sensor pin to INPUT for discharge measurement
      pinMode(LS_LEFT_PIN, INPUT); 

      // 4. captures start time 
      unsigned long start_time = micros();

      // 5. waits for discharge with timeout
      // unsigned long max_wait = 5000; 
      unsigned long max_wait = 2000; // Timeout in microseconds

      while (digitalRead(LS_LEFT_PIN) == HIGH) {

        unsigned long current_time = micros();

        if ((current_time - start_time) > max_wait) { 
          // elapsed_time = 0;
          break; // timeout has occurred 
        }

      }

      // 6. captures end time
      unsigned long end_time = micros(); 

      // 7. disables IR LEDs 
      pinMode(EMIT_PIN, INPUT); 

      // 8. computes elapsed time
      elapsed_time = end_time - start_time;

      // 9. reports to serial monitor
      Serial.println(elapsed_time); 

    }

    void algorithm_2() {

      // collects 10 sensor readings and store in results[]
      for (int i = 0; i < MAX_SAMPLES; i++) {

        // turns on IR LEDs directly
        pinMode(EMIT_PIN, OUTPUT); 
        digitalWrite(EMIT_PIN, HIGH); 

        pinMode(LS_LEFT_PIN, OUTPUT);
        digitalWrite(LS_LEFT_PIN, HIGH);
        delayMicroseconds(10);  

        pinMode(LS_LEFT_PIN, INPUT); 

        unsigned long start_time = micros();
        unsigned long timeout = 5000; 

        while (digitalRead(LS_LEFT_PIN) == HIGH) {

          unsigned long current_time = micros();

          if ((current_time - start_time) > timeout) {
            results[i] = -1; 
            break; 
          }

        }

        // obtains end_time only if successful reading 
        unsigned long end_time = (results[i] != -1) ? micros() : 0;  

        pinMode(EMIT_PIN, INPUT); // turns off EMIT 

        if (results[i] != -1) {  

          unsigned long elapsed_time = end_time - start_time;
          results[i] = elapsed_time;
        
        }

        delay(200); // optional delay between measurements

      }

      // reports all readings / results, forever 
      while (true) {

        Serial.println("Results: ");

        for (int i = 0; i < MAX_SAMPLES; i++) {
          Serial.println(results[i]);
        }

        // waits before next trial (optional)
        delay(1000);

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

      unsigned long start_time = micros();

      while (digitalRead(ls_pins[number]) == HIGH) {}

      unsigned long end_time = micros();

      pinMode(EMIT_PIN, INPUT); 

      unsigned long elapsed_time = end_time - start_time;
      
      return elapsed_time; 

    } 

    void multipleSensors() {

      // collects MAX_SAMPLES readings for each of the 5 sensors
      for (int i = 0; i < MAX_SAMPLES; i++) {

        for (int sensor_num = 0; sensor_num < 5; sensor_num++) {

          sensorReadings[sensor_num] = readLineSensor(sensor_num); 
          delay(200); 

        }

        // reports results for set of readings
        Serial.print("[");
        Serial.print(i); // greyscale index (adjust if needed)

        for (int sensor_num = 0; sensor_num < 5; sensor_num++) {

          Serial.print(", ");
          Serial.print(sensorReadings[sensor_num]);
          
        }

        Serial.println("]");

      }

    }

    void readAllSensorsPseudoParallel() {

      const unsigned long TIMEOUT_MICROSECONDS = 2000;

      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH); // turns on IR LEDs
      
      // sets all sensor pins to OUTPUT and charges them
      for (int i = 0; i < 5; i++) {
        pinMode(ls_pins[i], OUTPUT);
        digitalWrite(ls_pins[i], HIGH);
      }
      
      delayMicroseconds(10); // charges for 10 microseconds
      
      // sets all sensor pins to INPUT for discharging and start timing
      unsigned long start_times[5];
      bool sensorDone[5] = {false, false, false, false, false};
      int sensorsStillReading = 5;
      
      for (int i = 0; i < 5; i++) {
        pinMode(ls_pins[i], INPUT);
        start_times[i] = micros();
      }
      
      // polls each sensor to check if it has finished reading
      while (sensorsStillReading > 0) {

        for (int i = 0; i < 5; i++) {

          if (!sensorDone[i] && digitalRead(ls_pins[i]) == LOW) {

            unsigned long end_time = micros();
            sensorReadings[i] = end_time - start_times[i];
            sensorDone[i] = true;
            sensorsStillReading--;
            // prints the reading for this sensor
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(sensorReadings[i]);

          }
          
          // checks for timeout
          if (!sensorDone[i] && (micros() - start_times[i]) > TIMEOUT_MICROSECONDS) {

            sensorReadings[i] = TIMEOUT_MICROSECONDS; // assings timeout value
            sensorDone[i] = true;
            sensorsStillReading--;
            // prints a timeout message for this sensor
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.println(" timed out.");

          }

        }

      }
      
      // turns off IR LEDs
      pinMode(EMIT_PIN, INPUT); // turns off IR LEDs

      // debugging: indicates that all sensors have been read
      Serial.println("All sensors read.");

    }

};

#endif
