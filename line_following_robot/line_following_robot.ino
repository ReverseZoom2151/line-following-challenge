#include "motors.h"
#include "linesensors.h"

#define SENSOR_THRESHOLD 1000 
#define STATE_JOIN_LINE 0
#define STATE_FOLLOW_LINE 1
#define STATE_TURN_LEFT 2
#define STATE_TURN_RIGHT 3
#define STATE_CROSSROADS 4
#define STATE_REDISCOVER_LINE 5
#define STATE_DETECT_END 6

LineSensor_c line_sensors;
Motors_c motors;

const unsigned long maxLineFollowDuration = 2000;
const int TRIGGER_DELAY = 200;   // delay in milliseconds
const float BiasPWM = 30.0; // starts with a positive forward bias
const float MaxTurnPWM = 20.0; 
int state = 0;
int lineDetectionCount = 0;
unsigned long lastTriggerTime = 0;  // tracks when the last trigger occurred
unsigned long lineFollowingStartTime = 0;  // start time when following the line


bool lineDetected() {

  unsigned long leftSensorReading = line_sensors.readLineSensor(1); // DN2
  unsigned long middleSensorReading = line_sensors.readLineSensor(2); // DN3
  unsigned long rightSensorReading = line_sensors.readLineSensor(3); // DN4

  // reads all three line sensors and determines if any sensor detects the line
  return ((leftSensorReading >= SENSOR_THRESHOLD) || (middleSensorReading >= SENSOR_THRESHOLD) || (rightSensorReading >= SENSOR_THRESHOLD));

}

float weightedMeasurement() {

  unsigned long leftSensorReading = line_sensors.readLineSensor(1); // DN2
  unsigned long rightSensorReading = line_sensors.readLineSensor(3); // DN4

  unsigned long sum = leftSensorReading + rightSensorReading;

  float leftNormalized = (float)leftSensorReading / sum;
  float rightNormalized = (float)rightSensorReading / sum;

  float leftWeighted = 2.0 * leftNormalized;
  float rightWeighted = 2.0 * rightNormalized;

  float W = rightWeighted - leftWeighted;
  // float W = leftWeighted - rightWeighted;

  // Serial.print("Weighted Measurement (W): ");
  // Serial.println(W); 

  return W;

}

void followLine() {

  unsigned long farLeftSensorReading = line_sensors.readLineSensor(0); // DN1
  unsigned long leftSensorReading = line_sensors.readLineSensor(1); // DN2
  unsigned long middleSensorReading = line_sensors.readLineSensor(2); // DN3
  unsigned long rightSensorReading = line_sensors.readLineSensor(3); // DN4
  unsigned long farRightSensorReading = line_sensors.readLineSensor(4); // DN5

  if (lineDetected()) { 

    float W = weightedMeasurement();
    float LeftPWM = BiasPWM + (MaxTurnPWM * W);
    float RightPWM = BiasPWM - (MaxTurnPWM * W);
    
    motors.setMotorPower(LeftPWM, RightPWM);
  
  }

  if (millis() - lineFollowingStartTime > maxLineFollowDuration && !lineDetected()) {

    state = STATE_DETECT_END; 
    return;

  } else if (millis() - lineFollowingStartTime < maxLineFollowDuration && !lineDetected()) {
    
    state = STATE_REDISCOVER_LINE;

  }

  if (farLeftSensorReading >= SENSOR_THRESHOLD && lineDetected()) {

    state = STATE_TURN_LEFT;

  } 
  
  if (farRightSensorReading >= SENSOR_THRESHOLD && !lineDetected()) {

    state = STATE_TURN_RIGHT;

  } 

  if (farLeftSensorReading >= SENSOR_THRESHOLD && farRightSensorReading >= SENSOR_THRESHOLD && !lineDetected()) {

    state = STATE_CROSSROADS;

  } 

}

void turnLeft() {

  unsigned long farLeftSensorReading = line_sensors.readLineSensor(0); // DN1
  unsigned long leftSensorReading = line_sensors.readLineSensor(1); // DN2
  unsigned long middleSensorReading = line_sensors.readLineSensor(2); // DN3
  unsigned long rightSensorReading = line_sensors.readLineSensor(3); // DN4
  unsigned long farRightSensorReading = line_sensors.readLineSensor(4); // DN5

  if (farLeftSensorReading >= SENSOR_THRESHOLD && lineDetected()) {

    // executes a sharp left turn for corners (90 degrees)
    motors.spinLeft(BiasPWM, 250);
  
  } 
  
  if (farLeftSensorReading >= SENSOR_THRESHOLD && lineDetected() && farRightSensorReading >= SENSOR_THRESHOLD) {

    motors.driveStraight(BiasPWM);
    delay(225);
    motors.spinLeft(BiasPWM, 250);

  }

  lineFollowingStartTime = millis();

  state = STATE_FOLLOW_LINE;

}

void turnRight() {

  unsigned long farLeftSensorReading = line_sensors.readLineSensor(0); // DN1
  unsigned long leftSensorReading = line_sensors.readLineSensor(1); // DN2
  unsigned long middleSensorReading = line_sensors.readLineSensor(2); // DN3
  unsigned long rightSensorReading = line_sensors.readLineSensor(3); // DN4
  unsigned long farRightSensorReading = line_sensors.readLineSensor(4); // DN5

  if (farLeftSensorReading >= SENSOR_THRESHOLD) {

    state = STATE_TURN_LEFT;

  } else {

    motors.spinRight(BiasPWM, 250);

  }

  lineFollowingStartTime = millis();

  state = STATE_FOLLOW_LINE;
 
}

void rediscoverLine() {

  unsigned long farLeftSensorReading = line_sensors.readLineSensor(0); // DN1
  unsigned long leftSensorReading = line_sensors.readLineSensor(1); // DN2
  unsigned long middleSensorReading = line_sensors.readLineSensor(2); // DN3
  unsigned long rightSensorReading = line_sensors.readLineSensor(3); // DN4
  unsigned long farRightSensorReading = line_sensors.readLineSensor(4); // DN5

  if (!lineDetected() && farLeftSensorReading < SENSOR_THRESHOLD && farRightSensorReading < SENSOR_THRESHOLD) {

    // drives straight for 0.4 seconds
    motors.driveStraight(BiasPWM);  
    // pauses for 400 milliseconds (0.4 seconds)
    delay(400);
    // spins clockwise 
    motors.spin180(32);  
    // resumes straight driving (no timer needed)   
    motors.driveStraight(BiasPWM); 

  } 
  
  if (lineDetected() || farLeftSensorReading >= SENSOR_THRESHOLD || farRightSensorReading >= SENSOR_THRESHOLD) {

    lineFollowingStartTime = millis();  
    state = STATE_FOLLOW_LINE;
  
  }

}

void crossroads() {

  unsigned long farLeftSensorReading = line_sensors.readLineSensor(0); // DN1
  unsigned long leftSensorReading = line_sensors.readLineSensor(1); // DN2
  unsigned long middleSensorReading = line_sensors.readLineSensor(2); // DN3
  unsigned long rightSensorReading = line_sensors.readLineSensor(3); // DN4
  unsigned long farRightSensorReading = line_sensors.readLineSensor(4); // DN5

  if (farLeftSensorReading >= SENSOR_THRESHOLD && farRightSensorReading >= SENSOR_THRESHOLD && !lineDetected()) {

    motors.spinLeft(BiasPWM, 250);
    
  }

  lineFollowingStartTime = millis();  
  state = STATE_FOLLOW_LINE;

}

void joinLine() {

  unsigned long farLeftSensorReading = line_sensors.readLineSensor(0); // DN1
  unsigned long leftSensorReading = line_sensors.readLineSensor(1); // DN2
  unsigned long middleSensorReading = line_sensors.readLineSensor(2); // DN3
  unsigned long rightSensorReading = line_sensors.readLineSensor(3); // DN4
  unsigned long farRightSensorReading = line_sensors.readLineSensor(4); // DN5

  motors.driveStraight(BiasPWM);

  if (lineDetected() && millis() - lastTriggerTime > TRIGGER_DELAY) {

    lineDetectionCount++;
    lastTriggerTime = millis();
  
  } 

  if (lineDetectionCount >= 2) {
    
    lineFollowingStartTime = millis();
    state = STATE_FOLLOW_LINE;
    lineDetectionCount = 0; 
    lastTriggerTime = 0;
    
  }

}

void detectEnd() {

  motors.stopRobot();

}

void updateState() {

  if (state == STATE_JOIN_LINE) {

    joinLine();

  } else if (state == STATE_FOLLOW_LINE) {

    followLine();

  } else if (state == STATE_TURN_LEFT) {

    turnLeft();

  } else if (state == STATE_TURN_RIGHT) {
    
    turnRight();
    
  } else if (state == STATE_CROSSROADS) {

    crossroads();

  } else if (state == STATE_REDISCOVER_LINE) {

    rediscoverLine();

  } else if (state == STATE_DETECT_END) {

    detectEnd();

  }

}

void setup() {

  motors.initialise();
  line_sensors.setupAllLineSensors();

  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");

}

void loop() {

  updateState();

}
