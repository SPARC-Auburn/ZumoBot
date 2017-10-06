#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;


#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];
// When the reading on a line sensor goes below this value, we
// consider that line sensor to have detected the white border at
// the edge of the ring.  This value might need to be tuned for
// different lighting conditions, surfaces, etc.
const uint16_t lineSensorThreshold = 1000;

// The speed that the robot uses when backing up.
const uint16_t reverseSpeed = 200;

// The speed that the robot uses when turning.
const uint16_t turnSpeed = 200;

// The speed that the robot usually uses when moving forward.
// You don't want this to be too fast because then the robot
// might fail to stop when it detects the white border.
const uint16_t forwardSpeed = 200;

// These two variables specify the speeds to apply to the motors
// when veering left or veering right.  While the robot is
// driving forward, it uses its proximity sensors to scan for
// objects ahead of it and tries to veer towards them.
const uint16_t veerSpeedLow = 0;
const uint16_t veerSpeedHigh = 250;

// The speed that the robot drives when it detects an opponent in
// front of it, either with the proximity sensors or by noticing
// that it is caught in a stalemate (driving forward for several
// seconds without reaching a border).  400 is full speed.
const uint16_t rammingSpeed = 400;

// The amount of time to spend backing up after detecting a
// border, in milliseconds.
const uint16_t reverseTime = 200;

// The minimum amount of time to spend scanning for nearby
// opponents, in milliseconds.
const uint16_t scanTimeMin = 200;

// The maximum amount of time to spend scanning for nearby
// opponents, in milliseconds.
const uint16_t scanTimeMax = 2100;

// The amount of time to wait between detecting a button press
// and actually starting to move, in milliseconds.  Typical robot
// sumo rules require 5 seconds of waiting.
const uint16_t waitTime = 5000;

uint8_t sensorValues[] = {1,1,1,1};
//speed range 0-400
//bool buttonPress = buttonA.getSingleDebouncedPress();
//motors.setSpeeds(-reverseSpeed, -reverseSpeed); left, right


void setup() {
  // initiallizes line and ir sensors
  lineSensors.initFiveSensors();
  proxSensors.initThreeSensors();
}

void loop() {
  lcd.clear();
  lcd.gotoXY(0,0);
  lcd.print("Press A");
  
  // put your main code here, to run repeatedly:
  while(bool buttonPress = buttonA.getSingleDebouncedPress() == false){
    //Stand By
  }
    
    //once A has been pressed readA ALL sensors to get a picture of what the surroundings are
    lcd.clear();
    lcd.print("Begin");
    
    scanAround();
    
    if(sensorValues[0] < 4 || sensorValues[1] < 4){
      moveForward();
    }
    
}
void scanAround(){
  //sensors read from 0 to 6, higher means closer
  proxSensors.read();
  //lineReader();
  uint8_t frontRightSensor = proxSensors.countsFrontWithRightLeds();
  uint8_t frontLeftSensor = proxSensors.countsFrontWithLeftLeds();
  uint8_t leftSensor = proxSensors.countsLeftWithLeftLeds();
  uint8_t rightSensor = proxSensors.countsRightWithRightLeds();
  sensorValues[0] = frontLeftSensor;
  sensorValues[1] = frontRightSensor;
  sensorValues[2] = leftSensor;
  sensorValues[3] = rightSensor;
  return;
}
void moveForward(){
  motors.setSpeeds(forwardSpeed, forwardSpeed);
  while(0 ==0){
    scanAround();
    lcd.clear();
    lcd.print(String(sensorValues[0]) + String(sensorValues[1]) + String(sensorValues[2]) + String(sensorValues[3]));
    
    if(sensorValues[0] == 6 or sensorValues[1] == 6){
      motors.setSpeeds(0, 0);
      lcd.clear();
      lcd.print("Target");
      lcd.gotoXY(0,1);
      lcd.print("Found");
      delay(5000);
      lcd.clear();
      return;
      }
  } 
  return;
}

void faceOP(surroundings){
  for(k = 1; k < sizeof(surroundings);k++){
    
  }
  
}

void turnRight(){
  motors.setSpeeds(400, -400);
}
void turnLeft(){
  motors.setSpeeds(-400, 400);
}
