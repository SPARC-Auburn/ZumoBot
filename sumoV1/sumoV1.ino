#include <Wire.h>
#include <Zumo32U4.h>
#include <Zumo32U4LineSensors.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;



unsigned int lineSensorValues[3];
const uint8_t sensorThreshold = 1;
const uint16_t lineSensorThreshold = 1000;
#define LEFT 0
#define RIGHT 1

bool senseDir = RIGHT;

const uint16_t deceleration = 10;

// The amount to increase the speed by during each cycle when an
// object is not seen.
const uint16_t acceleration = 10;

// True if the robot is turning left (counter-clockwise).
bool turningLeft = false;

// True if the robot is turning right (clockwise).
bool turningRight = false;

// The time, in milliseconds, when an object was last seen.
uint16_t lastTimeObjectSeen = 0;
// If the robot is turning, this is the speed it will use.
uint16_t turnSpeed = 100;
// When the reading on a line sensor goes below this value, we
// consider that line sensor to have detected the white border at
// the edge of the ring.  This value might need to be tuned for
// different lighting conditions, surfaces, etc.


// The speed that the robot uses when backing up.
const uint16_t reverseSpeed = 100;


// The speed that the robot usually uses when moving forward.
// You don't want this to be too fast because then the robot
// might fail to stop when it detects the white border.
const uint16_t forwardSpeed = 100;

// These two variables specify the speeds to apply to the motors
// when veering left or veering right.  While the robot is
// driving forward, it uses its proximity sensors to scan for
// objects ahead of it and tries to veer towards them.
const uint16_t veerSpeedLow = 0;
const uint16_t veerSpeedHigh = 100;

// The speed that the robot drives when it detects an opponent in
// front of it, either with the proximity sensors or by noticing
// that it is caught in a stalemate (driving forward for several
// seconds without reaching a border).  400 is full speed.
const uint16_t rammingSpeed = 100;

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
  lineSensors.emittersOn();
  lineSensors.calibrate();
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
    
    delay(5000);
    lcd.clear();
    int i = 0;
    
    while(true){
      
      scanAround();
      if(sensorValues[0] != sensorValues[1]){
      findOp();
      }

      moveForward();
      detectLine();
      i++;
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

    lcd.clear();
    lcd.print(String(sensorValues[0]) + String(sensorValues[1]) + String(sensorValues[2]) + String(sensorValues[3]));
    
    if(sensorValues[0] == 6 or sensorValues[1] == 6){
      //motors.setSpeeds(0, 0);
      lcd.clear();
      lcd.print("Target");
      lcd.gotoXY(0,1);
      lcd.print("Found");
      //delay(5000);
      lcd.clear();
      return;
      }
  
  return;
}



void turnRight()
{
  motors.setSpeeds(turnSpeed, -turnSpeed);
  turningLeft = false;
  turningRight = true;
}

void turnLeft()
{
  motors.setSpeeds(-turnSpeed, turnSpeed);
  turningLeft = true;
  turningRight = false;
}

void stop()
{
  motors.setSpeeds(0, 0);
  turningLeft = false;
  turningRight = false;
}

void detectLine(){
  
  //lineSensors.readCalibrated(lineSensorValues);
   lineSensors.read(lineSensorValues);
    if (lineSensorValues[0] > lineSensorThreshold || lineSensorValues[1] > lineSensorThreshold || lineSensorValues[2] > lineSensorThreshold)
    {
      //lcd.gotoXY(0,0);
      //lcd.clear();
     // lcd.print("LINE");
      
    }
 }

void findOp(){
  while(sensorValues[0] != sensorValues[1]){

    scanAround();
    bool objectSeen = sensorValues[0] >= sensorThreshold || sensorValues[1] >= sensorThreshold || sensorValues[2] >= sensorThreshold || sensorValues[3] >= sensorThreshold;
  
    if (objectSeen)
    {
      // An object is visible, so we will start decelerating in
      // order to help the robot find the object without
      // overshooting or oscillating.
      turnSpeed -= deceleration;
    }
    else
    {
      // An object is not visible, so we will accelerate in order
      // to help find the object sooner.
      turnSpeed += acceleration;
    }

    
  
    turnSpeed = constrain(100, 100, 100);
    
    
    if (objectSeen)
    {
      // An object seen.
      ledYellow(1);
      bool lastTurnRight = turnRight;

      
        scanAround();
        if (sensorValues[0] < sensorValues[1])
        {
          // The right value is greater, so the object is probably
          // closer to the robot's right LEDs, which means the robot
          // is not facing it directly.  Turn to the right to try to
          // make it more even.
          turnRight();
          senseDir = RIGHT;
        }
        else if (sensorValues[0] > sensorValues[1])
        {
          // The left value is greater, so turn to the left.
          turnLeft();
          senseDir = LEFT;
        }
        else
        {
          // The values are equal, so return.
          return;
        }
    }

    
      else
      {
        // No object is seen, so just keep turning in the direction
        // that we last sensed the object.
        ledYellow(0);
        scanAround();
        while(sensorValues[0] < 1 && sensorValues[1] < 1 && sensorValues[2] < 1 && sensorValues[3] < 1){
          if (senseDir == RIGHT)
          {
            turnRight();
          }
          else
          {
            turnLeft();
          }
          scanAround();
        }
      }
  }
}
 



