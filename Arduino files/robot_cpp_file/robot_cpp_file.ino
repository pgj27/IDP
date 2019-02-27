/*
  Robot.cpp - Library for all robot functions.
  Created by James Lloyd, February 26, 2019.
*/

#include "Arduino.h"
#include "Robot.h"

Robot::Robot()
{
  AFMS = Adafruit_MotorShield();
  conveyorMotor = AFMS.getMotor(1);
  Serial.begin(9600);
  AFMS.begin();
  pinMode(1, INPUT);
}

void Robot::conveyorIncrement()
{  
  uint8_t i;
  int incrementSpeed = 170;

  conveyorMotor->run(FORWARD);
  for (i=0; i<incrementSpeed; i++) {
    conveyorMotor->setSpeed(i);
    delay(10);
  }
  for (i=incrementSpeed; i!=0; i--) {
    conveyorMotor->setSpeed(i);
    delay(10);
  }

  conveyorMotor->run(RELEASE);
  delay(100);
}

void Robot::unloadConveyor()
{
  uint8_t i;
  int unloadSpeed = 255;

  //Accelerate
  conveyorMotor->run(BACKWARD);
  for (i=0; i<unloadSpeed; i++) {
    conveyorMotor->setSpeed(i);
    delay(100);
  }
  
  //Decelerate
  for (i=unloadSpeed; i!=0; i--) {
    conveyorMotor->setSpeed(i);
    delay(10);
  }
  
  //Brake
  conveyorMotor->run(RELEASE);
  delay(100);
}

bool Robot::checkOptoStatus() //Consider moving inside distananceCalculator function
{
  optoPin = 1; //WILL BE DIFFERENT FOR DIGITAL AND THIS IS USED IN MUTIPLE FUNCTIONS
  int cutOff = 100; //This may need to be a global variable if we need to calibrate using a function
                    //Otherwise we don't need this value if we are using digital

  //call optoswitch function for HIGH or LOW value
  bool optoReading;
  //optoReading = digitalRead(optoPin); //if using digital
  int optoReadingAnalog = analogRead(optoPin);

  if(optoReading>cutOff) //only need this if using analog
    optoReading = false;
  else
    optoReading = true;

  return optoReading;
}

void Robot::distanceCalculator()
{

  float conversion = 19.634954; //a change in pulse corresponds to x distance NEED TO WORK OUT x

  int optoReading = checkOptoStatus();
  if(optoCounter != optoReading)
    currentDist += conversion;

  optoCounter = optoReading;
  //delay(10);
}


int main()
{
  Robot robot();
  return 0;
}
