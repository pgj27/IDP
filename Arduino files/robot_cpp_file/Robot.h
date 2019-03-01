#ifndef Robot_h
#define Robot_h

#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_MotorShield.h"

class Robot
{
  public:
    Robot();
    void conveyorIncrement();
    void unloadConveyor();
    void distanceCalculator();
    void straightMovement(float distance);
    void turn90(int deg);
    float currentDist;
  private:
    Adafruit_MotorShield AFMS;
    Adafruit_DCMotor *conveyorMotor;
    Adafruit_DCMotor *leftDriveMotor;
    Adafruit_DCMotor *rightDriveMotor;
    Adafruit_DCMotor *gripperMotor;
    int optoPin;
    int gripperServoPin;
    bool optoCounter;

};

#endif
