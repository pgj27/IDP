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
  private:
    Adafruit_MotorShield AFMS;
    Adafruit_DCMotor *conveyorMotor;
    int optoPin;
    bool optoCounter;
    float currentDist;
};

#endif
