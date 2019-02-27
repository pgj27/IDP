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
  private:
    Adafruit_MotorShield AFMS;
    Adafruit_DCMotor *conveyorMotor;
};

#endif
