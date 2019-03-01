#ifndef Robot_h
#define Robot_h
#define CMD_START 99
#define CMD_FORWARD 102
#define CMD_END 101

#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_MotorShield.h"

class Robot
{
  public:
    Robot();
    void processCommand(String input);
    void conveyorIncrement();
    void unloadConveyor();
    void distanceCalculator();
    void straightMovement(float distance);
    void turn90(int deg);
    float currentDist;
    String command;
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
