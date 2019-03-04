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
    void processCommand(char byte_in);
    void conveyorIncrement();
    void unloadConveyor();
    void straightMovement(float distance);
    void turn90(int deg);
    String command;
    Robot (const byte which);
    void begin ();
    volatile float currentDist;
    volatile int process;
    
  private:
    static void isr0();
    static void isr1();
    static void isr2();
    const byte whichISR;
    static Robot * instance0;
    Adafruit_MotorShield AFMS;
    Adafruit_DCMotor *conveyorMotor;
    Adafruit_DCMotor *leftDriveMotor;
    Adafruit_DCMotor *rightDriveMotor;
    Adafruit_DCMotor *gripperMotor;
    int optoPin;
    int gripperServoPin;
    bool optoCounter;
    void distanceCalculator();
     

};

#endif
