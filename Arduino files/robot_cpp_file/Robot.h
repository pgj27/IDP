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
    void gripBlock();
    void turn90(int deg);
    String command;
    Robot (const byte which);
    void begin ();
    volatile float currentDist;
    
    volatile int process; //detetmines what the robot should be doing, 0 = setup/scanning route,
                          //1 = following route, 2 = positioning above block and gripping, 3 = storing block on conveyor
                          //4 = releasing a magnetic block, 5 = scanning and re-routing, 6 = unloading
    
  private:
    static void isr0(); //intermediate ISRs for directing to actual ISRs
    static void isr1();
    static void isr2();
    void distanceCalculator(); //ISR for updating distance when opto switch is triggered
    void blockDetection(); //ISR for updating the process when block detector is triggered
    void magnetDetection();//ISR for updating the process when hall effect is triggered
    const byte whichISR; //Do we need this? check
    static Robot * instance0; //For directing inermediate ISRs to actual ISRs
    Adafruit_MotorShield AFMS;
    Adafruit_DCMotor *conveyorMotor; 
    Adafruit_DCMotor *leftDriveMotor;
    Adafruit_DCMotor *rightDriveMotor;
    int optoPin; //Opto switch pin (input)
    int hallPin; //Hall effect sensor pin (input)
    int blockdetecPin; //Block detector pin (input)
    int gripperServoPin; //Pin to gripper servo (output)

};

#endif
