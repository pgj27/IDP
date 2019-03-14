#ifndef Robot_h
#define Robot_h
#define CMD_START 99
#define CMD_FORWARD 102
#define CMD_STEERING 115
#define CMD_ROTATE 114
#define CMD_UNLOAD 117

#include "Arduino.h"
#include "Wire.h"
#include "Adafruit_MotorShield.h"
#include <Servo.h>

class Robot
{


  public:
    Robot();
    short processCommand(char byte_in);
    void loadConveyor();
    void releaseBlock();
    void conveyorIncrement();
    void unloadConveyor();
    void straightMovement(short distance);
    void rotate(short distance);
    void gripBlock();
    void positionGripperArm();
    void turn90(int deg);
    Robot (const byte which);
    void begin ();
    float distance;
    volatile float currentDist;
    float logDistance;
    int coordinate;
    
    volatile int process; //detetmines what the robot should be doing, 0 = setup/scanning route,
                          //1 = following route, 2 = positioning above block and gripping, 3 = storing block on conveyor
                          //4 = releasing a magnetic block, 5 = scanning and re-routing, 6 = unloading
    int blockNo; //for storing the value of blocks stored on the robot
    bool waitingDist;
    bool waitingRot; // Used when waiting for commands
    Servo gripperServo;
    Servo backstopServo;
    int startingPos = 120; //for gripping block (NEED ROUGHLY 90DEG DIFFERENCE)
    int gripPos = 65;    //for gripping block
    int backstopClosed = 90; 
    int backstopOpen = 20;
    Adafruit_DCMotor *gripperMotor;
    
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
