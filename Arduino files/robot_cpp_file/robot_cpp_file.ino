/*
  Robot.cpp - Library for all robot functions.
  Created by James Lloyd, February 26, 2019.
*/

#include "Arduino.h"
#include "Robot.h"
#include <Servo.h>

Robot::Robot(const byte whichISR) : whichISR (whichISR)
{
  AFMS = Adafruit_MotorShield();
  leftDriveMotor = AFMS.getMotor(1);
  rightDriveMotor = AFMS.getMotor(2);
  conveyorMotor = AFMS.getMotor(3);
  Servo gripperServo;
  gripperServo.attach(9); 
  Serial.begin(9600);
  AFMS.begin();
  optoPin = 2; //All interrupts pins chosen correctly (2,3,18,19,20,21)
  hallPin = 3;
  //blockdetecPin = 18;
  
  pinMode(optoPin, INPUT_PULLUP);
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(blockdetecPin, INPUT_PULLUP);
  currentDist = 0;
  process = 0;
}

void Robot::begin(){
  switch (whichISR)
  {
    case 0:
      attachInterrupt (digitalPinToInterrupt(optoPin), isr0, CHANGE);
      attachInterrupt (digitalPinToInterrupt(hallPin), isr1, RISING);
      attachInterrupt (digitalPinToInterrupt(blockdetecPin), isr2, RISING);
      instance0 = this;
      break;
  }
}

Robot * Robot::instance0; // for use by ISR glue routines

void Robot::isr0(){
  instance0 -> distanceCalculator();
}

void Robot::isr1(){
  instance0 -> magnetDetection();
}

void Robot::isr2(){
  instance0 -> blockDetection();
}


void Robot::distanceCalculator(){
  float conversion = 19.634954; //a change in pulse corresponds to x distance NEED TO WORK OUT x
    currentDist += conversion;
}

void Robot::magnetDetection(){
  process = 4;
}

void Robot::blockDetection(){
  process = 2;
}



void Robot::processCommand(char byte_in)
{
  if (byte_in == CMD_START) {

    Serial.print("Got command: ");
    while (!Serial.available())
      delay(10);
    byte_in = Serial.read();
    Serial.println(byte_in);

    if (byte_in == CMD_FORWARD) {
      unsigned char byte_1 = Serial.read();
      char byte_2 = Serial.read();
      short dist;
      dist = byte_2;
      dist <<= 8;
      dist += byte_1;
      Serial.print("Forward distance: ");
      Serial.println(dist);
      this->straightMovement(dist);
    }
    else
      Serial.println("Unknown command");
  }
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




//MAY NEED TO MAKE DECCELERATION AND ACCELERATION FASTER
//May be a better way to deccelerate 
void Robot::straightMovement(float distance) { 
  if(process == 1){
    uint8_t i; //used for incrementing speed for acceleration and deceleration
    int fullSpeed = 100;
    currentDist = 0;
    float brakeDistance = 300; //For stopping on path and preparing to break
  
    //if x is negative then we are moving backwards, if x positive -> forward
    if(distance < 0){
      leftDriveMotor->run(FORWARD);
      rightDriveMotor->run(BACKWARD);
    }
    else if(distance > 0){
       leftDriveMotor->run(BACKWARD);
       rightDriveMotor->run(FORWARD);
    }
    
    //Acceleration
    Serial.println("Accelerating");
    for (i=0; i<fullSpeed; i++) {
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      Serial.print("Current dist acceleration: ");
      Serial.println(currentDist);
    }
  
    Serial.println("Constant movement");
    //Constant speed whilst currentDist is less than distance
    while(currentDist <= distance-brakeDistance) {
      Serial.print("Current dist constant movement: ");
      Serial.println(currentDist);
     
  
    }
    //Deceleration
    Serial.println("Decelerating");
    for (i=fullSpeed; i!=0; i--) {
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      Serial.print("Current dist decceleration: ");
      Serial.println(currentDist);
    }
  
    //Brake
    Serial.println("Brake");
    leftDriveMotor->run(RELEASE);
    rightDriveMotor->run(RELEASE);
    //delay(100);
  }
}


void Robot::gripBlock(){

    //GETTING ROBOT POSITIONED ABOVE BLOCK 
    uint8_t i; //used for incrementing speed for acceleration and deceleration
    int getBlockSpeed = 100; //need to test this
    Serial.println("Accelerating");
    for (i=0; i<getBlockSpeed; i++) {
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      delay(5); //Need to test this

    Serial.println("Decelerating"); 
    for (i=getBlockSpeed; i!=0; i--) {
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      delay(5); //Need to test this
    }
    Serial.println("Brake");
    leftDriveMotor->run(RELEASE);
    rightDriveMotor->run(RELEASE);
    delay(100);
  }

  //GRIPPING BLOCK
  //int startingPos 
  //for(
  
}

void setup()
{

  Robot r(0);
  r.begin();
  Serial.println("Set up complete");
  r.process = 1;
  r.straightMovement(600);
  r.straightMovement(100);
  Serial.println();
}

void loop()
{
/*
  static Robot r(0);
  if (Serial.available()) {

    char byte_in = Serial.read();
    Serial.print("Got byte: ");
    Serial.println(byte_in);
    r.processCommand(byte_in);
  }
  //delay(100);*/

}
