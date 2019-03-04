/*
  Robot.cpp - Library for all robot functions.
  Created by James Lloyd, February 26, 2019.
*/

#include "Arduino.h"
#include "Robot.h"

bool setup_done = false;

Robot::Robot(const byte whichISR) : whichISR (whichISR)
{
  AFMS = Adafruit_MotorShield();
  leftDriveMotor = AFMS.getMotor(1);
  rightDriveMotor = AFMS.getMotor(2);
  conveyorMotor = AFMS.getMotor(3);
  gripperMotor = AFMS.getMotor(4);
  Serial.begin(9600);
  AFMS.begin();
  optoPin = 2;
  pinMode(optoPin, INPUT_PULLUP);
  currentDist = 0;
  process = 0;
}

void Robot::begin(){
  switch (whichISR)
  {
    case 0:
      attachInterrupt (digitalPinToInterrupt(optoPin), isr0, CHANGE);
      instance0 = this;
      break;
  }
}

void Robot::isr0(){
  instance0 -> distanceCalculator();
}

Robot * Robot::instance0;

void Robot::distanceCalculator()
{
  float conversion = 19.634954; //a change in pulse corresponds to x distance NEED TO WORK OUT x
    currentDist += conversion;
}

void Robot::processCommand(char byte_in)
{
  if (byte_in == CMD_START) {

    Serial.println("Got command");
    while (!Serial.available())
      delay(10);
    byte_in = Serial.read();

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
      Serial.println("Finished moving");
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




//MAY NEED TO MAKE DECCELERATION AND ACCELERATION FASTER. ALSO HAVE CONTROL METHODS
//May be a better way to deccelerate 
void Robot::straightMovement(float distance) { 
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
    Serial.print("Current dist decceleration: ");
    Serial.println(currentDist);
  }

  //Brake
  Serial.println("Brake");
  leftDriveMotor->run(RELEASE);
  rightDriveMotor->run(RELEASE);
  //delay(100);
}

void Robot::turn90(int rotation) { //positive for right negative for left
  float turnDistance = 100; //NEED TO TEST THIS TO FIND HOW MANY ROTATIONS GIVE 90 degress 
  uint8_t i; //used for incrementing speed for acceleration and deceleration
  int turnSpeed = 100; //Maximum speed during turning
  currentDist = 0; 
  float brakeRotation = 200; //For stopping constant velocit and preparing to break

  //if x is negative then we are moving backwards, if x positive -> forward
  if(rotation < 0){
    leftDriveMotor->run(FORWARD); //THESE NEED TO BE CHECKED FOR CORRECT DIR
    rightDriveMotor->run(FORWARD);
  }
  else if(rotation > 0){
     leftDriveMotor->run(BACKWARD);//THESE NEED TO BE CHECKED FOR CORRECT DIR
     rightDriveMotor->run(BACKWARD);
  }
  
  //Acceleration
  Serial.println("Accelerating");
  for (i=0; i<turnSpeed; i++) {
    leftDriveMotor->setSpeed(i);
    rightDriveMotor->setSpeed(i);
    distanceCalculator(); //Update distance since start of this command
    Serial.print("Current dist acceleration: ");
    Serial.println(currentDist);
    //delay(5);
  }

  Serial.println("Constant movement");
  //Constant speed whilst currentDist is less than distance
  while(currentDist <= turnDistance - brakeRotation) {
    distanceCalculator();
    Serial.print("Current dist constant movement: ");
    Serial.println(currentDist);
    //delay(5);
   

  }
  //Deceleration
  Serial.println("Decelerating"); 
  for (i=turnSpeed; i!=0; i--) {
    distanceCalculator();
    Serial.print("Current dist decceleration: ");
    Serial.println(currentDist);
    //delay(5);
  }

  //Brake
  Serial.println("Brake");
  leftDriveMotor->run(RELEASE);
  rightDriveMotor->run(RELEASE);
  delay(100);
}

void setup()
{

  //Robot r(0);
  //r.begin();
}

void loop()
{

  static Robot r(0);
  if (!setup_done) {
    r.begin();
    setup_done = true;
    Serial.println("Set up complete");
    Serial.println();
  }

  if (Serial.available()) {
    char byte_in = Serial.read();
    Serial.print("Got byte: ");
    Serial.println(byte_in);
    r.processCommand(byte_in);
  }
  //delay(100);

}
