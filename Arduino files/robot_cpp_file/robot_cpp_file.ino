/*
  Robot.cpp - Library for all robot functions.
  Created by James Lloyd, February 26, 2019.
*/

#include "Arduino.h"
#include "Robot.h"

Robot::Robot()
{
  AFMS = Adafruit_MotorShield();
  leftDriveMotor = AFMS.getMotor(1);
  rightDriveMotor = AFMS.getMotor(2);
  conveyorMotor = AFMS.getMotor(3);
  gripperMotor = AFMS.getMotor(4);
  Serial.begin(9600);
  AFMS.begin();
  optoPin = 1;
  pinMode(optoPin, INPUT);
}

void Robot::processCommand(String input)
{
  Serial.println(input);
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


void Robot::distanceCalculator()
{
  optoPin = 8; //WILL BE DIFFERENT FOR ANALOG DIGITAL AND THIS IS USED IN MUTIPLE FUNCTIONS
  float conversion = 19.634954; //a change in pulse corresponds to x distance NEED TO WORK OUT x
 
  int cutOff = 100; //This may need to be a global variable if we need to calibrate using a function
                    //Otherwise we don't need this value if we are using digital

  //call optoswitch function for HIGH or LOW value
  bool optoReading;
  
  optoReading = digitalRead(optoPin); //if using digital
  //int optoReadingAnalog = analogRead(optoPin); //if using analog
  /*if(optoReading>cutOff) //if using analog
    optoReading = false;
  else
    optoReading = true;
*/
  Serial.println(currentDist);
  if(optoCounter != optoReading)
    currentDist += conversion;
    

    optoCounter = optoReading;
  //delay(10); //may make a difference in testing
}
//MAY NEED TO MAKE DECCELERATION AND ACCELERATION FASTER. ALSO HAVE CONTROL METHODS
//May be better way to deccelerate 
void Robot::straightMovement(float distance) { 
  uint8_t i; //used for incrementing speed for acceleration and deceleration
  int fullSpeed = 100;
  currentDist = 0;
  float brakeDistance = 200; //For stopping on path and preparing to break

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
    distanceCalculator(); //Update distance since start of this command
    Serial.print("Current dist acceleration: ");
    Serial.println(currentDist);
    //delay(5);
  }

  Serial.println("Constant movement");
  //Constant speed whilst currentDist is less than distance
  while(currentDist <= distance-brakeDistance) {
    distanceCalculator();
    Serial.print("Current dist constant movement: ");
    Serial.println(currentDist);
    //delay(5);
   

  }
  //Decceleration
  Serial.println("Deccelerating");
  for (i=fullSpeed; i!=0; i--) {
    distanceCalculator();
    Serial.print("Current dist decceleration: ");
    Serial.println(currentDist);
    //delay(5);
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
  //Decceleration
  Serial.println("Deccelerating"); 
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
  Robot r;
  Serial.println("Set up complete");
  //r.straightMovement(600);
  Serial.println();
}

void loop()
{
  if (Serial.available()) {
    char byte_in = Serial.read();
    if (byte_in == CMD_START) {
      Serial.println("Got command");
      byte_in = Serial.read();
      int i = 0;
      
      if (byte_in == CMD_FORWARD) {
        Serial.println("Forward command");
        char dist[10];

        while (byte_in != CMD_END && i < 10) {
          dist[i] = byte_in;
          byte_in = Serial.read();
          i++;
        }

        Serial.print("Forward distance: ");
        for (int n = 0; n < i; n++)
          Serial.print(dist[n]);
        Serial.println();
      }
      else
        Serial.println("Unknown command");
    }
  }
  else
    Serial.println("No byte");
  delay(100);
  //r.processCommand();
}
