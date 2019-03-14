/*
  Robot.cpp - Library for all robot functions.
  Created by James Lloyd, February 26, 2019.
*/

#include "Arduino.h"
#include "Robot.h"
#include <Servo.h>

bool setup_done = false;

Robot::Robot(const byte whichISR) : whichISR (whichISR)
{
  AFMS = Adafruit_MotorShield();
  leftDriveMotor = AFMS.getMotor(1);
  rightDriveMotor = AFMS.getMotor(2);
  conveyorMotor = AFMS.getMotor(3);
  gripperMotor = AFMS.getMotor(4);
  gripperServo.attach(10); 
  backstopServo.attach(9);
  Serial.begin(9600);
  AFMS.begin();
  optoPin = 19; //All interrupts pins chosen correctly (2,3,18,19,20,21)
  hallPin = 3;
  blockdetecPin = 2;
  pinMode(optoPin, INPUT_PULLUP);
  pinMode(hallPin, INPUT);
  pinMode(blockdetecPin, INPUT_PULLUP);
  currentDist = 0;
  logDistance = 0;
  coordinate = 1;
  process = 0;
  blockNo = 0;
  waitingDist = false;
  waitingRot = false;
}

void Robot::begin(){
  switch (whichISR)
  {
    case 0:
      attachInterrupt (digitalPinToInterrupt(optoPin), isr0, CHANGE);
      //attachInterrupt (digitalPinToInterrupt(hallPin), isr1, RISING);
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
  //noInterrupts();
  float conversion = 19.634954; //a change in pulse corresponds to x distance NEED TO WORK OUT x
  currentDist += conversion;
  //delayMicroseconds(1000);
  //interrupts();
}

void Robot::magnetDetection(){
  if(process == 2)
    process = 4;
}

void Robot::blockDetection(){
  process = 2;
}


short Robot::processCommand(char byte_in)
{
  unsigned char byte_1 = byte_in;
  while(!Serial.available())
    delay(0.01);
  char byte_2 = Serial.read();
  short data;
  data = byte_2;
  data <<= 8;
  data |= byte_1;
  Serial.println("Command received");
  return data;
}

void Robot::conveyorIncrement()
{  
  uint8_t i;
  int incrementSpeed = 200;

  conveyorMotor->run(BACKWARD);
  for (i=0; i<incrementSpeed; i++) {
    conveyorMotor->setSpeed(i);
    delay(5);
  }
  for (i=incrementSpeed; i!=0; i--) {
    conveyorMotor->setSpeed(i);
    delay(5);
  }

  conveyorMotor->run(RELEASE);
}

void Robot::unloadConveyor()
{
  uint8_t i;
  int unloadSpeed = 200;
  int unloadTime = 150;

  //Accelerate
  conveyorMotor->run(FORWARD);
  for (i=0; i<unloadSpeed; i++) {
    conveyorMotor->setSpeed(i);
    delay(5);
  }

  for (i=0; i<unloadTime; i++) {
    conveyorMotor->setSpeed(unloadSpeed);
    delay(100);
  }
  
  //Decelerate
  for (i=unloadSpeed; i!=0; i--) {
    conveyorMotor->setSpeed(i);
    delay(5);
  }
  
  //Brake
  conveyorMotor->run(RELEASE);
  delay(100);
}


//MAY NEED TO MAKE DECCELERATION AND ACCELERATION FASTER
//May be a better way to deccelerate 
void Robot::straightMovement(short distance) {
  Serial.print("Forward ");
  Serial.println(distance);
  if(process == 1){    
    uint8_t i; //used for incrementing speed for acceleration and deceleration
    int fullSpeed = 200;
    currentDist = 0;
    float brakeDistance = 60; //For stopping on path and preparing to break
  
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
    for (i=0; i<fullSpeed; i+=10) {
      if (currentDist > abs(distance) - brakeDistance)
        break;
      leftDriveMotor->setSpeed(i*1.06);
      rightDriveMotor->setSpeed(i);
      Serial.print("Current dist acceleration: ");
      Serial.println(currentDist);
    }
  
    Serial.println("Constant movement");

    //Constant speed whilst currentDist is less than distance
    while(currentDist <= abs(distance)-brakeDistance and process == 1) {
    Serial.print("Current dist constant movement: ");
    Serial.println(currentDist);
    }

    //Deceleration
    Serial.println("Decelerating");
    for (i=fullSpeed; i!=0; i-= 20) {
      leftDriveMotor->setSpeed(i*1.06);
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

void Robot::rotate(short rotation) {
  Serial.print("Rotate ");
  Serial.println(rotation);
  float conversion2 = 1.745329; //1.745329 mm = 1 deg turned 
  float distance = abs(conversion2 * rotation) + 40;
  
  if(process == 1){
    uint8_t i; //used for incrementing speed for acceleration and deceleration
    int fullSpeed = 100;
    currentDist = 0;
    float brakeDistance = 5; //For stopping on path and preparing to break
  
    //if x is negative then we are moving backwards, if x positive -> forward
    if(rotation < 0){
      leftDriveMotor->run(FORWARD);
      rightDriveMotor->run(FORWARD);
    }
    else if(rotation > 0){
       leftDriveMotor->run(BACKWARD);
       rightDriveMotor->run(BACKWARD);
    }
    
    //Acceleration
    Serial.println("Accelerating");
    for (i=0; i<fullSpeed; i+=10) {
      if (currentDist > abs(distance) - brakeDistance)
        break;
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      Serial.print("Current angle acceleration: ");
      Serial.println(currentDist/conversion2);
    }
  
    Serial.println("Constant movement");
    //Constant speed whilst currentDist is less than distance
    while(currentDist <= abs(distance)-brakeDistance and process == 1) {
      Serial.print("Current angle constant movement: ");
      Serial.println(currentDist/conversion2);
     
  
    }
    //Deceleration
    Serial.println("Decelerating");
    for (i=fullSpeed; i!=0; i-=10) {
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      Serial.print("Current angle decceleration: ");
      Serial.println(currentDist/conversion2);
    }
  
    //Brake
    Serial.println("Brake");
    leftDriveMotor->run(RELEASE);
    rightDriveMotor->run(RELEASE);
    //delay(100);
  }
}


void Robot::gripBlock(){

    //Close backstop
    Serial.println("Closing backstop");
    int pos1;
    for(pos1 = backstopOpen; pos1 <= backstopClosed; pos1 += 5){ 
    backstopServo.write(pos1);
    delay(5);
  }

    //GETTING ROBOT POSITIONED ABOVE BLOCK 
    uint8_t i; //used for incrementing speed for acceleration and deceleration


    int getBlockSpeed = 200; //need to test this
    leftDriveMotor->run(BACKWARD);
    rightDriveMotor->run(FORWARD);
    Serial.println("Accelerating");
    for (i=0; i<getBlockSpeed; i++) {
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      delay(5); //Need to test this

    }

    Serial.println("Decelerating"); 
    for (i=getBlockSpeed; i!=0; i--) {
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      delay(1); //Need to test this
    }
    
    Serial.println("Brake");
    leftDriveMotor->run(RELEASE);
    rightDriveMotor->run(RELEASE);
    delay(100);
  

  //GRIPPING BLOCK
  int pos;
  Serial.println("Gripping");
  for(pos = startingPos; pos >= gripPos; pos -= 1){ 
    gripperServo.write(pos);
    delay(5);
  }
  //delay(2000); //this is for stabalising the hall effect and finding a value
  
}

void Robot::loadConveyor(){

  //ROTATE GRIPPER ARM
  Serial.println("Moving up");
  uint8_t i;
  int rotateSpeed = 255; //this may be different for returning arm
  int rotateTime = 60;
  gripperMotor->run(BACKWARD);
  for (i=0; i<rotateSpeed; i+=5) {
    gripperMotor->setSpeed(i);
    delay(5); 
  }
  
    for (i=0; i<rotateTime; i++) {
    gripperMotor->setSpeed(rotateSpeed);
    delay(100); 
  }


  for (i=rotateSpeed; i!=0; i-=5) {
    gripperMotor->setSpeed(i);
    delay(5);
  }
  gripperMotor->run(RELEASE); //Need to test how to pause the gripper arm

  //RELEASE BLOCK
  int pos;
  for(pos = gripPos; pos <= startingPos; pos += 1){ 
    gripperServo.write(pos);
    delay(5);
   }

  //REVERSING ROBOT

    int reverseSpeed = 100; //need to test this
    leftDriveMotor->run(FORWARD);
    rightDriveMotor->run(BACKWARD);
    Serial.println("Accelerating");
    for (i=0; i<reverseSpeed; i++) {
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      delay(1); //Need to test this

    }

    Serial.println("Decelerating"); 
    for (i=reverseSpeed; i!=0; i--) {
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      delay(1); //Need to test this
    }
    
    Serial.println("Brake");
    leftDriveMotor->run(RELEASE);
    rightDriveMotor->run(RELEASE);
    delay(100);
   
 //ROTATE GRIPPER ARM
 Serial.println("Moving back");
  gripperMotor->run(FORWARD);
  for (i=0; i<rotateSpeed; i+=5) {
    gripperMotor->setSpeed(i);
    delay(5); 
  }
  
  for (i=0; i<rotateTime; i++) {
    gripperMotor->setSpeed(rotateSpeed);
    delay(80);
  }

  for (i=rotateSpeed; i!=0; i-=5) {
    gripperMotor->setSpeed(i);
    delay(5);
  }
  gripperMotor->run(RELEASE);
}

void Robot::releaseBlock(){ //BLOCK SHOULD BE ABLE TO PASS UNDER WITHOUT MOVING ARM
  //RELEASE BLOCK
  int pos;
  for(pos = gripPos; pos <= startingPos; pos += 1){ 
    gripperServo.write(pos);
    delay(5);
   }
}


void Robot::positionGripperArm(){
   //ROTATE GRIPPER ARM
  uint8_t i;
   Serial.println("putting gripper in 0 position");
  gripperMotor->run(FORWARD);
  for (i=0; i<50; i+=1) {
    gripperMotor->setSpeed(i);
    delay(100);
  }

  for (i=50; i!=0; i-=1) {
    gripperMotor->setSpeed(i);
    delay(5);
  }
  gripperMotor->run(RELEASE);
  
}

void setup()
{
  Serial.println("Starting up");
}

void loop()
{
  static Robot r(0);
  if (!setup_done) {
    r.begin();
    r.process = 0;
    setup_done = true;
  }

  if(r.process == 0){
    r.gripperServo.write(r.startingPos);
    r.backstopServo.write(r.backstopOpen);
 
    
    Serial.println("Set up complete");
    Serial.println("Routing");
    //instructions for planning first route (this may all be moved to setup)
    r.process = 1;
  }

  else if(r.process == 1){
    //here we pring out coordinates for rotating and straightmovement
    Serial.println("Following path");
    short dist = 0;
    short rot = 0;
    if (Serial.available()) {
      char byte_in = Serial.read();
      if (byte_in == CMD_START) {
        //Serial.println("Got command");
        while (!Serial.available())
            delay(10);
        byte_in = Serial.read();
        if (byte_in == CMD_FORWARD) {
          dist = r.processCommand(Serial.read());
          r.straightMovement(dist);
        }
        else if (byte_in == CMD_ROTATE) {
          rot = r.processCommand(Serial.read());
          r.rotate(rot);
        }
      }
    }
  if(r.coordinate == 1){
    r.distance = 2000; //distance to move forward
  }
  else if(r.coordinate == 2){
    r.distance = -30;
  }
  else if(r.coordinate == 3){
    r.distance = 0;
    r.rotate(50); //rotate 80deg
  }
  else if(r.coordinate == 4){
    r.distance = 2700; //distance to move forward
  }
  else if(r.coordinate == 5){
    r.distance = 0;
    r.rotate(-10);
  }
  else if(r.coordinate == 6){
    r.distance = -900;
  }
  else if(r.coordinate == 7){
    r.distance = 0;
    r.rotate(90);
  }
  else if(r.coordinate == 8){
    r.distance = 1200; //Move to middle of table
  }
  else if(r.coordinate == 9){
    r.distance = 0;
    r.rotate(200);
  }
  else if(r.coordinate == 10){
    r.distance = -1200; //reverse up to shelf
  }
  else if(r.coordinate == 11){
    r.distance = 0;
    r.unloadConveyor();
  }
  else if(r.coordinate == 12){
    r.distance = 100;
  }
  else if(r.coordinate == 13){
    r.distance = 0;
    r.rotate(100);
  }
  else if(r.coordinate == 14){
    r.distance = 1000; //go to corner (start position)
  }
  else if(r.coordinate == 15){
    r.distance = -100; //go to corner (start position)
  }

  else if(r.coordinate == 16){
    r.distance = 0; //END
  }

  
  if(r.logDistance <= abs(r.distance) and r.coordinate != 16){ //log Distance is updated after r.gripBlock has been called
    Serial.print("Moved ");
    Serial.print(r.logDistance);
    Serial.print(" of ");
    Serial.print(r.distance);
    if(r.distance > 0){
      r.straightMovement(r.distance - r.logDistance); //move distance still left to move
    }
    else {
      r.straightMovement(r.distance + r.logDistance); //move distance still left to move
    }
    
    if(r.process == 1){ //if block not detected we have made the distance
      r.coordinate +=1; //go onto next coordinate
      r.logDistance = 0; //reset distance for next coordinate
    }
  }
  else if(r.coordinate !=16){ //!= 3 means we haven't ended yet
    r.coordinate +=1;
    r.logDistance = 0; //reset distance for next coordinate
  }
  
  }
  
  
  
  //Block has been detected
  else if(r.process == 2){
    Serial.println("Moving over block");
    //delay(1000); //comment this out (just for testing)
    r.gripBlock();
    r.logDistance += r.currentDist; //update distance moved along path until sensor interrupt is called
    
    delay(1000);
    //OPEN BACKSTOP
    int pos;
    for(pos = r.backstopClosed; pos >= r.backstopOpen; pos -= 5){ 
      r.backstopServo.write(pos);
    delay(5);
  }
    if(r.process == 2) {
      r.process = 3; //if magnet not detected go on to process 3
    }
  }

  //Magnet not detected
  else if(r.process == 3){
    Serial.println("Loading block");
    r.loadConveyor();
    r.conveyorIncrement();
    r.blockNo += 1;
    Serial.print(r.blockNo);
    Serial.println(" blocks");

      r.process = 1; //if all instructions printed at once else to 5/0 and re-route
    
  }

  //Magnet detected
  else if(r.process == 4){
    Serial.println("Releasing magnetic block");
    r.releaseBlock();
    r.process = 1; //if all instructions printed at once else to 5/0 and re-route
  }

}
