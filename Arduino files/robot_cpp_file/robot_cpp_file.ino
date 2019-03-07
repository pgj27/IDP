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
  Serial.begin(9600);
  AFMS.begin();
  optoPin = 19; //All interrupts pins chosen correctly (2,3,18,19,20,21)
  hallPin = 3;
  blockdetecPin = 2;
  pinMode(optoPin, INPUT_PULLUP);
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(blockdetecPin, INPUT_PULLUP);
  currentDist = 0;
  process = 0;
  blockNo = 0;
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
  //noInterrupts();
  float conversion = 19.634954; //a change in pulse corresponds to x distance NEED TO WORK OUT x
  currentDist += conversion;
  //delayMicroseconds(1000);
  //interrupts();
}

void Robot::magnetDetection(){
  process = 4;
}

void Robot::blockDetection(){
  process = 2;
}


short Robot::processCommand(char byte_in, char request)
{
  if (byte_in == CMD_START) {
    //Serial.println("Got command");
    while (!Serial.available())
      delay(10);
    byte_in = Serial.read();

    //Serial.print("Byte in / Request: ");
    //Serial.print(byte_in);
    //Serial.println(request);
    while (byte_in != request) {
      byte_in = Serial.read();
    }
    if (byte_in == request) {
      unsigned char byte_1 = Serial.read();
      char byte_2 = Serial.read();
      short data;
      data = byte_2;
      data <<= 8;
      data += byte_1;
      return data;
    }
    return 0;
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
}

void Robot::unloadConveyor()
{
  uint8_t i;
  int unloadSpeed = 255;
  int unloadTime = 50;

  //Accelerate
  conveyorMotor->run(BACKWARD);
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
  if(process == 1){
    uint8_t i; //used for incrementing speed for acceleration and deceleration
    int fullSpeed = 200;
    currentDist = 0;
    float brakeDistance = 35; //For stopping on path and preparing to break
  
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
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      Serial.print("Current dist acceleration: ");
      Serial.println(currentDist);
    }
  
    Serial.println("Constant movement");

    //Constant speed whilst currentDist is less than distance
    while(currentDist <= abs(distance)-brakeDistance and process == 1) {
    Serial.print("Current dist constant movement: ");
    Serial.println(currentDist);
      short steering = 0;
      //while (!Serial.available())
        //delay(0.01);
      //if (Serial.available())
        //steering = this->processCommand(Serial.read(), CMD_STEERING);
      //leftDriveMotor->setSpeed(fullSpeed / 2 + steering / 100);
      //rightDriveMotor->setSpeed(fullSpeed / 2 - steering / 100);
      /*Serial.println("Steering command");
      Serial.println(steering);
      Serial.print("Left motor speed: ");
      Serial.print(fullSpeed / 2 + steering / 100);
      Serial.print(", Right motor speed: ");
      Serial.println(fullSpeed / 2 - steering / 100);
      //Serial.print("Current dist constant movement: ");
      //Serial.println(currentDist);*/
    }

    //Deceleration
    Serial.println("Decelerating");
    for (i=fullSpeed; i!=0; i-= 10) {
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

void Robot::rotate(short distance) { 
  if(process == 1){
    uint8_t i; //used for incrementing speed for acceleration and deceleration
    int fullSpeed = 100;
    currentDist = 0;
    float brakeDistance = 5; //For stopping on path and preparing to break
  
    //if x is negative then we are moving backwards, if x positive -> forward
    if(distance < 0){
      leftDriveMotor->run(FORWARD);
      rightDriveMotor->run(FORWARD);
    }
    else if(distance > 0){
       leftDriveMotor->run(BACKWARD);
       rightDriveMotor->run(BACKWARD);
    }
    
    //Acceleration
    Serial.println("Accelerating");
    for (i=0; i<fullSpeed; i+=10) {
      leftDriveMotor->setSpeed(i);
      rightDriveMotor->setSpeed(i);
      Serial.print("Current dist acceleration: ");
      Serial.println(currentDist);
    }
  
    Serial.println("Constant movement");
    //Constant speed whilst currentDist is less than distance
    while(currentDist <= abs(distance)-brakeDistance) {
      Serial.print("Current dist constant movement: ");
      Serial.println(currentDist);
     
  
    }
    //Deceleration
    Serial.println("Decelerating");
    for (i=fullSpeed; i!=0; i-=10) {
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
    }

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
  

  //GRIPPING BLOCK
  int pos;
  Serial.println("Gripping");
  for(pos = startingPos; pos <= gripPos; pos += 1){ 
    gripperServo.write(pos);
    delay(5);
  }
  delay(2000); //this is for stabalising the hall effect and finding a value
  
}

void Robot::loadConveyor(){

  //ROTATE GRIPPER ARM
  Serial.println("Moving up");
  uint8_t i;
  int rotateSpeed = 255; //this may be different for returning arm
  int rotateTime = 40;
  gripperMotor->run(FORWARD);
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
  for(pos = gripPos; pos >= startingPos; pos -= 1){ 
    gripperServo.write(pos);
    delay(5);
   }
   
 //ROTATE GRIPPER ARM
 Serial.println("Moving back");
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
  gripperMotor->run(RELEASE);
}

void Robot::releaseBlock(){ //BLOCK SHOULD BE ABLE TO PASS UNDER WITHOUT MOVING ARM
  //RELEASE BLOCK
  int pos;
  for(pos = gripPos; pos >= startingPos; pos -= 1){ 
    gripperServo.write(pos);
    delay(5);
   }
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
    Serial.println("Set up complete");
    r.process = 0;
    setup_done = true;
  }

  if(r.process == 0){
    Serial.println("Routing");
    //instructions for planning first route (this may all be moved to setup)
    r.process = 1;
  }

  else if(r.process == 1){
    Serial.println("Following path");
    //delay(5000);
    r.straightMovement(600);
    
    r.rotate(160);
    
    r.straightMovement(300);
    
    r.rotate(160);
    
    r.straightMovement(600);
    
    r.rotate(160);
    
    r.straightMovement(300);
    
    r.rotate(160);
    
    
    /*if (Serial.available()) {
      short dist = r.processCommand(Serial.read(), CMD_FORWARD);
      Serial.println("Distance received");
      Serial.println(dist);
      r.straightMovement(dist);
      delay(1000);
      
    }*/
  
    if(r.process == 1) {
      r.process = 5; //if block not found revert to 5 and re-route
    }
  }
  
  
  
  //Block has been detected
  else if(r.process == 2){
    Serial.println("Moving over block");
    //delay(1000); //comment this out (just for testing)
    r.gripBlock();
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
    if(r.blockNo == 5){ //are there a set number of non magnetic blocks
      r.process = 6; //go back to shelf
    }
    else{
      r.process = 1; //if all instructions printed at once else to 5/0 and re-route
    }
  }

  //Magnet detected
  else if(r.process == 4){
    Serial.println("Releasing magnetic block");
    r.releaseBlock();
    r.process = 1; //if all instructions printed at once else to 5/0 and re-route
  }

  else if(r.process == 5){
    Serial.println("Re-routing");
    //instructions to re-route (this may be done under 1)
    r.process = 1;
  }

  else if(r.process == 6){
    Serial.println("Unloading conveyor");
    //instructions to get back to shelf
    r.unloadConveyor();
    r.process =1;
  }

  
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
