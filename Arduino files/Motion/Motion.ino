/* 
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2 
---->  http://www.adafruit.com/products/1438
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setupMotors() {
            //Add paremters for setting up the motors
            
}


void forward(int x) {
  uint8_t i;
  int fullSpeed = 200;
  int current_dist = 0; //For logging how far we have moved (not being used rn)
  //Need to be able to pass an amount x to move forward (will be paired with wheel encoder to get correct distance)
  myOtherMotor->run(FORWARD);
  myMotor->run(BACKWARD);

  while(current_dist < x){
    
    //Acceleration
    Serial.println("Accelerating forward");
    for (i=0; i<fullSpeed; i++) {
    myOtherMotor->setSpeed(i); 
    myMotor->setSpeed(i);  
    delay(10);
  }

  //Constant movement

  Serial.println("Constant movement forward");
  
    while(current_dist <=x){
    Serial.print(current_dist);
    myOtherMotor->setSpeed(fullSpeed); 
    myMotor->setSpeed(fullSpeed);
    current_dist +=1; 
    delay(100);
    }
    //Decceleration
    Serial.println("Deccelerating forward");
    
    for (i=fullSpeed; i!=0; i--) {
      
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }

  //Brake

  Serial.println("Brake");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
}}

void backward(int x) {
  uint8_t i;
  
  int fullSpeed = 200;
  int current_dist = 0; //For logging how far we have moved (not being used rn)
  
  //Need to be able to pass an amount x to move forward (will be paired with wheel encoder to get correct distance)
    myOtherMotor->run(BACKWARD);
    myMotor->run(FORWARD);
    
    //Acceleration
    Serial.println("Accelerating backward");
    for (i=0; i<fullSpeed; i++) {
    myOtherMotor->setSpeed(i); 
    myMotor->setSpeed(i);  
    delay(10);
  }

  //Constant movement

  Serial.println("Constant movement backward");
  
    while(current_dist <=x){
    Serial.print(current_dist);
    myOtherMotor->setSpeed(fullSpeed); 
    myMotor->setSpeed(fullSpeed);
    current_dist +=1; 
    delay(100);
    }
    //Decceleration
    Serial.println("Deccelerating backward");
    
    for (i=fullSpeed; i!=0; i--) {
      
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }

  //Brake

  Serial.println("Brake");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
  
}

void turnRight(int x) {
    Serial.println("Turning right");
  uint8_t i;
  int turnSpeed = 150;
  int current_dist = 0; //For logging how far we have moved (not being used rn)
  
  //Need to be able to pass an amount x to move forward (will be paired with wheel encoder to get correct distance)
    myOtherMotor->run(BACKWARD);
    myMotor->run(BACKWARD);
    
    //Acceleration
    for (i=0; i<turnSpeed; i++) {
    myOtherMotor->setSpeed(i); 
    myMotor->setSpeed(i);  
    delay(10);
  }

  //Constant movement
  
    while(current_dist <=x){
    Serial.print(current_dist);
    myOtherMotor->setSpeed(turnSpeed); 
    myMotor->setSpeed(turnSpeed);
    current_dist +=1; 
    delay(100);
    }
    
    //DecceleratioN
    for (i=turnSpeed; i!=0; i--) {
      
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }

  //Brake

  Serial.println("Brake");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
}

void turnLeft(int x) {
  Serial.println("Turning left");
  uint8_t i;
  
  int fullSpeed = 155;
  int current_dist = 0; //For logging how far we have moved (not being used rn)
  
  //Need to be able to pass an amount x to move forward (will be paired with wheel encoder to get correct distance)
    myOtherMotor->run(FORWARD);
    myMotor->run(FORWARD);
    
    //Acceleration
    for (i=0; i<fullSpeed; i++) {
    myOtherMotor->setSpeed(i); 
    myMotor->setSpeed(i);  
    delay(10);
  }

  //Constant movement
    while(current_dist <=x){
      
    Serial.print(current_dist);
    myOtherMotor->setSpeed(fullSpeed); 
    myMotor->setSpeed(fullSpeed);
    current_dist +=1; 
    delay(100);
    }
    
    //Decceleration
    for (i=fullSpeed; i!=0; i--) {
      
    myMotor->setSpeed(i);
    myOtherMotor->setSpeed(i);  
    delay(10);
  }

  //Brake
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
}


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  Serial.println("Setup complete, ready to start");

  //delay(2000);
  forward(50);
  //backward(50);
}

void loop() {
}
