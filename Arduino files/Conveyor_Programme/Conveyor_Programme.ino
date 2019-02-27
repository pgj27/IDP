
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *converyorMotor = AFMS.getMotor(1);




//Call function to run motor and increment conveyor belt
void conveyorIncrament() {
  uint8_t i;
  int incramentSpeed = 170;
  
  converyorMotor->run(FORWARD);
  for (i=0; i<incramentSpeed; i++) {
    converyorMotor->setSpeed(i);  
    delay(10);
  }
  for (i=incramentSpeed; i!=0; i--) {
    converyorMotor->setSpeed(i);  
    delay(10);
  }
  
  converyorMotor->run(RELEASE);
  delay(100);
                    
    }
  
//Call function to run motor and unload coneyor belt
void unloadConveyor(){
    uint8_t i;
    int unloadSpeed = 255;

    //Accelerate
    converyorMotor->run(BACKWARD);
    for (i=0; i<unloadSpeed; i++) {
    converyorMotor->setSpeed(i);  
    delay(100);
  }
  //Deccelerate
  for (i=unloadSpeed; i!=0; i--) {
    converyorMotor->setSpeed(i);  
    delay(10);
  }
    //Brake
    converyorMotor->run(RELEASE);
    delay(100);
  }



void setup() {
  Serial.begin(9600);
  AFMS.begin();

  }


void loop() {

}
