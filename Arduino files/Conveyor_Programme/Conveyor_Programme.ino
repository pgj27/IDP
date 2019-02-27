
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *converyorMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

/*Need to move servo to specific position depending on how many
blocks have been picked up, if we set up paremeter as to how many
blocks there are this will determine the position the servo=>the conveyor needs to be.
Once we have got 5 blocks stored we need to wait until the robot is by the shelf, 
then unload (back to 0 position)*/



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
