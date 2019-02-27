//WHEEL ENCODER

//take inputs from opto switch to determine how far the wheels have turned

//KEEP IN MIND
//The maximum frequency that is possible depends on the other jobs the controller has to do, too.
//Maximum number of pulses (need to consider that the counter can only count up to a certain number(distance))
//Distance between slits should be 2 times distance of slit width of opto switch

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


int checkOptoStatus(int counter) {
  int optoPin = 1; //WILL BE DIFFERENT FOR DIGITAL AND THIS IS USED IN MUTIPLE FUNCTIONS
  int cutOff = 100; //This may need to be a global variable if we need to calibrate using a function
                    //Otherwise we don't need this value if we are using digital

  //call optoswitch function for HIGH or LOW value
  //int optoReading = digitalRead(optoPin); //if using digital
  int optoReading = analogRead(optoPin);

  if(optoReading>cutOff){ //only need this if using analog
    optoReading = 0;
  }
  else if(optoReading<=cutOff){
    optoReading = 1;
  }

  //if counter is same as output then do nothing
  //if counter is different value to output then change value of counter
    if(optoReading==1){
      counter = 1;
    }
    
    else if(optoReading==0){
      counter = 0;
    }
  //delay(10)
  return counter;     
}

float distanceCalculator(float currentDist , int counter){ //both global variables
  int optoPin = 1;
  int cutOff = 100; //This may need to be a global variable if we need to calibrate using a function
                    //We don't need this value if we are using digital
                    
  float conversion = 19.634954; //a change in pulse corresponds to x distance NEED TO WORK OUT x
  

  //call optoswitch function for HIGH or LOW value
  //int optoReading = digitalRead(optoPin); // if using digital
  int optoReading = analogRead(optoPin);
  
   //Serial.print("Optoreading in fist function: ");
   //Serial.println(optoReading);
   if(optoReading>cutOff){ //Only need this for analog
      optoReading = 0;
  }
    else if(optoReading<=cutOff){
      optoReading = 1;
  }

   
   if(counter!=optoReading){
      currentDist += conversion;

  }
  //delay(10);
  return currentDist;       
  
}



void straightMovement(float x) {
  uint8_t i; //used for incramenting speed for acceleration and decceleration
  int counter; //GLOBAL VARIABLE THIS WILL NEED TO BE LOOKED AT
  int fullSpeed = 100;
  float currentDist= 0; //GLOBAL VARIABLE THIS WILL NEED TO BE LOOKED AT
  float brakeDistance = 300; //For stopping on path and preparing to break

  //if x is negative then we are moving backwards, if x positive -> forward
  if(x<0){
    myOtherMotor->run(FORWARD);
    myMotor->run(BACKWARD); 
  }
  else if(x>0){
     myOtherMotor->run(BACKWARD);
     myMotor->run(FORWARD); 
}
  //Acceleration
    Serial.println("Accelerating");
    for (i=0; i<fullSpeed; i++) {
      myOtherMotor->setSpeed(i); 
      myMotor->setSpeed(i);
      currentDist = distanceCalculator(currentDist, counter);
      counter = checkOptoStatus(counter); 
      Serial.print("Current dist acceleration: ");
      Serial.println(currentDist);
      delay(5);
    }

  Serial.println("Constant movement");
  //Constant speed whilst currentdistance is less thatn 
  while(currentDist <= x-brakeDistance){

    currentDist = distanceCalculator(currentDist, counter);
    counter = checkOptoStatus(counter); 
    

   Serial.print("Current dist constant movement: ");
   Serial.println(currentDist);
   delay(5);
    
      }
      
    //Decceleration
    Serial.println("Deccelerating");

    for (i=fullSpeed; i!=0; i--) {
      currentDist = distanceCalculator(currentDist, counter);
      counter = checkOptoStatus(counter); 
      Serial.print("Current dist decceleration: ");
      Serial.println(currentDist);
      delay(5);  
  }

  //Brake

  Serial.println("Brake");
  myMotor->run(RELEASE);
  myOtherMotor->run(RELEASE);
  delay(1000);
}

void setup() {
  AFMS.begin();  // create with the default frequency 1.6KHz
  Serial.begin(9600);
  pinMode(1, INPUT);
  
  //forward(630);
  //calibrate high low values

}


void loop() {
  //int counter = encoder(counter);
  //Serial.println(counter);

  //distance = distanceCalculator(distance, counter);
  //Serial.println(distance);
  //Serial.println();

  


}



