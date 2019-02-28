//wheel encoder
//take inputs from opto switch to determine how far the wheels have turned
//need to first test that the output of the opto switch is always correct when wheel is in motion
//Once this is working we can work out how far the wheel has moved by its dimensions given how many 
//times the opto switch output has changed state (between 2 changes in state gives x cm moved)

//KEEP IN MIND
//The maximum frequency that is possible depends on the other jobs the controller has to do, too.
//Maximum number of pulses (need to consider that the counter can only count up to a certain number(distance)
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


float encoder(int counter) { 
   //Serial.print("Counter before: ");
   //Serial.println(counter);

  //call optoswitch function for HIGH or LOW value
  //int optoReading = digitalRead(8);
  int optoReading = analogRead(1);

  if(optoReading>100){
    optoReading = 0;
  }
  else if(optoReading<=100){
    optoReading = 1;
  }
   //Serial.print("Optoreading in second function: ");
   //Serial.println(optoReading);

  //if counter is same as output then do nothing
  //if counter is different value to output then change value of counter and incrament distance by conversion
  //return distance 8  if(counter != optoReading){
    if(optoReading==1){
      counter = 1;
    }
    
    else if(optoReading==0){
      counter = 0;
    }
  
  //Serial.print("Counter after: ");
  //Serial.println(counter);
 // delay(10); //SEE IF DELAY MAKES DIFFERENCE
  return counter;     
}

float distanceCalculator(float distance, int counter){
  
  int highLowValue;
  float conversion = 19.634954; //a change in pulse corresponds to x distance NEED TO WORK OUT x
  

  //call optoswitch function for HIGH or LOW value
  //int optoReading = digitalRead(8);
  int optoReading = analogRead(1);
  
   //Serial.print("Optoreading in fist function: ");
   //Serial.println(optoReading);
   if(optoReading>100){
      optoReading = 0;
  }
    else if(optoReading<=100){
      optoReading = 1;
  }
   
   if(counter!=optoReading){
      distance = distance + conversion;

  }
  //delay(10);
  return distance;       
  
}



void forward(float x) {
  uint8_t i;
  int counter;
  int fullSpeed = 100;
  float current_dist= 0; //For logging how far we have moved
  float brakeDistance = 300; //For stopping on path and preparing to break experiment to find value

  
  //Need to be able to pass an amount x to move forward (will be paired with wheel encoder to get correct distance)
  myOtherMotor->run(FORWARD);
  myMotor->run(BACKWARD);

  //Acceleration
    Serial.println("Accelerating forward");
    for (i=0; i<fullSpeed; i++) {
      myOtherMotor->setSpeed(i); 
      myMotor->setSpeed(i);
      current_dist = distanceCalculator(current_dist, counter);
      counter = encoder(counter); 
      Serial.print("current dist acceleration: ");
      Serial.println(current_dist);
      delay(5);  
    }

  Serial.println("Constant movement forward");
  
  while(current_dist <= x-brakeDistance){

    current_dist = distanceCalculator(current_dist, counter);
    counter = encoder(counter); 
    

   Serial.print("current dist constant movement: ");
   Serial.println(current_dist);
   delay(5);
    
      }
      
    //Decceleration
    Serial.println("Deccelerating forward");

    for (i=fullSpeed; i!=0; i--) {
      current_dist = distanceCalculator(current_dist, counter);
      counter = encoder(counter); 
      Serial.print("current dist decceleration: ");
      Serial.println(current_dist);
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
  
  forward(630);
  //calibrate high low values

}


void loop() {
  //int counter = encoder(counter);
  //Serial.println(counter);

  //distance = distanceCalculator(distance, counter);
  //Serial.println(distance);
  //Serial.println();

  


}



