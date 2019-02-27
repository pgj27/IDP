#include <Servo.h>
Servo myservo;
int servoPin = 10;//insert value of servoPin

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



void servo_control(int numBlocks) {
  uint8_t i;
  int incrament = 27; //increment to move conveyor
  int start_pos = 30; //starting position
  int end_pos = 19; //ending position after unloading
  int current_pos = start_pos + (x-1)*incrament;
  int pos;
  
  //blocks loading
  if(numBlocks<6){

  converyorMotor->run(FORWARD);
  for (i=0; i<170; i++) {
    converyorMotor->setSpeed(i);  
    delay(10);
  }
  for (i=170; i!=0; i--) {
    converyorMotor->setSpeed(i);  
    delay(10);
  }
  
  

  Serial.print("tech");
  converyorMotor->run(RELEASE);
  delay(100);
  
    //for(pos = current_pos; pos <= current_pos + incrament; pos +=1){  
    //Serial.println(pos);  
      
    //myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    //delay(100); //Need to calculate delays (if we want 2 seconds then delay time = 2000/incrament)
                    
    }
  

//unloading
  else if(numBlocks==6){

    Serial.print("tock");

    converyorMotor->run(BACKWARD);
    for (i=0; i<255; i++) {
    converyorMotor->setSpeed(i);  
    delay(100);
  }
  
  for (i=255; i!=0; i--) {
    converyorMotor->setSpeed(i);  
    delay(10);
  }

    converyorMotor->run(RELEASE);
    delay(100);
    //for(pos = current_pos; pos >= end_pos; pos -=1){
    //Serial.println(pos); 
    //myservo.write(pos);
    //delay(100);
  }
  

  //Report error
  else if (x>6){ 
    //an error method
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);                       // wait for a second
  } 
}


void setup() {
  Serial.begin(9600);
  AFMS.begin();
  
  //myservo.attach(servoPin);
  pinMode(LED_BUILTIN, OUTPUT); //For error 
  


  for(int i=1; i<10; i++){
    Serial.println(i);
   servo_control(i);
   delay(100);
  }
}

void loop() {

}
