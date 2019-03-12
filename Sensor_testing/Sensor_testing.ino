const byte hallPin = 3; 
const byte blockDetPin = 2;
const byte encoder = 19;
int blockVal;
int hallVal;
volatile int process=0;
int distance=0;


     
void setup() {
  Serial.begin(9600);
  pinMode(hallPin, INPUT_PULLUP);
  pinMode(blockDetPin, INPUT_PULLUP);
  pinMode(encoder, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), magnetDetection, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder), encoderWork, RISING);
  attachInterrupt(digitalPinToInterrupt(blockDetPin), blockDetection, RISING);
}

void loop() {
     if(process == 0){
      delay(1000);
      Serial.println("Set-up complete");
      process = 1;
     }
     
     else if(process == 1){
     Serial.println("Following route");
     forward();
     forward();
     forward();
     if(process ==1){
      process = 5;
      }
     }

     else if(process == 2){
      Serial.println("Positioning above block");
      delay(3000);
      Serial.println("Gripping block");
      delay(3000);
      if(process == 2){
        process = 3;
      }
     }

     else if(process == 3){
      Serial.println("Putting block on conveyor");
      delay(3000);
      process = 1;
     }

     else if(process == 4){
      Serial.println("Releasing magnetic block");
      delay(3000);
      process = 1;
     }

     else if(process == 5){
      Serial.println("Scanning and re-routing");
      delay(3000);
      process = 1;
     }
     
}
void forward(){
    if(process == 1){
      Serial.println("Moving forward");
      int i;
      for (i=0; i<10; i +=1) {
        if(process == 1){
        delay(500);
        Serial.println(distance);
        }
      }
      delay(1000);
    }
}

void blockDetection(){
     Serial.println("Block detected");
     process = 2;
     /*In here we need to call a function that moves the robot 
     forward enough to position the block between the gripper
     and then grip the block (ready for magnet Detection)*/
    
}

void magnetDetection(){
     //noInterrupts();
     process = 4;
     Serial.println("Magnet detected");
     //delayMicroseconds(1000); 
     //interrupts();
     /*as this is called the gripper will be gripping the block, 
      so if it is activated the gripper must release the block
     */
}

void encoderWork(){
     //noInterrupts();
     distance+=10;
     //delayMicroseconds(1000); 
     //interrupts();
     /*as this is called the gripper will be gripping the block, 
      so if it is activated the gripper must release the block
     */
}

/*if the magnet detection interrupt does not occur after block detection is finished
 then we must continue with rotating the gripper arm, releasing the block, 
 incramenting the conveyor whilst (or after/before), rotating the gripper arm
 into its 0 position, then continuing on path to next block*/
