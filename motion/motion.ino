#include <QueueArray.h>
#include <Servo.h>

/* Variables */

//Direction
byte fwd;
byte rev;
byte rotl;
byte rotr;
byte strl;
byte strr;

//Motor Pairs
byte motion = B01010101;
byte leftmotion = B00010001;
byte rightmotion = B01000100;
byte frontmotion = B01010000;
byte rearmotion = B00000101;
byte motion45right = B00010100;
byte motion45left = B01000001;

volatile byte masterWheels;
volatile byte slaveWheels;
volatile float steps;
volatile float steps_counter;
volatile bool flag1;

//Measurements
float distance = 6 * PI; // This is for centimeters.
float steps_per_cm = 3200 / distance;
float steps_per_inch = 3200 / (60 * PI / 25.4);
float steps_per_degree = 23.7;  //Needs to be calibrated
float straight = 1.000;
float arc180 = .3;
float start_dis = .25;
float threshold = 250;
float tile_dist = 12;

// Declare the Servo pin
int servoPin1 = 2;
int servoPin2 = 3;
// Create a servo object
Servo Servo1;
Servo Servo2;
Servo ServoCam;
//Declare positions for servos
int up1 = 0;
int up2 = 180;
int down1 = 160;
int down2 = 20;
//Declare queue for commands
QueueArray <byte> queue;

void setup() {
  DDRL = B11111111;
  ud_bot();
  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2);
  Serial.begin(9600);
}

void loop() {
  readPython();
  runCommand();
}

//Wait for function calls from RaspberryPi
void readPython(){
  if(Serial.available() > 0){
    byte data [4];
    Serial.readBytes(data, 4);
    for(int i = 0; i < 4; i++){
      queue.push(data[i]);
    }
  }
 }
// Run commands waiting in queue
void runCommand(){
  if (!queue.isEmpty()){
    int flag = (int) queue.pop();
    byte data1 = queue.pop();
    byte data2 = queue.pop();
    byte data3 = queue.pop();
    
    if (flag == 0){
     mov(data1, data2, 300);
    }
    else if (flag == 1){
      turn(data1, data2, 300);
    }
    else if (flag == 2){
      mov45(data1, data2, 300, data3);
    }
    else if (flag == 3){
      gridMov(data1, 300, data2);
    }
    //Signal back to RaspberryPi    
    Serial.write(B11111111);
  }
}

/*Servo Code*/

//Move servos to default position
void resetArms(){
  Servo1.write(up1);
  Servo2.write(up2);
}

//Moves into position to pick up object
void armsDown(){
  Servo1.write(down1);
  Servo2.write(down2);
}

//Pick up and hold objects
void pickup(){
  Servo2.write(120);
  delay(15);
  Servo1.write(60);
}

/*Motor Code*/

//Move using centimeters
void mov(byte dir, float dist, long del) {
  PORTL = dir;
  float stepf = dist * steps_per_cm;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motion;
  }
}

//Grid movement - 12 inches per call
void gridMov(byte dir, long del, byte motors) {
  PORTL = dir;
  float stepf = tile_dist * steps_per_inch;
  if (motors != B01010101){
    stepf *= 2;
  }
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motors;
  }
}

//Turn given degrees- Only used with rotl or rotr
void turn(byte dir, float dist, long del) {
  PORTL = dir;
  float stepf = dist * steps_per_degree;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motion;
  }
}

//Move 45 degrees in centimeters
void mov45(byte dir, float dist, long del, byte motors) {
  PORTL = dir;
  float stepf = 2 * dist * steps_per_cm;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motors;
  }
}

//Declares directional bytes for UglyDuckling bot
void ud_bot(){
  fwd = B10100000;
  rev = B00001010;
  strl = B00100010;
  strr = B10001000;
  rotl = B00000000;
  rotr = B10101010;
}
