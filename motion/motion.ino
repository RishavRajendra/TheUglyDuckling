#include <QueueArray.h>
#include <Servo.h>
#include "constants.h"

/* Variables */

// Create a servo object
Servo Servo1;
Servo Servo2;
Servo ServoCam;

//Declare queue for commands
QueueArray <byte> queue;

void setup() {
  DDRL = B11111111;
  ud_bot();
  Servo1.attach(servoPin1);
  Servo2.attach(servoPin2);
  Serial.begin(9600);
//  mov(fwd, 12, 500);
//  vars(fwd, 72, 500, 1.005, rightmotion, leftmotion);
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
    int flag = (int)queue.pop();
    byte data1 = queue.pop();
    byte data2 = queue.pop();
    byte data3 = queue.pop();
    
    if (flag == 0){
     mov(data1, data2, 500);
    }
    else if (flag == 1){
      turn(data1, data2, 500);
    }
    else if (flag == 2){
      mov45(data1, data2, 500, data3);
    }
    else if (flag == 3){
      gridMov(data1,data3, 500, data2);
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

//Move using inches
void mov(byte dir, float dist, long del) {
  PORTL = dir;
  float stepf = dist * steps_per_inch;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motion;
  }
}

//Grid movement - 12 inches per call
void gridMov(byte dir, long dist, long del, byte motors) {
  PORTL = dir;
  float stepf = dist * steps_per_inch;
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

//Move 45 degrees in inches
void mov45(byte dir, float dist, long del, byte motors) {
  PORTL = dir;
  float stepf = 2 * dist * steps_per_inch;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motors;
  }
}

void vars(byte dir, float dist, long del, float ratio, byte master, byte slave) {
  PORTL = dir;
  float stepf = dist * steps_per_inch;
  long steps = stepf;

  long masterCounter = 0;
  long slaveCounter = 0;
  long stepCounter = 0;

  float temp = del * ratio;
  long slaveDelay = temp;

  while (stepCounter < steps) {

    if (masterCounter > del) {
      PORTL ^= master;
      masterCounter = 0;
      stepCounter++;
    }

    if (slaveCounter > slaveDelay) {
      PORTL ^= slave;
      slaveCounter = 0;
    }
    masterCounter++;
    slaveCounter++;
  }
}

//void acceleration(byte dir, float dist, long del, int N) {
//  float total_dis = N * (N + 1) / 2 * 3 * start_dis;
//
//  if (total_dis > dist) {
//    float m = sqrt((dist / start_dis) * 2 / 3);
//    N = m;
//  }
//
//  float mid_dis = dist - start_dis * 3 * N * (N + 1) / 2;
//
//  for (int i = 1; i <= N; i++) {
//    vars(dir, start_dis * i, del / i, straight, rightmotion, leftmotion);
//  }
//
//  vars(dir, mid_dis, del / N, straight, rightmotion, leftmotion);
//
//  for (int i = N; i > 0; i--) {
//    vars(dir, start_dis * 2 * i, del / i, straight, rightmotion, leftmotion);
//  }
//}

//Declares directional bytes for UglyDuckling bot
void ud_bot(){
  fwd = B10100000;
  rev = B00001010;
  strl = B00100010;
  strr = B10001000;
  rotl = B00000000;
  rotr = B10101010;
}
