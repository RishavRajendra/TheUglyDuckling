#include <QueueArray.h>
#include "constants.h"

/* Variables */

//Declare queue for commands
QueueArray <byte> queue;

void setup() {
  DDRL = B11111111;
  ud_bot();
  Serial.begin(9600);
  reset_servo();
  send_sensor_data();
//  acceleration(fwd, 24, 350, 8, motion);
  turn(rotl, 90, 400, 0);
  mov45(rev, 3, 400, motion45right);
//  mov(fwd, 10, 400);
}

void loop() {
//  readPython();
//  runCommand();
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
     mov(data1, data2, 400);
    }
    else if (flag == 1){
      turn(data1, data2, 700, data3);
    }
    else if (flag == 2){
      acceleration(data1, data2, 350, 8, data3);
    }
    else if (flag == 3){
      pickup();
    }
    else if (flag == 4){
      drop();
    }
    else if (flag == 5){
      reset_servo();
    }
    else if (flag == 6){
      cam_up();
    }
    else if (flag == 7){
      cam_down();
    }
    else if(flag == 8){
      send_sensor_data();
    }
    //Signal back to RaspberryPi    
//    Serial.write(B11111111);
  }
}

/*Servo Code*/

void pickup(){
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  int i = 1850;

  digitalWrite(3, HIGH);
  delayMicroseconds(i);
  digitalWrite(3, LOW);
  delay(10);

  digitalWrite(2, HIGH);
  delayMicroseconds(3000 - i);
  digitalWrite(2, LOW);
  delay(10);
}

void drop(){
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  int i = 2500;

  digitalWrite(3, HIGH);
  delayMicroseconds(i);
  digitalWrite(3, LOW);
  delay(10);

  digitalWrite(2, HIGH);
  delayMicroseconds(3000 - i);
  digitalWrite(2, LOW);
  delay(10);
}

void reset_servo(){
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  int i = 500;

  digitalWrite(3, HIGH);
  delayMicroseconds(i);
  digitalWrite(3, LOW);
  delay(10);

  digitalWrite(2, HIGH);
  delayMicroseconds(2999 - i);
  digitalWrite(2, LOW);
  delay(10);
}

void cam_up(){
  
}

void cam_down(){
  
}

////Move servos to default position
//void resetArms(){
//  Servo1.write(up1);
//  Servo2.write(up2);
//}
//
////Moves into position to pick up object
//void armsDown(){
//  Servo1.write(down1);
//  Servo2.write(down2);
//}
//
////Pick up and hold objects
//void pickup(){
//  Servo2.write(180);
//  delay(15);
//  Servo1.write(0);
//}

/* Sensor Code */

void send_sensor_data(){
  float val = 0.0;
  for (int i = 0; i < 5; i++){
    val = analogRead(sensorpin);
    val = calscale * pow(val, calpower);
  }
  float sum = 0;
  for (int i = 0; i<5; i++){
    val = analogRead(sensorpin);
    val = calscale * pow(val, calpower);
    sum += val;
  }
  int avg = sum/5; 
  Serial.write(avg);
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
  if (motors != motion){
    stepf *= 2;
  }
  if (dir == strr || dir == strl){
    stepf = dist * steps_per_inch_strafe;
  }
  
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motion;
  }
}

//Turn given degrees- Only used with rotl or rotr
void turn(byte dir, float dist, long del, byte flag) {
//  if( flag == 1){
//    dist/camera_degree_ratio;
//  }
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

void acceleration(byte dir, float dist, long del, int N, byte flag) {
//  if (flag == 0){
//    dist *= diagonal_dist;
//  }
  float total_dis = N * (N + 1) / 2 * 3 * start_dis;

  if (total_dis > dist) {
    float m = sqrt((dist / start_dis) * 2 / 3);
    N = m;
  }

  float mid_dis = dist - start_dis * 3 * N * (N + 1) / 2;

  for (int i = 1; i <= N; i++) {
    vars(dir, start_dis * i, del / i, straight, rightmotion, leftmotion);
  }

  vars(dir, mid_dis, del / N, straight, rightmotion, leftmotion);

  for (int i = N; i > 0; i--) {
    vars(dir, start_dis * 2 * i, del / i, straight, rightmotion, leftmotion);
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
