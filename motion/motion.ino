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
//  pickup();
  cam_up();
//  mov(strr, 15, 500);
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
     mov(data1, data2, 400);
    }
    else if (flag == 1){
      turn(data1, data2, 700, data3);
    }
    else if (flag == 2){
      int n = 8;
      if (data2 < 36){
        n = 4; 
      }
      acceleration(data1, data2, 350, n, data3);
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
  int speed = 2500;
  int steps = 13;
  int pin = 6;
  int highDelay = speed;
  int lowDelay = 20000 - steps;
  pinMode(pin, OUTPUT);
  for (int cnt = 0; cnt < steps; cnt++){
    digitalWrite(pin, LOW);
    delayMicroseconds(lowDelay);
    digitalWrite(pin, HIGH);
    delayMicroseconds(highDelay);
  }
}

void cam_down(){
  int speed = 250;
  int steps = 13;
  int pin = 6;
  int highDelay = speed;
  int lowDelay = 20000 - steps;
  pinMode(pin, OUTPUT);
  for (int cnt=0; cnt < steps; cnt++){
    digitalWrite(pin, LOW);
    delayMicroseconds(lowDelay);
    digitalWrite(pin, HIGH);
    delayMicroseconds(highDelay);
  }
}


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

void updateFacingAngle(byte dir, float dist) {
  Serial.print("facingAngle updated from ");
  Serial.print(facingAngle);
  
  if (dir == rotl){
    facingAngle += dist;
  }
  else if (dir == rotr){
    facingAngle -= dist;
  }
  while (facingAngle <= -180){ //see if you can replace these with facingAngle = fixDegrees(facingAngle);
    facingAngle += 360;
  }
  while (facingAngle > 180){
    facingAngle -= 360;
  }

void ctc4_setup() {
  noInterrupts();
  TCCR4A = 0;  // clear counter control register
  TCCR4B = 0;
  TCNT4 = 0;

  OCR4A = 16000; // compare match register – 1000 usecond delay
          // countCompareValue = delayInMicroseconds * 16
          // countCompareValue = 16000000 / prescaler / desired frequency
  TCCR4B |= (1 << WGM42); // count to compare mode
  TCCR4B |= (1 << CS40); // 1 prescaler
  TIMSK4 |= (1 << OCIE4A); // enable timer compare interrupt
  interrupts();
}

void ctc3_setup() {
  noInterrupts();
  TCCR3A = 0;  // clear counter control register
  TCCR3B = 0;
  TCNT3 = 0;
 // OCR3A = 16000; // compare match register – 1000 usecond delay
          // countCompareValue = delayinmicroseconds * 16 
          // countCompareValue = 16000000 / prescaler / desired frequency
  TCCR3B |= (1 << WGM32); // count to compare mode
  TCCR3B |= (1 << CS30); // 1 prescaler
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt
  interrupts();
}
//3: Slave
ISR(TIMER3_COMPA_vect) { // timer compare ISR
  downLookValueLeft = digitalRead(downLookLeft);
  if (steps > 0 && downLookValueLeft == 0) {
    PORTL ^= slaveWheels;
//    PORTL ^= slaveWheels;
  }
  steps_counter++;
}

//4: Master
ISR(TIMER4_COMPA_vect) { // timer compare ISR
  downLookValueRight = digitalRead(downLookRight);
  if (steps > 0 && downLookValueRight == 0) {
    PORTL ^= masterWheels;
//    PORTL ^= masterWheels;
  }

  steps--;
}

void varsIntTurn(byte dir, float dist, long del, float ratio, byte master, byte slave) {
  ctc3_setup();
  ctc4_setup();
  PORTL = dir;
  updateFacingAngle(dir, dist); //update global Facing Angle
  float stepf = dist*steps_per_degree;
  masterWheels = master;
  slaveWheels = slave;

  noInterrupts();
  //OCR4A = 16 * del - 1;//setup speed for master
  OCR4A = 16 * del;
  TCNT4 = 0;//reset
  float temp = del * ratio;
  long slaveDelay = temp;
  //OCR3A = slaveDelay * 16 - 1;//setup speed for slave 
  OCR3A = slaveDelay * 16;
  TCNT3 = 0;//reset
  steps = stepf;
  steps_counter = 0;  //reset step counter
  interrupts();

}

void edgeAlign() {
  for (int i = 3; i > 0; i--) { //decrementing loop scales the forward motion to be a smaller approach each time
    varsInt(rev, i * 8, 1000, straight, rightmotors, leftmotors);

    while (steps > 0) {
      if (downLookValueLeft == 1 && downLookValueRight == 1) {
        steps = 0;
      }
    }
    
    delay(500);
    mov(rev, 2, 1000);
    delay(500);
  }
}

void varsInt(byte dir, float dist, long del, float ratio, byte master, byte slave) {
  updateXandY(dir, dist);
  ctc3_setup();
  ctc4_setup();
  PORTL = dir;
  float stepf = dist*steps_per_inch;
  masterWheels = master;
  slaveWheels = slave;

  noInterrupts();
  //OCR4A = 16 * del - 1;//setup speed for master
  OCR4A = 16 * del;
  TCNT4 = 0;//reset
  float temp = del * ratio;
  long slaveDelay = temp;
  //OCR3A = slaveDelay * 16 - 1;//setup speed for slave 
  OCR3A = slaveDelay * 16;
  TCNT3 = 0;//reset
  steps = stepf;
  steps_counter = 0;  //reset step counter
  interrupts();
}
