/* Variables */

byte fwd;
byte rev;
byte rotl;
byte rotr;
byte strl;
byte strr;

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

float distance = 6 * PI;
float steps_per_cm = 3200 / distance;
//float steps_per_degree = 23.7  for test bot
float steps_per_degree = 29.5;
float straight = 1.000;
float arc180 = .3;
float start_dis = .25;
float threshold = 250;

void setup() {
  DDRL = B11111111;
  ud_bot();
  Serial.begin(9600);
  delay(2000);
  mov45(rev, 20, 250, motion45right);
}

void loop() {
//  readPython();
}

void readPython(){
  if(Serial.available() > 0){
    byte data [4];
    Serial.readBytes(data, 4);
    int flag = (int) data[0];
    if (flag == 0){
     mov(data[1], data[2], 500);
    }
    else if (flag == 1){
      turn(data[1], data[2], 500);
    }
    else if (flag == 2){
      mov45(data[1], data[2], 500, data[3]);
    }
  }
}

void mov(byte dir, float dist, long del) {
  PORTL = dir;
  float stepf = dist * steps_per_cm;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motion;
    //      PORTL^= motion;
  }
}

void turn(byte dir, float dist, long del) {
  PORTL = dir;
  float stepf = dist * steps_per_degree;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motion;
    //      PORTL^= motion;
  }
}

void arc(byte dir, float dist, long del, byte motors) {
  PORTL = dir;
  float stepf = 2 * dist * steps_per_degree;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motors;
    //      PORTL^= motion;
  }
}

//  Requires rotational direction
void pivot(byte dir, float dist, long del, byte motors) {
  PORTL = dir;
  float stepf = 2 * dist * steps_per_degree;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motors;
    //      PORTL^= motion;
  }
}

void mov45(byte dir, float dist, long del, byte motors) {
  PORTL = dir;
  float stepf = 2 * dist * steps_per_cm;
  long steps = stepf;
  for (long i = 0; i < steps; i++) {
    delayMicroseconds(del);
    PORTL ^= motors;
    //      PORTL^= motion;
  }
}

void vars(byte dir, float dist, long del, float ratio, byte master, byte slave) {
  PORTL = dir;
  float stepf = dist * steps_per_cm;
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

void acceleration(byte dir, float dist, long del, int N) {
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



void varsInt(byte dir, float dist, long del, float ratio, byte master, byte slave) {
  ctc3_setup();
  ctc4_setup();
  PORTL = dir;
  float stepf = dist * steps_per_cm;
  masterWheels = master;
  slaveWheels = slave;

  noInterrupts();
  OCR4A = 16 * del;
  TCNT4 = 0;
  float temp = del * ratio;
  long slaveDelay = temp;
  OCR3A = slaveDelay * 16;
  TCNT3 = 0;
  steps = stepf;
  steps_counter = 0;
  interrupts();
}
void varsIntTurn(byte dir, float dist, long del, float ratio, byte master, byte slave) {
  ctc3_setup();
  ctc4_setup();
  PORTL = dir;
  float stepf = dist * steps_per_degree;
  masterWheels = master;
  slaveWheels = slave;

  noInterrupts();
  OCR4A = 16 * del;
  TCNT4 = 0;
  float temp = del * ratio;
  long slaveDelay = temp;
  OCR3A = slaveDelay * 16;
  TCNT3 = 0;
  steps = stepf;
  steps_counter = 0;
  interrupts();
}

void ctc4_setup() {
  noInterrupts();
  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;

  OCR4A = 16000;

  TCCR4B |= (1 << WGM42);
  TCCR4B |= (1 << CS40);
  TIMSK4 |= (1 << OCIE4A);
  flag1 = true;
  interrupts();
}

void ctc3_setup() {
  noInterrupts();
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;

  OCR3A = 16000;
  TCCR3B |= (1 << WGM32);
  TCCR3B |= (1 << CS30);
  TIMSK3 |= (1 << OCIE3A);
  interrupts();
}

//********************Set upt ISR
//3: Slave
ISR(TIMER3_COMPA_vect) { // timer compare ISR
  if (steps > 0) {
    PORTL ^= slaveWheels;
    //PORTL ^= slaveWheels;
    steps_counter++;
  }
}

//4: Master
ISR(TIMER4_COMPA_vect) { // timer compare ISR
  if (flag1) {
    if (steps > 0) {
      PORTL ^= masterWheels;
      //PORTL ^= masterWheels;
      steps--;
      if (steps == 0) flag1 = false;
    }
  }
}
//********************End Set upt ISR

void kbt() {
  rotl = B00100010;
  rotr = B10001000;
  strr = B01000001;
  strl = B10101010;
  fwd = B10000010;
  rev = B00101000;

  rightmotion = B00000101;
  leftmotion = B01010000;
  frontmotion = B00010001;
  rearmotion = B01000100;
  motion45right = B00010100;
  motion45left = B01000001;
}

void ud_bot(){
  fwd = B00100010;
  rev = B10001000;
  strl = B00001010;
  strr = B10100000;
  rotl = B00000000;
  rotr = B10101010;
}

void test_bot(){
  fwd = B10001000;
  rev = B00100010;
  strl = B10100000;
  strr = B00001010;
  rotl = B10101010;
  rotr = B00000000;

  motion45right =B00010100;
}

void cal() {
  int i, j;
  long sum, mean;
  float inc, dist;
  inc = 1.0;
  for (i = 0; i < 61; i++) {
    sum = 0;
    dist = i * inc;
    for (j = 0; j < 500; j ++) {
      sum += analogRead(A4);
    }
    mean = sum / j;
    Serial.print(" Mean = ");
    Serial.print(mean);
    Serial.print(" Dist = ");
    Serial.println(dist);
    mov(fwd, inc, 2000);
    delay(250);
  }
}

void scan() {
  varsIntTurn(rotl, 90, 500, 1.0, rightmotion, leftmotion);
  while (steps > 0)
    Serial.println(analogRead(A0));
}

//    Targeting

void findTarget() {
  long a, i;
  bool lastEdge = false;
  long firstEdge;
  long secondEdge;
  long width;
  long center;
  float centerAngle;
  float distToTarget;
  long sum = 0;

  varsIntTurn(rotl, 90, 500, 1.0, rightmotion, leftmotion);

  while (steps > 0) {
    a = analogRead(A0);
    if (a < threshold && lastEdge == true) {
      secondEdge = steps;
      lastEdge = false;
    }
    if (a > threshold && lastEdge == false) {
      firstEdge = steps;
      lastEdge = true;
    }
  }
  width = secondEdge - firstEdge;
  center = (secondEdge + firstEdge) / 2;

  varsIntTurn(rotr, center / steps_per_degree, 500, 1.0, rightmotion, leftmotion);
  while ( steps > 0) {
    ;
  }
  for (i = 0 ; i < 500; i++) {
    sum += analogRead(A0);
  }
  distToTarget = sum / i;
  distToTarget = 37991*pow(distToTarget,-1.3914);
  mov(fwd, distToTarget, 500);
}
