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

void setup() {
  DDRL = B11111111;
  ud_bot();
  Serial.begin(9600);
}

void loop() {
  readPython();
}

//Wait for function calls from RaspberryPi
void readPython(){
  if(Serial.available() > 0){
    byte data [4];
    Serial.readBytes(data, 4);
    int flag = (int) data[0];
    if (flag == 0){
     mov(data[1], data[2], 300);
    }
    else if (flag == 1){
      turn(data[1], data[2], 300);
    }
    else if (flag == 2){
      mov45(data[1], data[2], 300, data[3]);
    }
    else if (flag == 3){
      gridMov(data[1], 300, data[2]);
    }
    //Signal back to RaspberryPi    
    Serial.write(B11111111);
  }
 }

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
