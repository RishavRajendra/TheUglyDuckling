//Direction bytes
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
float distance = (60 * PI)/ 25.4;
float steps_per_inch = 3200 / distance;
float steps_per_degree = 30;
float straight = 0.980;
float arc180 = .3;
float start_dis = .25;
float threshold = 300;
float tile_dist = 12;

// Declare the Servo pin
int servoPin1 = 2;
int servoPin2 = 3;

//Declare positions for servos
int up1 = 0;
int up2 = 180;
int down1 = 160;
int down2 = 20;
