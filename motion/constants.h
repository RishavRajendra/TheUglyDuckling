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
float diagonal_dist = 1.40;
float camera_degree_ratio = 1.20;
float distance = (60 * PI)/ 25.4;
float steps_per_inch = 3200 / distance;
float steps_per_inch_strafe = 3200 / (56 * PI) / 25.4;
float steps_per_degree = 30;
float straight = 0.980;
float arc180 = .3;
float start_dis = .25;
float threshold = 300;
//Change log
// [0.0.1] Benji
// removed turn_offset and tile_dist; not used anymore 

int sensorpin = A1;
float calscale = 59994.0;
float calpower = -1.494;
