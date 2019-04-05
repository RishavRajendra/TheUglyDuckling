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
byte cerasoright = B01010000;
byte cerasoleft = B00000101;
byte frontmotion = B01010000;
byte rearmotion = B00000101;
byte motion45right = B00010100;
byte motion45left = B01000001;

volatile byte masterWheels;
volatile byte slaveWheels;
volatile float steps;
volatile float steps_counter;
volatile bool flag1;
volatile byte downLookValueLeft; //down looking IR sensor flags
volatile byte downLookValueRight;

const byte downLookRight = 38;
const byte downLookLeft = 39;
const byte fwdLookRight = A0;
const byte fwdLookLeft = A1;
const byte rearLookLong = A2;

//Where I think I am //Ceraso
float facingAngle = 0.0;
float xPos = 54; 
float yPos = 54;

//Measurements
float diagonal_dist = 1.40;
float camera_degree_ratio = 1.20;
float distance = (60 * PI)/ 25.4;
float steps_per_inch = 800 / distance;
float steps_per_inch_strafe = 3200 / (56 * PI) / 25.4;
float steps_per_degree = 29.87;
float straight = 0.980;
float arc180 = .3;
float start_dis = .25;
float threshold = 300;
//Change log
// [0.0.1] Benji
// removed turn_offset and tile_dist; not used anymore 

int sensorpin = A1;
float calscaleRight = 325694.0;
float calpowerRight = -1.796;
float calscaleLeft = 26334.0;
float calpowerLeft = -1.371;
float calscale = 59994.0;
float calpower = -1.494;

//-------------CHARLIE CONSTANTS---------------//
const int objcnt = 30;
float realTheta[4] = {-1, -1, -1, -1}; //real angles between the corner posts
float theta[4] = {}; //theoretical angles between corner posts
int objectcountUL = 0;
long firstEdgeUL[objcnt], secondEdgeUL[objcnt], widthUL[objcnt],  centerstepsUL[objcnt];
float centerUL[objcnt], distToTargetUL[objcnt];
float xobjUL[objcnt], yobjUL[objcnt];
const float calScaleUpperLong = 5854042.18;
const float calPowerUpperLong = -1.91;
const float pi = 3.14159265;
float CA[4] = {}; //theoritical angle locations of corner posts
