#include <QueueArray.h>
#include "constants.h"

/* Variables */

//Declare queue for commands
QueueArray <byte> queue;

void setup() {
  DDRL = B11111111;
  ud_bot();
  Serial.begin(9600);
  drop();
  //  pickup();
  cam_up();
  //  mov(strr, 15, 500);
  //  edgeAlign(); //reverse align to edge 3 times by Ceraso
}

void loop() {
  //objectID();
  //Serial.println(analogRead(rearLookLong)); //test the fwdLooking mid range sensors and rev looking long range sensor
  //readPython();
  //runCommand();
}

//Wait for function calls from RaspberryPi
void readPython() {
  if (Serial.available() > 0) {
    byte data [4];
    Serial.readBytes(data, 4);
    for (int i = 0; i < 4; i++) {
      queue.push(data[i]);
    }
  }
}
// Run commands waiting in queue
void runCommand() {
  if (!queue.isEmpty()) {
    int flag = (int)queue.pop();
    byte data1 = queue.pop();
    byte data2 = queue.pop();
    byte data3 = queue.pop();

    if (flag == 0) {
      mov(data1, data2, 400);
    }
    else if (flag == 1) {
      turn(data1, data2, 700, data3);
    }
    else if (flag == 2) {
      int n = 8;
      if (data2 < 36) {
        n = 4;
      }
      acceleration(data1, data2, 350, n, data3);
    }
    else if (flag == 3) {
      pickup();
    }
    else if (flag == 4) {
      drop();
    }
    else if (flag == 5) {
      reset_servo();
    }
    else if (flag == 6) {
      cam_up();
    }
    else if (flag == 7) {
      cam_down();
    }
    else if (flag == 8) {
      send_sensor_data();
    }
    //Signal back to RaspberryPi
    //    Serial.write(B11111111);
  }
}

/*Servo Code*/

void pickup() {
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

void drop() {
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

void reset_servo() {
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

void cam_up() {
  int speed = 2500;
  int steps = 13;
  int pin = 6;
  int highDelay = speed;
  int lowDelay = 20000 - steps;
  pinMode(pin, OUTPUT);
  for (int cnt = 0; cnt < steps; cnt++) {
    digitalWrite(pin, LOW);
    delayMicroseconds(lowDelay);
    digitalWrite(pin, HIGH);
    delayMicroseconds(highDelay);
  }
}

void cam_down() {
  int speed = 250;
  int steps = 13;
  int pin = 6;
  int highDelay = speed;
  int lowDelay = 20000 - steps;
  pinMode(pin, OUTPUT);
  for (int cnt = 0; cnt < steps; cnt++) {
    digitalWrite(pin, LOW);
    delayMicroseconds(lowDelay);
    digitalWrite(pin, HIGH);
    delayMicroseconds(highDelay);
  }
}


/* Sensor Code */

void send_sensor_data() {
  float val = 0.0;
  for (int i = 0; i < 5; i++) {
    val = analogRead(sensorpin);
    val = calscale * pow(val, calpower);
  }
  float sum = 0;
  for (int i = 0; i < 5; i++) {
    val = analogRead(sensorpin);
    val = calscale * pow(val, calpower);
    sum += val;
  }
  int avg = sum / 5;
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
  if (motors != motion) {
    stepf *= 2;
  }
  if (dir == strr || dir == strl) {
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
void ud_bot() {
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

  if (dir == rotl) {
    facingAngle += dist;
  }
  else if (dir == rotr) {
    facingAngle -= dist;
  }
  while (facingAngle <= -180) { //see if you can replace these with facingAngle = fixDegrees(facingAngle);
    facingAngle += 360;
  }
  while (facingAngle > 180) {
    facingAngle -= 360;
  }
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
  downLookValueRight = digitalRead(downLookRight);
  if (steps > 0 && downLookValueRight == 0) {
    PORTL ^= slaveWheels;
    //    PORTL ^= slaveWheels;
  }
  steps_counter++;
}

//4: Master
ISR(TIMER4_COMPA_vect) { // timer compare ISR
  downLookValueLeft = digitalRead(downLookLeft);
  if (steps > 0 && downLookValueLeft == 0) {
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
  float stepf = dist * steps_per_degree;
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
    varsInt(rev, i * 8, 1000, straight, cerasoright, cerasoleft); //back up to the edge; varsInt lets you read sensors while you move

    while (steps > 0) { //0 out the remaining steps of varsInt when both sensors are off the board (downLookValues == 1)
      if (downLookValueLeft == 1 && downLookValueRight == 1) {
        steps = 0;
      }
    }

    delay(500);
    mov(fwd, 2, 1000); //move back fwd and do the backup edgeAlign again
    delay(500);
  }
}

void varsInt(byte dir, float dist, long del, float ratio, byte master, byte slave) {
  //updateXandY(dir, dist); you might need this function to keep track of your X and Y position throughout the round - Ceraso
  ctc3_setup();
  ctc4_setup();
  PORTL = dir;
  float stepf = dist * steps_per_inch;
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

byte objectID() {
  byte type = 0;
  long sum_right, sum_left, avg_right, avg_left, dist_right, dist_left;
  int i;

  sum_right = 0;
  sum_left = 0;
  for (i = 0; i < 25; i++) {
    sum_right += analogRead(fwdLookRight);
    sum_left += analogRead(fwdLookLeft);
  }
  avg_right = sum_right / i;
  avg_left = sum_left / i;
  dist_right = calscale * pow(avg_right, calpower);
  dist_left = calscale * pow(avg_left, calpower);

  if (dist_right < 14 && dist_left < 14) { //these inch values should be calibrated 
    type = 1;
    Serial.println("objectID saw a mothership");
  }
  else {
    type = 0;
    Serial.println("objectID did not see an object");
  }
  
  type = type / i;
  return type;

}

void align() {
  int a, b, cnt;
  Serial.println("AlignmentMap");
  alignmentMap();
  Serial.println("calcTheoAngles");
  calcTheoAngles(xPos, yPos, facingAngle);
  Serial.println("validatePosts");
  cnt = validatePosts();
  if (cnt > 2) {
    Serial.println("Calc Quadratic");
    if (cnt == 4) {
      calcQuadratic(2, 3);
    }
    if (cnt == 3) {
      for (int i = 0; i < 4; i++) {
        if (realTheta[i] != -1 && realTheta[(i + 1) % 4] != -1) {
          a = i;
          b = (i + 1) % 4;
          Serial.print(a);
          Serial.print("\t");
          Serial.println(b);
        }
      }
      calcQuadratic(a, b);
    }
  }
}

void alignmentMap() {
  int dist = 450;
  
  //Lower Mid
  float centerAngleM;
  bool laststateM = false;
  long sumA = 0;
  long thresholdM = 250;
  long maxDistM = 0;
  objectcountM = 0;

  //Lower Long
  float centerAngleLL;
  bool laststateLL = false;
  long sumB = 0;
  long thresholdLL = 400;
  long maxDistLL = 0;
  objectcountLL = 0;

  //Upper Long
  float centerAngleUL;
  bool laststateUL = false;
  long sumC = 0;
  long thresholdUL = 400;
  long maxDistUL = 0;
  objectcountUL = 0;

  int i;
  long centersteps[20];

  varsIntTurn(rotl, dist, 2000, 1.0, leftmotion, rightmotion); //begin turning

  //determine initial conditions
  long a = analogRead(A1);
  long b = analogRead(A4);
  long c = analogRead(A8);
  if (a < thresholdM)
    laststateM = false;
  if (b < thresholdLL)
    laststateLL = false;
  if (c < thresholdUL)
    laststateUL = false;

  //take reads while turning
  while (steps > 0) {
    sumA = 0;
    sumB = 0;
    sumC = 0;

    for (i = 0; i < 50; i++) {
      sumA += analogRead(A0);
      sumB += analogRead(A4);
      sumC += analogRead(A8);
    }
    a = sumA / i;
    b = sumB / i;
    c = sumC / i;

    //Lower Mid Data Storage
    if (laststateM && a > maxDistM)
      maxDistM = a;
    if (a > thresholdM && !laststateM) {
      firstEdgeM[objectcountM] = steps;
      laststateM = true;
    }
    if (a < thresholdM && laststateM) {
      secondEdgeM[objectcountM] = steps;
      centerM[objectcountM] = dist*steps_per_degree - (firstEdgeM[objectcountM] + secondEdgeM[objectcountM]) / 2;
      widthM[objectcountM] = (firstEdgeM[objectcountM] - secondEdgeM[objectcountM]);
      laststateM = false;

      distToTargetM[objectcountM] = calScale * pow(maxDistM,calPower);
      if (widthM[objectcountM] > 60 && widthM[objectcountM] < 200 && distToTargetM[objectcountM] < 14) {
        objectcountM++;
        maxDistM = 0;
      }
    }

    //Lower Long Data Storage
    if (laststateLL && b > maxDistLL)
      maxDistLL = b;
    if (b > thresholdLL && !laststateLL) {
      firstEdgeLL[objectcountLL] = steps;
      laststateLL = true;
    }
    if (b < thresholdLL && laststateLL) {
      secondEdgeLL[objectcountLL] = steps;
      centerLL[objectcountLL] = dist*steps_per_degree - (firstEdgeLL[objectcountLL] + secondEdgeLL[objectcountLL]) / 2;
      widthLL[objectcountLL] = (firstEdgeLL[objectcountLL] - secondEdgeLL[objectcountLL]);
      laststateLL = false;

      if (widthLL[objectcountLL] > 170) {
        distToTargetLL[objectcountLL] = calScaleLong * pow(maxDistLL, calPowerLong);
        objectcountLL++;
        maxDistLL = 0;
      }
    }

    //Upper Long Data Storage
    if (laststateUL && c > maxDistUL)
      maxDistUL = c;
    if (c > thresholdUL && !laststateUL) {
      firstEdgeUL[objectcountUL] = steps;
      laststateUL = true;
    }
    if (c < thresholdUL && laststateUL) {
      secondEdgeUL[objectcountUL] = steps;
      centerUL[objectcountUL] = dist*steps_per_degree - (firstEdgeUL[objectcountUL] + secondEdgeUL[objectcountUL]) / 2;
      centerstepsUL[objectcountUL] = centerUL[objectcountUL];
      widthUL[objectcountUL] = (firstEdgeUL[objectcountUL] - secondEdgeUL[objectcountUL]);
      laststateUL = false;

      if (widthUL[objectcountUL] > 20) {
        distToTargetUL[objectcountUL] = calScaleUpperLong * pow(maxDistUL, calPowerUpperLong);
        if (widthUL[objectcountUL] > 40 && widthUL[objectcountUL] < 110) {
          objectcountUL++;
        }
        maxDistUL = 0;
      }
    }
    delay(50);
  }

  //correctFacingAngle();
  //calspd(); //calibrate steps per degree


  //convert degrees to radians
  //Lower Mid
  for (i = 0; i < objectcountM; i++) {
    centerM[i] /= steps_per_degree;
    //  center[i] -= 180;
    centerM[i] *= (pi / 180);
    centerM[i] = fixRadians(centerM[i]);
  }
  //Lower Long
  for (i = 0; i < objectcountLL; i++) {
    centerLL[i] /= steps_per_degree;
    centerLL[i] -= 180;
    centerLL[i] *= (pi / 180);
    centerLL[i] = fixRadians(centerLL[i]);
  }
  //Upper Long
  for (i = 0; i < objectcountUL; i++) {
    centerUL[i] /= steps_per_degree;
    centerUL[i] -= 180;
    centerUL[i] *= (pi / 180);
    centerUL[i] = fixRadians(centerUL[i]);
  }

  //Display Mid Range Sensor Data
  Serial.println("Mid Range Sensor");
  Serial.print("Objects = ");
  Serial.print("\t");
  Serial.println(objectcountM);
  Serial.print("Width");
  Serial.print("\t");
  Serial.print("Center");
  Serial.print("\t");
  Serial.print("distToTarget");
  Serial.print("\t");
  Serial.println("obstacle");
  Serial.println(" ");
  for (i = 0; i < objectcountM; i++) {
    Serial.print(widthM[i]);
    Serial.print("\t");
    Serial.print(centerM[i]);
    Serial.print("\t");
    Serial.print(distToTargetM[i]);
    Serial.print("\t");
    Serial.print("\t");
    Serial.println(objObstacleM[i]);
  }

  //Display Lower Long Range Sensor Data
  Serial.println(" ");
  Serial.println("Lower Long Range Sensor");
  Serial.print("Objects = ");
  Serial.print("\t");
  Serial.println(objectcountLL);
  Serial.print("Width");
  Serial.print("\t");
  Serial.print("Center");
  Serial.print("\t");
  Serial.println("distToTarget");
  Serial.println(" ");
  for (i = 0; i < objectcountLL; i++) {
    Serial.print(widthLL[i]);
    Serial.print("\t");
    Serial.print(centerLL[i]);
    Serial.print("\t");
    Serial.println(distToTargetLL[i]);
  }

  //Display Upper Long Range Sensor Data
  Serial.println(" ");
  Serial.println("Upper Long Range Sensor");
  Serial.print("Objects = ");
  Serial.print("\t");
  Serial.println(objectcountUL);
  Serial.print("Width");
  Serial.print("\t");
  Serial.print("Center");
  Serial.print("\t");
  Serial.println("distToTarget");
  Serial.println(" ");
  for (i = 0; i < objectcountUL; i++) {
    Serial.print(widthUL[i]);
    Serial.print("\t");
    Serial.print(centerUL[i]);
    Serial.print("\t");
    Serial.println(distToTargetUL[i]);
  }
}

int validatePosts() {
  Serial.println("");
  int postNum = 0;
  float rad;
  int cnt = 0;
  int validPosts[4] = { -1, -1, -1, -1};
  for (int i = 0; i < objectcountUL; i++) {
    rad = (centerstepsUL[i] / steps_per_degree - 180) * DEG_TO_RAD;
    rad = fixRadians(rad);
    Serial.print("obj = ");
    Serial.println(i);
    Serial.print("actual radians = ");
    Serial.println(rad);
    for (int j = 0; j < 4; j++) {
      Serial.print("comparing to = ");
      Serial.println(CA[j]);
      if (abs(rad - CA[j]) < 0.3) {
        Serial.print("This is a valid post stored in ");
        if (rad < -1.07) postNum = 2;
        if (rad < 0 && rad > -1.07) postNum = 3;
        if (rad < 1.07 && rad > 0) postNum = 0;
        if (rad > 1.07) postNum = 1;
        if (validPosts[2] != -1 and postNum == 2) {
          cnt--;
        }
        validPosts[postNum] = i;
        Serial.println(postNum);
        cnt++;

      }
    }
  }
  Serial.print("valid posts = ");
  Serial.println(cnt);
  Serial.println("");
  if (cnt == 4) {
    for (int i = 0; i < 4; i++) {
      realTheta[i] = fixRadians(centerUL[validPosts[(i + 1) % 4]] - centerUL[validPosts[(i)]]);
    }
  }
  if (cnt == 3) {
    for (int i = 0; i < 4; i++) {
      if (validPosts[(i + 1) % 4] != -1 && validPosts[i] != -1) {
        realTheta[i] = fixRadians(centerUL[validPosts[(i + 1) % 4]] - centerUL[validPosts[(i)]]);
      }
      else {
        realTheta[i] = -1;
      }
    }
  }
  Serial.println("Real Theta");
  for (int i = 0; i < 4; i++) {
    Serial.print(realTheta[i]);
    Serial.print("\t");
  }
  Serial.println("");
  return cnt;
}


void calcQuadratic(int index0, int index1) {
  //using the angle between the corner posts, create circles where we are on a point of the circle.
  Serial.print(realTheta[index0]);
  Serial.print("\t");
  Serial.println(realTheta[index1]);
  float d[4], R[4], radius[4], Cx[4], Cy[4];
  d[index0] = 48 * tan((pi - realTheta[index0]) / 2);
  d[index1] = 48 * tan((pi - realTheta[index1]) / 2);
  R[index0] = 48 * cos(pi - realTheta[index0]) / sin(pi - realTheta[index0]);
  R[index1] = 48 * cos(pi - realTheta[index1]) / sin(pi - realTheta[index1]);
  radius[index0] = R[index0] + d[index0];
  radius[index1] = R[index1] + d[index1];


  switch (index0) {
    case 0:
      Cx[index0] = 48;
      Cy[index0] = 96 + R[index0];
      break;

    case 1:
      Cx[index0] = -R[index0];
      Cy[index0] = 48;
      break;

    case 2:
      Cx[index0] = 48;
      Cy[index0] = -R[index0];
      break;

    case 3:
      Cx[index0] = 96 + R[index0];
      Cy[index0] = 48;
      break;
  }

  switch (index1) {
    case 0:
      Cx[index1] = 48;
      Cy[index1] = 96 + R[index1];
      break;

    case 1:
      Cx[index1] = -R[index1];
      Cy[index1] = 48;
      break;

    case 2:
      Cx[index1] = 48;
      Cy[index1] = -R[index1];
      break;

    case 3:
      Cx[index1] = 96 + R[index1];
      Cy[index1] = 48;
      break;
  }

  //based on the circle centers and radii calculate the x and y coordinates
  float distance, len, height, px2, py2, x1, x2, y1, y2;
  distance = sqrt((Cx[index1] - Cx[index0]) * (Cx[index1] - Cx[index0]) + (Cy[index1] - Cy[index0]) * (Cy[index1] - Cy[index0]));
  len = ((radius[index0] * radius[index0]) - (radius[index1] * radius[index1]) + distance * distance) / (2 * distance);
  height = sqrt(radius[index0] * radius[index0] - len * len);
  px2 = Cx[index0] + len * (Cx[index1] - Cx[index0]) / distance;
  py2 = Cy[index0] + len * (Cy[index1] - Cy[index0]) / distance;
  x1 = 96 - px2 + height * (Cy[index1] - Cy[index0]) / distance;
  x2 = 96 - px2 - height * (Cy[index1] - Cy[index0]) / distance;
  y1 = py2 + height * (Cx[index1] - Cx[index0]) / distance;
  y2 = py2 - height * (Cx[index1] - Cx[index0]) / distance;

  Serial.println(x1);
  Serial.println(x2);
  Serial.println(y1);
  Serial.println(y2);
  Serial.println(" ");
  if (x1 > 4 && x1 < 92) {
    xPos = x1;
  }
  if (x2 > 4 && x2 < 92) {
    xPos = x2;
  }
  if (y1 > 4 && y1 < 92) {
    yPos = y1;
  }
  if (y2 > 4 && y2 < 92) {
    yPos = y2;
  }
  Serial.print(xPos);
  Serial.print("\t");
  Serial.println(yPos);
}

void calcTheoAngles(float xp, float yp, float angle) {
  float theta01, theta02, theta11, theta12, theta21, theta22, theta31, theta32;

  theta01 = atan((xp) / (96 - yp));
  theta02 = atan((96 - xp) / (96 - yp));
  theta11 = atan((96 - yp) / (96 - xp));
  theta12 = atan(yp / (96 - xp));
  theta21 = atan((96 - xp) / yp);
  theta22 = atan((xp) / yp);
  theta31 = atan(yp / (xp));
  theta32 = atan((96 - yp) / (xp));

  angle = angle * DEG_TO_RAD;

  theta[0] = theta01 + theta02;
  theta[1] = theta11 + theta12;
  theta[2] = theta21 + theta22;
  theta[3] = theta31 + theta32;
  CA[0] = theta02 + angle;
  CA[1] = CA[0] + theta[1];
  CA[2] = CA[1] + theta[2];
  CA[3] = CA[2] + theta[3];
  Serial.println("Calculated Angles to Posts:");
  for (int i = 0; i < 4; i++) {
    CA[i] = fixRadians(CA[i]);
    Serial.print(CA[i]);
    Serial.print("\t");
  }
  Serial.println("");
  Serial.println("Calculated Angles btwn Posts");
  Serial.print(theta[0]);
  Serial.print("\t");
  Serial.print(theta[1]);
  Serial.print("\t");
  Serial.print(theta[2]);
  Serial.print("\t");
  Serial.println(theta[3]);
  Serial.println(" ");
}

float fixRadians(float input) {
  while (input < -pi) input += 2 * pi;
  while (input > pi) input -= 2 * pi;
  return input;
}
