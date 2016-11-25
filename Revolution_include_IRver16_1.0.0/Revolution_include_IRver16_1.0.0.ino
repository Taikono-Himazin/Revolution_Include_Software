#include <Wire.h>

///////////////////////////
////Kazuki Ngayama 2016////
///////////////////////////

#define IR0 8
#define IR1 9
#define IR2 10
#define IR3 11
#define IR4 12
#define IR5 13
#define IR6 14
#define IR7 15
#define IR8 16
#define IR9 1
#define IR10 2
#define IR11 3
#define IR12 4
#define IR13 5
#define IR14 6
#define IR15 7//赤外線センサ

#define LINE 10//ラインセンサ　この後光センサを組み合わせるがそれはMainで

#define TIMEOUT 1000


void setup() {
  Serial.begin(9600);
  Wire.onRequest(requestEvent);
  Wire.begin(9);

  pinMode(IR0, INPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);
  pinMode(IR6, INPUT);
  pinMode(IR7, INPUT);
  pinMode(IR8, INPUT);
  pinMode(IR9, INPUT);
  pinMode(IR10, INPUT);
  pinMode(IR11, INPUT);
  pinMode(IR12, INPUT);
  pinMode(IR13, INPUT);
  pinMode(IR14, INPUT);
  pinMode(IR15, INPUT);

}

uint16_t IR_Force = 0, IR_Degree = 0;
uint8_t LINE_status = 0;

void loop() {

  IR_read(&IR_Force, &IR_Degree);
  LINE_read();

}

void LINE_read() {
  if (digitalRead(LINE) == HIGH) {
    LINE_status = 1;
  } else {
    LINE_status = 0;
  }
}

void IR_read(uint16_t *f, uint16_t *d) {
  uint16_t IR_val[16];
  IR_val[0] = pulseIn(IR0, LOW, TIMEOUT);
  IR_val[1] = pulseIn(IR1, LOW, TIMEOUT);
  IR_val[2] = pulseIn(IR2, LOW, TIMEOUT);
  IR_val[3] = pulseIn(IR3, LOW, TIMEOUT);
  IR_val[4] = pulseIn(IR4, LOW, TIMEOUT);
  IR_val[5] = pulseIn(IR5, LOW, TIMEOUT);
  IR_val[6] = pulseIn(IR6, LOW, TIMEOUT);
  IR_val[7] = pulseIn(IR7, LOW, TIMEOUT);
  IR_val[8] = pulseIn(IR8, LOW, TIMEOUT);
  IR_val[9] = pulseIn(IR9, LOW, TIMEOUT);
  IR_val[10] = pulseIn(IR10, LOW, TIMEOUT);
  IR_val[11] = pulseIn(IR11, LOW, TIMEOUT);
  IR_val[12] = pulseIn(IR12, LOW, TIMEOUT);
  IR_val[13] = pulseIn(IR13, LOW, TIMEOUT);
  IR_val[14] = pulseIn(IR14, LOW, TIMEOUT);
  IR_val[15] = pulseIn(IR15, LOW, TIMEOUT);

  for (int i = 0; i < 16; ++i) { //300以下の値はカット
    if (IR_val[i] > 300) {
      IR_val[i] = IR_val[i] - 300;
    }
    else
      IR_val[i] = 0;
  }


  float IRx = 0;
  float IRy = 0;

  if ( IR_val[0] != 0 ) {//0
    IRx += (float)IR_val[0];
  }
  if ( IR_val[1] != 0 ) {//22.5
    IRx += (float)IR_val[1] * 0.92387953251;
    IRy += (float)IR_val[1] * 0.38268343237;
  }
  if ( IR_val[2] != 0 ) {//45
    float work = (float)IR_val[2] * 0.70710678119;
    IRx += work;
    IRy += work;
  }
  if ( IR_val[3] != 0 ) {//67.5
    IRx += (float)IR_val[3] * 0.38268343237;
    IRy += (float)IR_val[3] * 0.92387953251;
  }
  if ( IR_val[4] != 0 ) {//90
    IRy += (float)IR_val[4];
  }
  if ( IR_val[5] != 0 ) {//112.5
    IRx -= (float)IR_val[5] * 0.38268343237;
    IRy += (float)IR_val[5] * 0.92387953251;
  }
  if ( IR_val[6] != 0 ) {//135
    float work = (float)IR_val[6] * 0.70710678118;
    IRx -= work;
    IRy += work;
  }
  if ( IR_val[7] != 0 ) {//157.5
    IRx -= (float)IR_val[7] * 0.92387953251;
    IRy += (float)IR_val[7] * 0.38268343237;
  }
  if ( IR_val[8] != 0 ) {//180
    IRx -= (float)IR_val[8];
  }
  if ( IR_val[9] != 0 ) {//202.5
    IRx -= (float)IR_val[9] * 0.92387953251;
    IRy -= (float)IR_val[9] * 0.38268343237;
  }
  if ( IR_val[10] != 0 ) {//225
    float work = (float)IR_val[10] * -0.70710678118;
    IRx -= work;
    IRy -= work;
  }
  if ( IR_val[11] != 0 ) {//247.5
    IRx -= (float)IR_val[11] * 0.38268343237;
    IRy -= (float)IR_val[11] * 0.92387953251;
  }
  if ( IR_val[12] != 0 ) {//270
    IRy -= (float)IR_val[12] ;
  }
  if ( IR_val[13] != 0 ) {//292.5
    IRx += (float)IR_val[13] * 0.38268343237;
    IRy -= (float)IR_val[13] * 0.92387953251;
  }
  if ( IR_val[14] != 0 ) {//315
    float work = (float)IR_val[14] * 0.70710678118;
    IRx += work;
    IRy -= work;
  }
  if ( IR_val[15] != 0 ) {//337.5
    IRx += (float)IR_val[15] * 0.92387953251;
    IRy -= (float)IR_val[15] * 0.38268343237;
  }

  if ( (IRx != 0) || (IRy != 0) ) {
    float IR_Rad = atan2( IRy, IRx );
    if (IR_Rad < 0)IR_Rad += TWO_PI;
    if (IR_Rad > TWO_PI) IR_Rad -= TWO_PI;
    *d = degrees(IR_Rad);
    *f = sqrt( (IRx * IRx) + (IRy * IRy) );
  }
}

void requestEvent() {

  uint8_t buf[5];

  buf[0] = (IR_Force >> 8) & 0x00ff;
  buf[1] = IR_Force & 0x00ff;
  buf[2] = (IR_Degree >> 8) & 0x00ff;
  buf[3] = IR_Degree & 0x00ff;
  buf[4] = LINE_status;

  Wire.write(buf, 5);
}
