#include <Wire.h>

///////////////////////////
////Kazuki Ngayama 2016////
///////////////////////////

#define IR0 10
#define IR1 11
#define IR2 12
#define IR3 13
#define IR4 A0
#define IR5 A1
#define IR6 A2
#define IR7 9//赤外線センサ

#define LINE 1//ラインセンサ　この後光センサを組み合わせるがそれはMainで

#define TIMEOUT 10000


void setup() {
//  Serial.begin(115200);
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

}

uint16_t IR_Force = 0, IR_Degree = 0;
uint8_t LINE_status = 0;

void loop() {

  IR_read(&IR_Force, &IR_Degree);
  //  LINE_read();
//    Serial.print("\t");
//    Serial.print(IR_Force);
//    Serial.print("\t");
//    Serial.print(IR_Degree);
//    Serial.println("");
}

void IR_read(uint16_t *f, uint16_t *d) {
  uint16_t IR_val[8];
  /*
    IR_val[0] = 400;
    IR_val[1] = 400;
    IR_val[2] = 0;
    IR_val[3] = 0;
    IR_val[4] = 0;
    IR_val[5] = 0;
    IR_val[6] = 0;
    IR_val[7] = 0;
  */

  IR_val[0] = pulseIn(IR0, LOW, TIMEOUT);
  IR_val[1] = pulseIn(IR1, LOW, TIMEOUT);
  IR_val[2] = pulseIn(IR2, LOW, TIMEOUT);
  IR_val[3] = pulseIn(IR3, LOW, TIMEOUT);
  IR_val[4] = pulseIn(IR4, LOW, TIMEOUT);
  IR_val[5] = pulseIn(IR5, LOW, TIMEOUT);
  IR_val[6] = pulseIn(IR6, LOW, TIMEOUT);
  IR_val[7] = pulseIn(IR7, LOW, TIMEOUT);




  for (int i = 0; i < 8; ++i) { //300以下の値はカット
    if (IR_val[i] > 300) {
      IR_val[i] = IR_val[i] - 300;
    } else {
      IR_val[i] = 0;
    }

  }
  for (int j = 0; j < 8; j++) {
//    Serial.print(IR_val[j]);
//    Serial.print("\t");
  }


  float IRx = 0;
  float IRy = 0;

  if ( IR_val[0] != 0 ) {//0
    IRx += (float)IR_val[0];
  }
  if ( IR_val[1] != 0 ) {//45
    float work = (float)IR_val[1] * 0.70710678118;
    IRx += work;
    IRy += work;
  }
  if ( IR_val[2] != 0 ) {//90
    IRy += (float)IR_val[2];
  }
  if ( IR_val[3] != 0 ) {//135
    float work = (float)IR_val[3] * 0.70710678118;
    IRx -= work;
    IRy += work;
  }
  if ( IR_val[4] != 0 ) {//180
    IRx -= (float)IR_val[4];
  }
  if ( IR_val[5] != 0 ) {//225
    float work = (float)IR_val[5] * 0.70710678118;
    IRx -= work;
    IRy -= work;
  }
  if ( IR_val[6] != 0 ) {//270
    IRy -= (float)IR_val[6] ;
  }
  if ( IR_val[7] != 0 ) {//315
    float work = (float)IR_val[7] * 0.70710678118;
    IRx += work;
    IRy -= work;
  }

  if ( (IRx != 0) || (IRy != 0) ) {
    float IR_Rad = atan2( IRy, IRx );
    if (IR_Rad < 0)IR_Rad += TWO_PI;
    if (IR_Rad > TWO_PI) IR_Rad -= TWO_PI;
    *d = degrees(IR_Rad);
    *f = sqrt( (IRx * IRx) + (IRy * IRy) );
  } else {
    *d = 0;
    *f = 0;
  }

}


void requestEvent() {

  uint8_t buf[4];
  buf[0] = (IR_Force >> 8) & 0x00ff;
  buf[1] = IR_Force & 0x00ff;
  buf[2] = (IR_Degree >> 8) & 0x00ff;
  buf[3] = IR_Degree & 0x00ff;
  Wire.write(buf, 4);
}
