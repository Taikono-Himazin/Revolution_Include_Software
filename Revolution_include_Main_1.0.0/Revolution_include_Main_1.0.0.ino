#include <Wire.h>

//#define

//コンパス、pidは今できそうに無い　知識が足りない

uint8_t LINE_val = 0;
uint16_t IR_Force = 0, IR_Degree = 0;
bool Power_status = true;

void setup() {
  Serial.begin(115200);
  Wire.begin();
}


void loop() {
 /* if (Power_status) {
    LINE_val = LINE_Get();
    LINE_Decision(LINE_val);
    IR_Get();
    Motion_System(IR_Force, IR_Degree);
  } else {
    sleep();
  }
  */
 moter(255,90);
 

}


uint8_t LINE_Get() {
  Wire.requestFrom( 11, 1 );
  uint8_t buf;
  while (Wire.available()) {
    buf = Wire.read();
  }
  //Serial.print(buf);
  //Serial.print(",");
  return buf;
}

void LINE_Decision(uint8_t LINE_s) {
  // LINE_s=0;

  if (bitRead(LINE_s, 4) == 1) {
    if (bitRead(LINE_s, 0) == 1) {//右
      if (bitRead(LINE_s, 1) == 1) {//右かつ後ろ
        moter(255, 135);
      } else if (bitRead(LINE_s, 2) == 1) {//右かつ左
        return;
      } else if (bitRead(LINE_s, 3 ) == 1) { //右かつ前
        moter(255, 215);
      } else {//右のみ
        moter(255, 180);
      }
    } else if (bitRead(LINE_s, 1) == 1) { //後ろ
      if (bitRead(LINE_s, 3 ) == 1) { //後ろかつ前
        return;
      } else if (bitRead(LINE_s, 2 ) == 1) { //後ろかつ左
        moter(255, 45);
      } else {//後ろのみ
        moter(255, 90);
      }
    } else if (bitRead(LINE_s, 2) == 1) {//左
      if (bitRead(LINE_s, 3) == 1) { //左かつ前
        moter(255, 315);
      } else { //左のみ
        moter(255, 0);
      }
    } else if (bitRead(LINE_s, 3) == 1) {//前のみ
      moter(255, 270);
    }
  }
  /*
    Serial.print("LINE");
    Serial.println(LINE_s, BIN);
  */
}

void moter(uint8_t Force, uint16_t Degree) { //一応解読したがいじれるほどはわからん。とりあえず同じ形ならそのままいこう
  int16_t m1_Degree = Degree - 45;
  if (m1_Degree < 0) m1_Degree = m1_Degree + 360;
  else if (m1_Degree > 359) m1_Degree = m1_Degree - 360;

  int16_t m2_Degree = Degree - 315;
  if (m2_Degree < 0) m2_Degree = m2_Degree + 360;
  else if (m2_Degree > 359) m2_Degree = m2_Degree - 360;
  /*
    int m3_Degree = Degree - 225;
    if (m3_Degree < 0) m3_Degree = m3_Degree + 360;
    else if (m3_Degree > 359) m3_Degree = m3_Degree - 360;

    int m4_Degree = Degree - 135;
    if (m4_Degree < 0) m4_Degree = m4_Degree + 360;
    else if (m4_Degree > 359) m4_Degree = m4_Degree - 360;
  */
  int16_t m1 = sin((float)m1_Degree * 0.01745329) * Force; // sin でもcosじゃないと理解不能
  int16_t m2 = sin((float)m2_Degree * 0.01745329) * Force;
  //  int m3 = sin((float)m3_Degree * 0.01745329) * Force;
  //  int m4 = sin((float)m4_Degree * 0.01745329) * Force;

  int16_t Force_max = abs(m1);
  if ( Force_max < abs(m2) ) Force_max = abs(m2);
  //  if ( Force_max < abs(m3) ) Force_max = abs(m3);
  //  if ( Force_max < abs(m4) ) Force_max = abs(m4);

  //  int Force_max = max(  max( abs(m1), abs(m2) ),  max( abs(m3), abs(m4) ) );
  float k = (float)Force / Force_max;//各モーターの比を保ちながら最大値を225に
  m1 = m1 * k;
  m2 = m2 * k;
  //  m3 = m3 * k;
  //  m4 = m4 * k;

  int16_t m3 = -m1;//反対に位置しているから反転
  int16_t m4 = -m2;

  /*
    m1 =   random(255);
    m2 =   random(255);
    int16_t m3 = -m1;
    int16_t m4 = -m2;
  */
  uint8_t buf[5];//送信
  bitSet(buf[4], 4); //モータの電源on
  if ( m1 < 0 ) bitSet(buf[4], 0);
  else bitClear(buf[4], 0);
  buf[0] = abs(m1);
  if ( m2 < 0 ) bitSet(buf[4], 1);
  else bitClear(buf[4], 1);
  buf[1] = abs(m2);
  if ( m3 < 0 ) bitSet(buf[4], 2);
  else bitClear(buf[4], 2);
  buf[2] = abs(m3);
  if ( m4 < 0 ) bitSet(buf[4], 3);
  else bitClear(buf[4], 3);
  buf[3] = abs(m4);

  Wire.beginTransmission(10);
  Wire.write(buf, 5);
  Wire.endTransmission();

  Serial.print(m1);
  Serial.print(",");
  Serial.print(m2);
  Serial.print(",");
  Serial.print(m3);
  Serial.print(",");
  Serial.print(m4);
  Serial.print(",");
  Serial.println(buf[4], BIN);
  delay(1000);
  /*
    data->MotorPWM[0] = m1;
    data->MotorPWM[1] = m2;
    data->MotorPWM[2] = m3;     //dataという型の中にMoterPWM[]という変数があるらしい。よく分からない。ネット検索の際はtypedefで
    data->MotorPWM[3] = m4;
  */
}

void sleep() {
  uint8_t buf[5];//モータに送信
  buf[0] = 0;
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = 0;
  bitClear(buf[4], 4); //モーター電源off

  Wire.beginTransmission(10);
  Wire.write(buf, 5);
  Wire.endTransmission();
}

void moter_Brake(){
  uint8_t buf[5];//モータに送信
  buf[0] = 0;
  buf[1] = 0;
  buf[2] = 0;
  buf[3] = 0;
  bitSet(buf[4], 4); //モーター電源on
  
  Wire.beginTransmission(10);
  Wire.write(buf, 5);
  Wire.endTransmission();
}

void IR_Get() {
  Wire.requestFrom( 9, 4 );
  while (Wire.available()) {
    IR_Force = (Wire.read() << 8) | Wire.read(); // Force Read
    IR_Degree = (Wire.read() << 8) | Wire.read(); // Degree Read
  }
  Serial.print(IR_Force);
Serial.println(IR_Degree);
}

void Motion_System(uint8_t Force, uint16_t Degree) { //挙動制御 Force=IR_Force Degree=IR_Degree
  int16_t Dir = 0;
  if (Force != 0) {  // Ball Found                                           //ここから挙動制御
    if ( (270 <= Degree) && (Degree < 290) ) {                        //a
      Dir = Degree - 55 ;
    } else if ( (290 <= Degree) && (Degree < 340) ) {              //b
      Dir = Degree - 42 ;
    } else if ( (340 <= Degree) && (Degree < 360) ) {               //c
      Dir = Degree - 40 ;
    } else if ( (0 <= Degree) && (Degree < 50) ) {               //d
      Dir = Degree - 45 ;
    } else if ( (50 <= Degree) && (Degree <  60) ) {              //e
      Dir = Degree - 20 ;
    } else if ( (60 <= Degree) && (Degree <  70) ) {              //f
      Dir = Degree - 14 ;
    } else if ( (70 <= Degree) && (Degree <  80) ) {              //g
      Dir = Degree - 8;
    } else if ( (80 <= Degree) && (Degree <  100) ) {               //真ん中
      Dir =  84;
    } else if ( (100 <= Degree) && (Degree <  110) ) {              //g
      Dir = Degree + 8;
    } else if ( (110 <= Degree) && (Degree <  120) ) {              //f
      Dir = Degree + 14 ;
    } else if ( (120 <= Degree) && (Degree <  130) ) {              //e
      Dir = Degree + 20 ;
    } else if ( (130 <= Degree) && (Degree < 180) ) {               //d
      Dir = Degree + 45 ;
    } else if ( (180 <= Degree) && (Degree < 200) ) {               //c
      Dir = Degree + 40 ;
    } else if ( (200 <= Degree) && (Degree < 250) ) {             //b
      Dir = Degree + 42 ;
    } else if ( (250 <= Degree) && (Degree < 270) ) {              //a
      Dir = Degree + 55 ;
    }else{
      moter_Brake();
    }
    if(Dir<0){
      Dir=360+Dir;
    }
    Serial.println(Dir);
    moter(255,Dir);
  }
}


