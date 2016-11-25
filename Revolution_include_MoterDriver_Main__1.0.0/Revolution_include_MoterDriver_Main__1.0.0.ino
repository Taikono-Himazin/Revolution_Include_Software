
#include <Wire.h>

///////////////////////////////////
//delay関数がおかしくなっています//
///////////////////////////////////

#define M1A 4          // M1はデジタル4番ピン
#define M1B 2          // M1はデジタル2番ピン
#define M1P 3           // M1はデジタル3番ピン(PWM)

#define M2A 7           // M2はデジタル6番ピン
#define M2B 5           // M2はデジタル5番ピン
#define M2P 6           // M2はデジタル9番ピン(PWM)

#define M3A 10          // M3はデジタルA2番ピン
#define M3B 8          // M3はデジタルA3番ピン
#define M3P 9          // M3はデジタル10番ピン(PWM)

#define M4A 13          // M4はデジタルA0番ピン
#define M4B 12          // M4はデジタルA1番ピン
#define M4P 11          // M4はデジタル11番ピン(PWM)

int16_t Moter_Force[4], IR_Force, IR_Degree;
bool Power_Status = true, Moter_Brake[4], update_Flag = true;


void setup() {
  //  Serial.begin(9600);
  Wire.begin();

  pinMode(M1A, OUTPUT);   // 出力に設定
  pinMode(M1B, OUTPUT);   // 出力に設定
  pinMode(M1P, OUTPUT);   // 出力に設定

  pinMode(M2A, OUTPUT);   // 出力に設定
  pinMode(M2B, OUTPUT);   // 出力に設定
  pinMode(M2P, OUTPUT);   // 出力に設定

  pinMode(M3A, OUTPUT);   // 出力に設定
  pinMode(M3B, OUTPUT);   // 出力に設定
  pinMode(M3P, OUTPUT);   // 出力に設定

  pinMode(M4A, OUTPUT);   // 出力に設定
  pinMode(M4B, OUTPUT);   // 出力に設定
  pinMode(M4P, OUTPUT);   // 出力に設定
}

void loop() {
  moterMain();
  moter(IR_Force, IR_Degree);
  moterDrive();
}

void moterDrive() {

  if (Moter_Brake[0]) {                         //モーター1
    digitalWrite(M1A, HIGH);   // ブレーキ
    digitalWrite(M1B, HIGH);
    analogWrite(M1P, 255);
  }  else if (Moter_Force[0] > 0) {
    digitalWrite(M1A, LOW);   // M1 前進
    digitalWrite(M1B, HIGH);
    analogWrite(M1P, Moter_Force[0]);
  } else {
    digitalWrite(M1A, HIGH);  // M1 後進
    digitalWrite(M1B, LOW);
    analogWrite(M1P, -Moter_Force[0]);
  }

  if (Moter_Brake[1]) {                       //モータ一2
    digitalWrite(M2A, HIGH);   // ブレーキ
    digitalWrite(M2B, HIGH);
    analogWrite(M2P, 255);
  }  else if (Moter_Force[1] > 0) {
    digitalWrite(M2A, LOW);   // M2 前進
    digitalWrite(M2B, HIGH);
    analogWrite(M2P, Moter_Force[1]);
  } else {
    digitalWrite(M2A, HIGH);  // M2 後進
    digitalWrite(M2B, LOW);
    analogWrite(M2P, -Moter_Force[1]);
  }

  if (Moter_Brake[2]) {                       //モータ一3
    digitalWrite(M3A, HIGH);  // ブレーキ
    digitalWrite(M3B, HIGH);
    analogWrite(M3P, 255/2);
  } else if (Moter_Force[2] > 0) {
    digitalWrite(M3A, LOW);   // M3 前進
    digitalWrite(M3B, HIGH);
    analogWrite(M3P, Moter_Force[2]/2);
  } else {
    digitalWrite(M3A, HIGH);  // M3 後進
    digitalWrite(M3B, LOW);
    analogWrite(M3P, -Moter_Force[2]/2);
  }

  if (Moter_Brake[3]) {                        //モータ一4
    digitalWrite(M4A, HIGH);  // ブレーキ
    digitalWrite(M4B, HIGH);
    analogWrite(M4P, 255);
  }  else if (Moter_Force[3] > 0) {
    digitalWrite(M4A, LOW);   // M4 前進
    digitalWrite(M4B, HIGH);
    analogWrite(M4P, Moter_Force[3]);
  } else {
    digitalWrite(M4A, HIGH);  // M4 後進
    digitalWrite(M4B, LOW);
    analogWrite(M4P, -Moter_Force[3]);
  }
  /*
    Serial.print(Moter_Force[0]);
    Serial.print(",");
    Serial.print(Moter_Force[1]);
    Serial.print(",");
    Serial.print(Moter_Force[2]);
    Serial.print(",");
    Serial.println(Moter_Force[3]);
  */
}

void moterFree() {
  digitalWrite(M1A, LOW);   // フリー
  digitalWrite(M1B, LOW);
  analogWrite(M1P, 0);

  digitalWrite(M2A, LOW);   // フリー
  digitalWrite(M2B, LOW);
  analogWrite(M2P, 0);

  digitalWrite(M3A, LOW);   // フリー
  digitalWrite(M3B, LOW);
  analogWrite(M3P, 0);

  digitalWrite(M4A, LOW);   // フリー
  digitalWrite(M4B, LOW);
  analogWrite(M4P, 0);

}

void moterMain() {

  Wire.requestFrom( 9, 4 );
  while (Wire.available()) {
    IR_Force = (Wire.read() << 8) | Wire.read(); // Force Read
    IR_Degree = (Wire.read() << 8) | Wire.read(); // Degree Read
  }
}


void moter(int16_t Force, int16_t Degree) { //一応解読したがいじれるほどはわからん。とりあえず同じ形ならそのままいこう
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

  Moter_Force[0] = m1;
  Moter_Force[1] = m2;
  Moter_Force[2] = m3;
  Moter_Force[3] = m4;

  /*
    m1 =   random(255);
    m2 =   random(255);
    int16_t m3 = -m1;
    int16_t m4 = -m2;
  */
  /* uint8_t buf[5];//送信
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
  */
  /*
    Serial.print(m1);
    Serial.print(",");
    Serial.print(m2);
    Serial.print(",");
    Serial.print(m3);
    Serial.print(",");
    Serial.print(m4);
    Serial.print(",");
    //    Serial.println(buf[4], BIN);
    delay(1000);
    /*
      data->MotorPWM[0] = m1;
      data->MotorPWM[1] = m2;
      data->MotorPWM[2] = m3;     //dataという型の中にMoterPWM[]という変数があるらしい。よく分からない。ネット検索の際はtypedefで
      data->MotorPWM[3] = m4;
  */
}
