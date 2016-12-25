#include <Utility.h>
#include <Wire.h>


///////////////////////////////////
//delay関数がおかしくなっています//
//adjustDelay();を使ってください //
///////////////////////////////////


#define M1A 4          // M1はデジタル4番ピン
#define M1B 2          // M1はデジタル2番ピン
#define M1P 3           // M1はデジタル3番ピン(PWM)

#define M2A 7           // M2はデジタル6番ピン
#define M2B 5           // M2はデジタル5番ピン
#define M2P 6           // M2はデジタル9番ピン(PWM)

#define M3A 10          // M3はデジタルw2番ピン
#define M3B 8          // M3はデジタルA3番ピン
#define M3P 9          // M3はデジタル10番ピン(PWM)

#define M4A 13          // M4はデジタルA0番ピン
#define M4B 12          // M4はデジタルA1番ピン
#define M4P 11          // M4はデジタル11番ピン(PWM)

void setup() {
  //Serial.begin(9600);
  //Serial.print("Moter_Driver");
  Wire.begin(10);
  i2c_faster();
 Wire.onReceive(readEvent);

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
  
  setPWMFrequency(M1P, PWM_DIV1);//PWM高速化（分周比を1に）
  setPWMFrequency(M2P, PWM_DIV1);
  setPWMFrequency(M3P, PWM_DIV1);
  setPWMFrequency(M4P, PWM_DIV1);
  
  }

boolean Power_Status;
boolean update_Flag;
boolean Moter_Brake[4];
int16_t Moter_Force[4];


void loop() {

  if (Power_Status) {
    if (update_Flag) {
      moterDrive();
      update_Flag = false;
    }
  } else {
    moterFree();
  }
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
    analogWrite(M3P, 255);
  } else if (Moter_Force[2] > 0) {
    digitalWrite(M3A, LOW);   // M3 前進
    digitalWrite(M3B, HIGH);
    analogWrite(M3P, Moter_Force[2]);
  } else {
    digitalWrite(M3A, HIGH);  // M3 後進
    digitalWrite(M3B, LOW);
    analogWrite(M3P, -Moter_Force[2]);
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

void readEvent(int x) {

  uint8_t buf[5];

  while ( Wire.available() ) {
    buf[0] = Wire.read();//モータ1の強さ
    buf[1] = Wire.read();//モータ2の強さ
    buf[2] = Wire.read();//モータ3の強さ
    buf[3] = Wire.read();//モータ4の強さ
    buf[4] = Wire.read();//ビット0:モータ1の符号　ビット1:モータ2の符号　ビット2:モータ3の符号　ビット3:モータ4の符号　ビット4:電源ステータス　ビット5～8:無意味
  }
  if (bitRead(buf[4], 4) == 1) {

    for (uint8_t i = 0; i < 4; i++) {

      if (buf[i] == 0) {//pwmが0の時はブレーキ
        Moter_Brake[i] = true;
        Moter_Force[i] = 0;
      } else if (bitRead(buf[4], i) == 1) {
        Moter_Brake[i] = false;
        Moter_Force[i] = buf[i] * (-1);
      } else {
        Moter_Brake[i] = false;
        Moter_Force[i] = buf[i];
      }
    }

    Power_Status = true;
  } else {
    Power_Status = false;
  }
  update_Flag = true;
}