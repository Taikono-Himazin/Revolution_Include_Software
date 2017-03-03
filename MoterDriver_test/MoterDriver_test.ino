#include <Wire.h>
#include <Utility.h>


///////////////////////////////////
//delay関数がおかしくなっています//
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
  // put your setup code here, to run once:
  Serial.begin(9600);
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

  setPWMFrequency(M1P,PWM_DIV1);//PWM高速化（分周比を1に）
  setPWMFrequency(M2P, PWM_DIV1);
  setPWMFrequency(M3P, PWM_DIV1);
  setPWMFrequency(M4P, PWM_DIV1);
}

boolean Power_Status;
boolean update_Flag;
boolean Moter_Brake[4];
int16_t Moter_Force[4];
void loop() {
	moter_test(M1A,M1B,M1P);
	moter_test(M2A, M2B, M2P);
	moter_test(M3A, M3B, M3P);
	moter_test(M4A, M4B, M4P);
}

void moter_test(uint8_t A,uint8_t B,uint8_t PWM) {
	digitalWrite(A, HIGH);
	digitalWrite(B, LOW);
	for (int i=255; i >0; i--) {
		analogWrite(PWM, i);
		Serial.println(i);
	}

	delay(1000);

	digitalWrite(B, HIGH);
	digitalWrite(A, LOW);
	for (int i = 255; i > 0; i--) {
		analogWrite(PWM, i);
		Serial.println(i);
	}

	delay(1000);
	digitalWrite(A, HIGH);
	digitalWrite(B, HIGH);
	for (int i = 255; i > 0; i--) {
		analogWrite(PWM, i);
		Serial.println(i);
	}
	delay(1000);

	digitalWrite(A, LOW);
	digitalWrite(B, LOW);
	analogWrite(PWM, 0);
}