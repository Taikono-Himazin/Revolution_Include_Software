//////////////////////////////////
///RCJ  Revolution_Include 2017///
///      Main　永山和樹        ///
//////////////////////////////////


/*Include*/
#include <Servo.h>
#include <Utility.h>
#include <Wire.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <LiquidCrystal_I2C.h>
/*ここまで*/

/*define*/
#define M_sw 0
#define SPEAKER 13
#define BEEP 100
#define Old_Persent 0  //MOterの過去の値の割合
#define L_sw 4 
#define D_sw 7
#define R_sw 10
#define LED_L A3
#define LED_M A2
#define LED_R A1
#define Check_Delay 100
#define UI_Delay 200
#define IR_offset 5
#define  C_Reset 30
#define  C_Reset2 10

#define LED(a) digitalWrite(a, HIGH)
#define LEDoff(a) digitalWrite(a, LOW)
#define Servo_idel Dri1_Power=85;Dri2_Power=Dri1_Power
#define Servo1_Dri Dri1_Power=180;Dri2_Power=85
#define Servo2_Dri Dri2_Power=180;Dri1_Power=85
/*ここまで*/

/*川野さんからコピペ関数宣言*/
static double Setpoint, Input, Output;
static const double Kp = 2, Ki = 0, Kd = 0.01;
MPU6050 mpu;
Servo myServo1;
Servo myServo2;
static uint8_t mpuIntStatus;
static bool dmpReady = false;  // set true if DMP init was successful
static uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
/*ここまで*/

/*リキッドクリスタル関数宣言*/
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address													
/*ここまで*/

/*変数宣言*/
int16_t  Gyro_Now = 0, Gyro = 0, Gyro_Offset = 0, old_Moter_D = 0,LINE_count = 0;
uint8_t LINE_Status = 0, UI_status = 0;
uint16_t IR_F = 0, IR_D = 0, old_Moter_F = 0, LINE_NOW;

boolean change1 = true, change2 = false;
/*ここまで*/

//プロトタイプ宣言(不必要のため途中からコメントアウト　エラー起こしたらしてみて
/*void moter(uint8_t Force, uint16_t Degree);
void sleep();
void IR_Get();
void GyroGet();
void Motion_System(uint8_t Force, uint16_t Degree);
void Melody(uint8_t mode);
void LINE_Get();
void lcd_Start(char* ver);
void UI();
void LED_Check();

/*ここまで*/

/*--プログラム--*/
void setup() {
	//Serial.begin(115200);
	Wire.begin();
	i2c_faster();

	PID_Start();

	lcd_Start("2.1.5 LINE");//lcd初期化関数

	Gryo_Start();

	Servo_Start();

	LINE_Set(350);

	while (digitalRead(M_sw) == LOW) {
		Melody(1);
		delay(1000);
		lcd.setCursor(0, 1);
		lcd.print("Please turn off");
	}

	LED_Check();

	lcd.setCursor(0, 1);
	lcd.print("               ");
	lcd.setCursor(0, 1);
	lcd.print("ALL OK");

	UI_status = 9;
	Melody(0);
}

void loop() {
	if (digitalRead(M_sw) == LOW) {
		if (change1) {
			lcd.noBacklight();
			change1 = false;
			change2 = true;
			LEDoff(LED_R);
			LEDoff(LED_M);
			LEDoff(LED_L);
		}
		LINE_Get();
		if (LINE_count <= 0) {
			IR_Get();
			Motion_System(IR_F, IR_D);
	}
		else {
			LINE_count--;
		}
	}
	else {
		if (change2) {
			lcd.backlight();
			change1 = true;
			change2 = false;
			myServo1.write(0);
			myServo2.write(0);
		}
		sleep();
		UI();
	}
}

/*--自作関数--*/
void moter(uint8_t Force, int16_t Degree) { //一応解読したがいじれるほどはわからん。とりあえず同じ形ならそのままいこう

	int16_t m1, m2;
	old_Moter_D = old_Moter_D*Old_Persent + Degree*(1 - Old_Persent);
	old_Moter_F = old_Moter_F*Old_Persent + Force*(1 - Old_Persent);

	int16_t m1_D = old_Moter_D - 45;
	if (m1_D < 0) m1_D = m1_D + 360;
	else if (m1_D > 359) m1_D = m1_D - 360;

	m1 = sin((float)m1_D * 0.01745329) * old_Moter_F; // sin でもcosじゃないと理解不能

	int16_t m2_D = old_Moter_D - 315;
	if (m2_D < 0) m2_D = m2_D + 360;
	else if (m2_D > 359) m2_D = m2_D - 360;

	m2 = sin((float)m2_D * 0.01745329) * old_Moter_F;

	int16_t F_max = abs(m1);
	if (F_max < abs(m2)) F_max = abs(m2);

	float k = (float)old_Moter_F / F_max;//各モーターの比を保ちながら最大値を225に
	m1 = m1 * k;
	m2 = m2 * k;

	int16_t m3 = -m1;//反対に位置しているから反転
	int16_t m4 = -m2;

	GyroGet();//ジャイロのデータを取得
	Input = Gyro; //ジャイロのデータをInput関数に突っ込む
	myPID.Compute(); //pid計算

	m1 = m1 - Output;//pidのデータをモーターに突っ込む
	m2 = m2 - Output;
	m3 = m3 - Output;
	m4 = m4 - Output;
	m1 = constrain(m1, -255, 255);
	m2 = constrain(m2, -255, 255);
	m3 = constrain(m3, -255, 255);
	m4 = constrain(m4, -255, 255);

	/*lcd.print(m1);
	lcd.print(",");
	lcd.print(m2);
	lcd.setCursor(0, 1);
	lcd.print(m3);
	lcd.print(",");
	lcd.print(m4);
	lcd.print(",");*/

	uint8_t buf[5];//送信
	bitSet(buf[4], 4); //モータの電源on
	if (m1 < 0) bitSet(buf[4], 0);
	else bitClear(buf[4], 0);
	buf[0] = abs(m1);
	if (m2 < 0) bitSet(buf[4], 1);//モーターの配線を間違えた
	else bitClear(buf[4], 1);
	buf[1] = abs(m2);
	if (m3 < 0) bitSet(buf[4], 2);
	else bitClear(buf[4], 2);
	buf[2] = abs(m3);
	if (m4 < 0) bitSet(buf[4], 3);
	else bitClear(buf[4], 3);
	buf[3] = abs(m4);

	Wire.beginTransmission(10);
	Wire.write(buf, 5);
	Wire.endTransmission();
}

void GyroGet() {
	static uint16_t fifoCount;
	static uint8_t fifoBuffer[64]; // FIFO storage buffer
								   // orientation/motion vars
	static Quaternion q;           // [w, x, y, z]         quaternion container
	static VectorFloat gravity;    // [x, y, z]            gravity vector
	static float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	mpuIntStatus = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		mpu.resetFIFO();
	}
	else if (mpuIntStatus & 0x02) {
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		Gyro_Now = degrees(ypr[0]) + 180;
		Gyro = Gyro_Now + Gyro_Offset;
		if (Gyro < 0) Gyro += 360;
		if (Gyro > 359) Gyro -= 360;
	}
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

void IR_Get() {
	Wire.requestFrom(9, 4);
	while (Wire.available()) {
		IR_F = (Wire.read() << 8) | Wire.read(); // Force Read
		IR_D = (Wire.read() << 8) | Wire.read(); // Degree Read
	}
	IR_D = IR_D + IR_offset;
	if (IR_D < 0) {
		IR_D = IR_D + 360;
	}
	else if (IR_D > 360) {
		IR_D = IR_D - 360;
	}
	/*
	 Serial.print(IR_F);
	 Serial.println(IR_D);*/
}

void Motion_System(uint8_t Force, int16_t Degree) { //挙動制御 Force=IR_F Degree=IR_D
	int16_t M_Degree = 0, Dri1_Power, Dri2_Power;
	uint8_t	M_Force = 255;
	static uint8_t count = C_Reset, count2 = C_Reset2;
	bool Ball1 = analogRead(A6) > 950;
	bool Ball2 = analogRead(A7) > 950;
	/*Servo1_Dri=前 Servo2_Dri=後ろ*/
	if (Force != 0) {  // Ball Found                                           //ここから挙動制御
		if ((270 <= Degree) && (Degree < 280)) {//a
			M_Degree = 270;
			M_Force = 100;
			Servo2_Dri;
		}
		else if ((280 <= Degree) && (Degree < 290)) {//b
			M_Degree = Degree + 5;
			M_Force = 150;
			Servo_idel;
		}
		else if ((290 <= Degree) && (Degree < 300)) {//c
			M_Degree = Degree + 10;
			M_Force = 150;
			Servo_idel;
		}
		else if ((300 <= Degree) && (Degree < 310)) {//d
			M_Degree = Degree + 15;
			M_Force = 150;
			Servo_idel;
		}
		else if ((310 <= Degree) && (Degree < 320)) {//e
			M_Degree = Degree + 15;
			M_Force = 150;
			Servo_idel;
		}
		else if ((320 <= Degree) && (Degree < 330)) {//f
			M_Degree = Degree - 60;
			Servo_idel;
		}
		else if ((330 <= Degree) && (Degree < 340)) {//g
			M_Degree = Degree - 50;
			Servo_idel;
		}
		else if ((340 <= Degree) && (Degree < 350)) {//h
			M_Degree = Degree - 50;
			Servo_idel;
		}
		else if ((350 <= Degree) && (Degree < 360)) {//i
			M_Degree = Degree - 40;
			Servo_idel;
		}
		else if ((0 <= Degree) && (Degree < 10)) {//j
			M_Degree = Degree - 40;
			Servo_idel;
		}
		else if ((10 <= Degree) && (Degree < 20)) {//k
			M_Degree = Degree - 30;
			Servo_idel;
		}
		else if ((20 <= Degree) && (Degree < 30)) {//l
			M_Degree = Degree - 30;
			Servo_idel;
		}
		else if ((30 <= Degree) && (Degree < 40)) {//m
			M_Degree = Degree - 30;
			Servo_idel;
		}
		else if ((40 <= Degree) && (Degree < 50)) {//n
			M_Degree = Degree - 20;
			Servo_idel;
		}
		else if ((50 <= Degree) && (Degree < 60)) {//o
			M_Degree = Degree - 20;
			Servo_idel;
		}
		else if ((60 <= Degree) && (Degree < 70)) {//p
			M_Degree = Degree - 20;
			Servo_idel;
		}
		else if ((70 <= Degree) && (Degree < 80)) {//q
			M_Degree = Degree - 10;
			Servo_idel;
		}
		else if ((80 <= Degree) && (Degree < 85)) {//r
			M_Degree = Degree - 10;
			Servo1_Dri;
		}
		else if ((85 <= Degree) && (Degree < 95)) {//真ん中
			M_Degree = 90;
			Servo1_Dri;
		}
		else if ((95 <= Degree) && (Degree < 100)) {//r
			M_Degree = Degree + 10;
			Servo1_Dri;
		}
		else if ((100 <= Degree) && (Degree < 110)) {//q
			M_Degree = Degree + 10;
			Servo_idel;
		}
		else if ((110 <= Degree) && (Degree < 120)) {//p
			M_Degree = Degree + 20;
			Servo_idel;
		}
		else if ((120 <= Degree) && (Degree < 130)) {//o
			M_Degree = Degree + 20;
			Servo_idel;
		}
		else if ((130 <= Degree) && (Degree < 140)) {//n
			M_Degree = Degree + 20;
			Servo_idel;
		}
		else if ((140 <= Degree) && (Degree < 150)) {//m
			M_Degree = Degree + 30;
			Servo_idel;
		}
		else if ((150 <= Degree) && (Degree < 160)) {//l
			M_Degree = Degree + 30;
			Servo_idel;
		}
		else if ((160 <= Degree) && (Degree < 170)) {//k
			M_Degree = Degree + 30;
			Servo_idel;
		}
		else if ((170 <= Degree) && (Degree < 180)) {//j
			M_Degree = Degree + 40;
			Servo_idel;
		}
		else if ((180 <= Degree) && (Degree < 190)) {//i
			M_Degree = Degree + 40;
			Servo_idel;
		}
		else if ((190 <= Degree) && (Degree < 200)) {//h
			M_Degree = Degree + 50;
			Servo_idel;
		}
		else if ((200 <= Degree) && (Degree < 210)) {//g
			M_Degree = Degree + 50;
			Servo_idel;
		}
		else if ((210 <= Degree) && (Degree < 220)) {//f
			M_Degree = Degree + 60;
			Servo_idel;
		}
		else if ((220 <= Degree) && (Degree < 230)) {//e
			M_Degree = Degree - 15;
			M_Force = 150;
			Servo_idel;
		}
		else if ((230 <= Degree) && (Degree < 240)) {//d
			M_Degree = Degree - 15;
			M_Force = 150;
			Servo_idel;
		}
		else if ((240 <= Degree) && (Degree < 250)) {//c
			M_Degree = Degree - 10;
			M_Force = 150;
			Servo_idel;
		}
		else if ((250 <= Degree) && (Degree < 265)) {//b
			M_Degree = Degree - 5;
			M_Force = 150;
			Servo2_Dri;
		}
		else if ((265 <= Degree) && (Degree < 270)) {//a
			M_Degree = 270;
			M_Force = 100;
			Servo2_Dri;
		}
		if (M_Degree < 0) {
			M_Degree = 360 + M_Degree;
		}

		if (Ball1) {
			count--;
			count2 = C_Reset2;
		}
		else {
			count2--;
			if (count2 == 0) {
				count = C_Reset;
				count2 = C_Reset2;
			}
		}
	}
	else {
		M_Degree = 90;
		M_Force = 0;
		Dri1_Power = 0;
		Dri2_Power = 0;
	}


	if (count <= 0) {
		Spin();
		count = C_Reset;
		count2 = C_Reset2;
	}
	else {
		moter(M_Force, M_Degree);
		myServo1.write(Dri1_Power);
		myServo2.write(Dri2_Power);
	}

}

void Spin() {
	uint8_t m1, m2, m3, m4;
	m1 = 100;
	m2 = m1;
	m3 = m2;
	m4 = m3;

	uint8_t buf[5];//送信
	bitSet(buf[4], 4); //モータの電源on
	if (m1 < 0) bitSet(buf[4], 0);
	else bitClear(buf[4], 0);
	buf[0] = abs(m1);
	if (m2 < 0) bitSet(buf[4], 1);//モーターの配線を間違えた
	else bitClear(buf[4], 1);
	buf[1] = abs(m2);
	if (m3 < 0) bitSet(buf[4], 2);
	else bitClear(buf[4], 2);
	buf[2] = abs(m3);
	if (m4 < 0) bitSet(buf[4], 3);
	else bitClear(buf[4], 3);
	buf[3] = abs(m4);

	Wire.beginTransmission(10);
	Wire.write(buf, 5);
	Wire.endTransmission();

	delay(200);

	m1 = 255;
	m2 = m1;
	m3 = m2;
	m4 = m3;

	bitSet(buf[4], 4); //モータの電源on
	if (m1 < 0) bitSet(buf[4], 0);
	else bitClear(buf[4], 0);
	buf[0] = abs(m1);
	if (m2 < 0) bitSet(buf[4], 1);//モーターの配線を間違えた
	else bitClear(buf[4], 1);
	buf[1] = abs(m2);
	if (m3 < 0) bitSet(buf[4], 2);
	else bitClear(buf[4], 2);
	buf[2] = abs(m3);
	if (m4 < 0) bitSet(buf[4], 3);
	else bitClear(buf[4], 3);
	buf[3] = abs(m4);

	Wire.beginTransmission(10);
	Wire.write(buf, 5);
	Wire.endTransmission();
	delay(250);
}

void Melody(uint8_t mode) {
	if (mode == 0) {   // StartMelody
		tone(SPEAKER, 1319, BEEP);
		delay(BEEP);
		noTone(SPEAKER);
		delay(50);
		tone(SPEAKER, 1397, BEEP);
		delay(BEEP);
		noTone(SPEAKER);
		tone(SPEAKER, 2093, BEEP);
		delay(BEEP);
		noTone(SPEAKER);
		tone(SPEAKER, 2093, BEEP);
		delay(BEEP);
		noTone(SPEAKER);
	}
	else if (mode == 1) {//errerMelody
		tone(SPEAKER, 262, BEEP);
		delay(BEEP);
		tone(SPEAKER, 262, BEEP);
		delay(BEEP);
		noTone(SPEAKER);
	}
}

void LINE_Get() {
	Wire.requestFrom(11, 1);
	uint8_t buf;
	while (Wire.available()) {
		buf = Wire.read();
	}

	LINE_Status = buf;
	//	Serial.println(LINE_Status,BIN);

	if (bitRead(LINE_Status, 4) == 1) {
		LINE_count = 100;
		digitalWrite(LED_L, HIGH);
		if (bitRead(LINE_Status, 0) == 1) {//右
			if (bitRead(LINE_Status, 1) == 1) {//右かつ後ろ
				moter(255, 135);
			}
			else if (bitRead(LINE_Status, 2) == 1) {//右かつ左
				return;
			}
			else if (bitRead(LINE_Status, 3) == 1) { //右かつ前
				moter(255, 215);
			}
			else {//右のみ
				moter(255, 180);
			}
		}
		else if (bitRead(LINE_Status, 1) == 1) { //後ろ
			if (bitRead(LINE_Status, 3) == 1) { //後ろかつ前
				return;
			}
			else if (bitRead(LINE_Status, 2) == 1) { //後ろかつ左
				moter(255, 45);
			}
			else {//後ろのみ
				moter(255, 90);
			}
		}
		else if (bitRead(LINE_Status, 2) == 1) {//左
			if (bitRead(LINE_Status, 3) == 1) { //左かつ前
				moter(255, 315);
			}
			else { //左のみ
				moter(255, 0);
			}
		}
		else if (bitRead(LINE_Status, 3) == 1) {//前のみ
			moter(255, 270);
		}
	}
	else {
		digitalWrite(LED_L, LOW);
	}
	/*
	Serial.print("LINE");
	Serial.println(LINE_Status, BIN);
	*/
}

void UI() {
	bool L = digitalRead(L_sw) == HIGH;
	bool D = digitalRead(D_sw) == HIGH;
	bool R = digitalRead(R_sw) == HIGH;
	switch (UI_status)
	{
	case 0:
		lcd.home();
		lcd.print("Revolution    ");
		lcd.setCursor(0, 1);
		lcd.print("         Include");
		if (L || D || R) {
			UI_status = 10;
			lcd.clear();
			delay(UI_Delay);
		}
		break;


		/*
		case 1:
		lcd.home();
		lcd.print("Main Menu");
		lcd.setCursor(0, 1);
		lcd.print("L:exit D: R:set");
		if (R) {
			UI_status = 8;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (L) {
			UI_status = 0;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
		*/
	case 2:
		lcd.home();
		lcd.print("Dribler test");
		lcd.setCursor(0, 1);
		lcd.print("L:1 D:2 R:next");
		if (L) {
			UI_status = 3;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			UI_status = 4;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 5;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 3:
		lcd.home();
		lcd.print("Dribler 1 test");
		lcd.setCursor(0, 1);
		lcd.print("L: D:OP R:exit");
		if (D) {
			LED(LED_R);
			myServo1.write(180);

		}
		else {
			LEDoff(LED_R);
			myServo1.write(0);
		}
		if (R) {
			myServo1.write(0);
			UI_status = 2;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 4:
		lcd.home();
		lcd.print("Dribler 2 test");
		lcd.setCursor(0, 1);
		lcd.print("L: D:OP R:exit");
		if (D) {
			LED(LED_R);
			myServo2.write(180);
		}
		else {
			LEDoff(LED_R);
			myServo2.write(0);
		}
		if (R) {
			myServo2.write(0);
			UI_status = 2;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 5:
		lcd.home();
		lcd.print("LED test");
		lcd.setCursor(0, 1);
		lcd.print("L: D:Yes R:next");
		if (D) {
			LED_Check();
		}
		else if (R) {
			UI_status = 6;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 6:
		lcd.home();
		lcd.print("Gyro&IR Monitor");
		lcd.setCursor(0, 1);
		lcd.print("L: D:Yes R:next");
		if (D) {
			UI_status = 7;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 8;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 7:
		GyroGet();
		IR_Get();
		lcd.clear();
		lcd.home();
		lcd.print("IR   ");
		lcd.print(IR_D);
		lcd.setCursor(0, 1);
		lcd.print("Gyro ");
		lcd.print(Gyro);
		if (L || D || R) {
			UI_status = 6;
			lcd.clear();
			delay(UI_Delay);
		}
		delay(10);
		break;
	case 8:
		lcd.home();
		lcd.print("Gyro Set");
		lcd.setCursor(0, 1);
		lcd.print("L: D:Yes R:next");
		if (D) {
			UI_status = 9;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 10;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 9:
		lcd.home();
		lcd.print("Gyro            ");
		GyroGet();
		lcd.setCursor(6, 0);
		lcd.print(Gyro_Now);
		lcd.setCursor(0, 1);
		lcd.print("L:Re D:set R:exit");
		delay(100);
		if (D) {
			Gyro_Offset = 180 - Gyro_Now;
			lcd.setCursor(0, 1);
			lcd.print("                ");
			lcd.setCursor(0, 1);
			lcd.print("Set completed!");
			delay(1000);
			UI_status = 0;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (L) {
			Gryo_Start();
		}
		else if (R) {
			UI_status = 8;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 10:
		lcd.home();
		lcd.print("LINE_Val:");
		lcd.setCursor(6, 0);
		lcd.print(LINE_NOW);
		lcd.setCursor(0, 1);
		lcd.print("L:up D:down R:next");
		if (L) {
			LINE_Set(LINE_NOW + 10);
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 2;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			LINE_Set(LINE_NOW - 10);
			delay(UI_Delay);
		}
		break;
	default:
		lcd.clear();
		lcd.print("ERRER AUTO REPAIR");
		delay(1000);
		UI_status = 0;
		break;
	}

}

/*初期化関数*/
void lcd_Start(char* ver) {

	lcd.begin(16, 2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
	lcd.backlight(); // finish with backlight on  
	lcd.home();                   // go home
	lcd.print("Hello Revolution");
	delay(1000);
	lcd.setCursor(0, 1);        // go to the next line
	lcd.print("Main ");
	lcd.print(ver);
	delay(1000);
	lcd.clear();
	delay(500);
	lcd.print("Checking");
	for (int i = 0; i < 4; i++) {
		delay(100);
		lcd.print(".");
	}
}

void LED_Check() {
	LED(LED_L);
	delay(Check_Delay);
	LED(LED_M);
	delay(Check_Delay);
	LED(LED_R);
	delay(Check_Delay);
	LEDoff(LED_L);
	delay(Check_Delay);
	LEDoff(LED_M);
	delay(Check_Delay);
	LEDoff(LED_R);
}

void Servo_Start() {
	myServo1.attach(6, 1, 2);
	myServo1.write(0);
	myServo2.attach(3, 1, 2);
	myServo2.write(0);
}

void Gryo_Start() {
	mpu.initialize();
	if (mpu.testConnection() != true) {
		lcd.setCursor(0, 1);
		lcd.print("MPU disconection");
		while (true) {}
	}
	if (mpu.dmpInitialize() != 0) {
		lcd.setCursor(0, 1);
		lcd.print("MPU break");
		while (true) {}
	}
	mpu.setXGyroOffset(106);
	mpu.setYGyroOffset(-7);
	mpu.setZGyroOffset(-3);
	mpu.setZAccelOffset(1108);
	mpu.setDMPEnabled(true);
	mpuIntStatus = mpu.getIntStatus();
	dmpReady = true;
	packetSize = mpu.dmpGetFIFOPacketSize();
}

void PID_Start() {
	myPID.SetOutputLimits(-255, 255);
	myPID.SetMode(AUTOMATIC);
	myPID.SetSampleTime(15);
	Setpoint = 180;
}

void LINE_Set(uint16_t val) {
	LINE_NOW = val;
	uint8_t buf[2];
	buf[0] = (val >> 8) & 0x00ff;
	buf[1] = val & 0x00ff;
	Wire.beginTransmission(11);
	Wire.write(buf, 2);
	Wire.endTransmission();
}