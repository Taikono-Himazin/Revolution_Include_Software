//////////////////////////////////
///RCJ  Revolution_Include 2017///
///      Main　永山和樹        ///
//////////////////////////////////


/*Include*/
#include <VL53L0X.h>
#include <HMC5883L.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Utility.h>
#include <Wire.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
/*ここまで*/

/*define*/
#define M_sw 0
#define SPEAKER A0
#define BEEP 100
#define Old_Persent 0.1  //1以下！ Moterの過去の値の割合 
#define L_sw 4 
#define D_sw 7
#define R_sw 10
#define LED_L A3
#define LED_M A2
#define LED_R A1
#define UI_Delay 200
#define Check_Delay 100
#define IR_offset 5
#define  C_Reset 30
#define  C_Reset2 10
#define Escape 150

#define LED(a) digitalWrite(a, HIGH)
#define LEDoff(a) digitalWrite(a, LOW)
#define Servo_idel Dri1_Power=85;Dri2_Power=Dri1_Power
#define Servo1_Dri Dri1_Power=180;Dri2_Power=85
#define Servo2_Dri Dri2_Power=180;Dri1_Power=85
/*ここまで*/

/*川野さんからコピペ関数宣言*/
static double Setpoint, Input, Output;
static const double Kp = 2, Ki = 0, Kd = 0.001;
HMC5883L HMC;
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
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
int16_t  HMC_Now = 0, HMC_val = 0, HMC_Offset = 0, old_Moter_D = 0, Vl53L0X_LINE = 0;
uint8_t LINE_Status = 0, UI_status = 0, LINE_count;
uint16_t IR_F = 0, IR_D = 0, old_Moter_F = 0, LINE_NOW, VL53L0X_F, VL53L0X_R, VL53L0X_L, VL53L0X_B;
uint16_t LINE_M, LINE_D, LINE_RL = 150, LINE_FB = 50, LINE_RL2 = 200, LINE_FB2 = 20;
boolean change1 = true, change2 = false, LINE_F = false, LINE_R = false, LINE_B = false, LINE_L = false;
/*ここまで*/

//プロトタイプ宣言(不必要のため途中からコメントアウト　エラー起こしたらしてみて
void setup();
extern void loop();
extern void moter(uint8_t Force, int16_t Degree, bool PID = true);
extern void HMC_Get();
extern void sleep();
extern void IR_Get();
extern void Motion_System(uint8_t Force, int16_t Degree);
extern void Spin(boolean D);
extern void Melody(uint8_t mode);
extern void LINE_Get();
extern void UI();
extern void lcd_Start(char* ver);
extern void LED_Check();
extern void Servo_Start();
extern void HMC_Start();
extern void PID_Start();
extern void LINE_Set(uint16_t val);
extern void VL53L0X_Start();
extern void VL53L0X_Get();
/*ここまで*/

/*--プログラム--*/
void setup() {
	//Serial.begin(115200);
	Wire.begin();
	i2c_faster();

	lcd_Start("2.1.8C");
	VL53L0X_Start();

	PID_Start();
	HMC_Start();
	Servo_Start();
	
	uint16_t val = (EEPROM.read(1) << 8) | EEPROM.read(2); // LINE閾値読み込み
//	LINE_Set(val);

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

	UI_status = 11;
	Melody(0);
}

void loop() {
	if (digitalRead(M_sw) == LOW) {
		if (change1) {
			//	lcd.noBacklight();
			change1 = false;
			change2 = true;
			LEDoff(LED_R);
			LEDoff(LED_M);
			LEDoff(LED_L);
		}
		//	int16_t i1 = millis();
		LINE_Get();
		if (LINE_count == 0) {
			IR_Get();
			Motion_System(IR_F, IR_D);
		}
		else {
			LINE_count--;
		}
		//int16_t i2 = millis();
		//Serial.println(i2 - i1);
		//sleep();
		//delay(100);
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
void moter(uint8_t Force, int16_t Degree, bool PID = true) { //一応解読したがいじれるほどはわからん。とりあえず同じ形ならそのままいこう

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
	if (PID) {
		HMC_Get();//ジャイロのデータを取得
		Input = HMC_val; //ジャイロのデータをInput関数に突っ込む
		myPID.Compute(); //pid計算

		m1 = m1 - Output;//pidのデータをモーターに突っ込む
		m2 = m2 - Output;
		m3 = m3 - Output;
		m4 = m4 - Output;
		m1 = constrain(m1, -255, 255);
		m2 = constrain(m2, -255, 255);
		m3 = constrain(m3, -255, 255);
		m4 = constrain(m4, -255, 255);
	}

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

	Wire.beginTransmission(80);
	Wire.write(buf, 5);
	Wire.endTransmission();
}

void HMC_Get() {
	Vector norm = HMC.readNormalize();
	float heading = atan2(norm.YAxis, norm.XAxis);	// Calculate heading
													// Set declination angle on your location and fix heading
													// You can find your declination on: http://magnetic-declination.com/
													// (+) Positive or (-) for negative
													// For Bytom / Poland declination angle is 4'26E (positive)
													// Formula: (deg + (min / 60.0)) / (180 / M_PI);
	static const float declinationAngle = (-7.0 + (29.0 / 60.0)) / (180 / M_PI);
	heading += declinationAngle;

	if (heading < 0) heading += TWO_PI;
	if (heading > TWO_PI) heading -= TWO_PI;
	HMC_Now = degrees(heading);	// Convert to degrees

	HMC_val = HMC_Now + HMC_Offset;
	if (HMC_val < 0)   HMC_val += 360;
	if (HMC_val > 359) HMC_val -= 360;
}

/*void GyroGet() {
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
}*/

void sleep() {
	uint8_t buf[5];//モータに送信
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	bitClear(buf[4], 4); //モーター電源off

	Wire.beginTransmission(80);
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
	uint8_t	M_Force = 180;
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

	VL53L0X_Get();

	double rad = M_Degree*3.141592653589793 / 180.0;//角度のラジアン
	if (bitRead(Vl53L0X_LINE, 2) == 1 && M_Degree >= 180) {//後ろのライン処理
		M_Force = abs(M_Force*cos(rad));
		if (M_Degree >= 180 && M_Degree <= 270) {
			M_Degree = 180;
		}
		else {
			M_Degree = 0;
		}
	}

	if (bitRead(Vl53L0X_LINE, 3) == 1 && (M_Degree >= 0 && M_Degree < 180)) {//前のライン処理
		M_Force = abs(M_Force*cos(rad));
		if (M_Degree >= 0 && M_Degree <= 90) {
			M_Degree = 0;
		}
		else {
			M_Degree = 180;
		}
	}

	if (bitRead(Vl53L0X_LINE, 1) == 1 && (M_Degree >= 90 && M_Degree < 270)) {//左のライン処理
		M_Force = abs(M_Force*sin(rad));
		if (M_Degree >= 90 && M_Degree >= 180) {
			M_Degree = 90;
		}
		else {
			M_Degree = 270;
		}
	}

	if (bitRead(Vl53L0X_LINE, 0) == 1 && (M_Degree < 90 && M_Degree >= 270)) {//右のライン処理
		M_Force = abs(M_Force*sin(rad));
		if (M_Degree >= 270) {
			M_Degree = 270;
		}
		else {
			M_Degree = 90;
		}
	}

	if (bitRead(Vl53L0X_LINE, 4) == 1) {//右が反応
		M_Degree = 180;
		M_Force = Escape;
	}
	else if (bitRead(Vl53L0X_LINE, 5) == 1) {//左が反応
		M_Degree = 0;
		M_Force = Escape;
	}
	else if (bitRead(Vl53L0X_LINE, 6) == 1) {//後ろが反応
		M_Degree = 90;
		M_Force = Escape;
	}
	else if (bitRead(Vl53L0X_LINE, 7) == 1) {//前が反応
		M_Degree = 270;
		M_Force = Escape;
	}




	if (count <= 0) {
		Spin(true);
		count = C_Reset;
		count2 = C_Reset2;
	}
	else {
		moter(M_Force, M_Degree);
		myServo1.write(Dri1_Power);
		myServo2.write(Dri2_Power);
	}

}

void Spin(boolean D) {
	uint8_t m1, m2, m3, m4;
	int8_t i[2];
	if (D) {
		i[0] = 100;
		i[1] = 255;
	}
	else {
		i[0] = -100;
		i[1] = -255;
	}
	m1 = i[0];
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

	Wire.beginTransmission(80);
	Wire.write(buf, 5);
	Wire.endTransmission();

	delay(200);

	m1 = i[1];
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

	Wire.beginTransmission(80);
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
	LINE_F = false, LINE_R = false, LINE_B = false, LINE_L = false;
	LINE_Status = buf;
	//	Serial.println(LINE_Status,BIN);
	if (bitRead(LINE_Status, 4) == 1) {
		digitalWrite(LED_L, HIGH);
		LINE_count = 70;
		if (bitRead(LINE_Status, 0) == 1) {//右
			LINE_R = true;
			if (bitRead(LINE_Status, 1) == 1) {//右かつ後ろ
				moter(255, 135);
				LINE_M = 255; LINE_D = 135;
			}
			else if (bitRead(LINE_Status, 2) == 1) {//右かつ左
				LINE_count = 0;
				return;
			}
			else if (bitRead(LINE_Status, 3) == 1) { //右かつ前
				moter(255, 215);
				LINE_M = 255; LINE_D = 215;
			}
			else {//右のみ
				moter(255, 180);
				LINE_M = 255; LINE_D = 180;
			}
		}
		if (bitRead(LINE_Status, 1) == 1) { //後ろ
			LINE_B = true;
			if (bitRead(LINE_Status, 3) == 1) { //後ろかつ前
				LINE_count = 0;
				return;
			}
			else if (bitRead(LINE_Status, 2) == 1) { //後ろかつ左
				moter(255, 45);
				LINE_M = 255; LINE_D = 45;
			}
			else {//後ろのみ
				moter(255, 90);
				LINE_M = 255; LINE_D = 90;
			}
		}
		if (bitRead(LINE_Status, 2) == 1) {//左
			LINE_L = true;
			if (bitRead(LINE_Status, 3) == 1) { //左かつ前
				moter(255, 315);
				LINE_M = 255; LINE_D = 315;
			}
			else { //左のみ
				moter(255, 0);
				LINE_M = 255; LINE_D = 0;
			}
		}
		if (bitRead(LINE_Status, 3) == 1) {//前のみ
			LINE_F = true;
			moter(255, 270);
			LINE_M = 255; LINE_D = 270;
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
		/*	case 0:
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
			UI_status = 9;
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
			UI_status = 9;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 7:
		HMC_Get();
		IR_Get();
		lcd.clear();
		lcd.home();
		lcd.print("IR   ");
		lcd.print(IR_D);
		lcd.setCursor(0, 1);
		lcd.print("HMC ");
		lcd.print(HMC_val);
		if (L || D || R) {
			UI_status = 10;
			lcd.clear();
			delay(UI_Delay);
		}
		delay(30);
		break;
		/*	case 8:
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
		break;*/
	case 9:
		lcd.home();
		lcd.print("HMC             ");
		HMC_Get();
		lcd.setCursor(6, 0);
		lcd.print(HMC_val);
		lcd.setCursor(0, 1);
		lcd.print("L:Re D:set R:next");
		delay(100);
		if (D) {
			HMC_Offset = 180 - HMC_Now;
			lcd.setCursor(0, 1);
			lcd.print("                ");
			lcd.setCursor(0, 1);
			lcd.print("Set completed!");
			delay(1000);
			UI_status = 7;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (L) {
			HMC_Start();
		}
		else if (R) {
			UI_status = 7;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 10:
		lcd.home();
		lcd.print("LINE_Val:       ");
		lcd.setCursor(9, 0);
		lcd.print(LINE_NOW);
		lcd.setCursor(0, 1);
		lcd.print("L:up D:down R:next");
		if (L&&D) {
			LINE_Set(140);
		}
		else if (L) {
			LINE_Set(LINE_NOW + 10);
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 11;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			LINE_Set(LINE_NOW - 10);
			delay(UI_Delay);
		}
		break;
	case 11:
		lcd.home();
		lcd.print("VL53L0X         ");
		lcd.setCursor(0, 1);
		lcd.print("                ");
		lcd.setCursor(0, 1);
		VL53L0X_Get();
		lcd.print(VL53L0X_F);
		lcd.print(",");
		lcd.print(VL53L0X_L);
		lcd.print(",");
		lcd.print(VL53L0X_R);
		lcd.print(",");
		lcd.print(VL53L0X_B);
		if (R || D || L) {
			UI_status = 12;
			lcd.clear();
			delay(UI_Delay);
		}
		delay(100);
		break;
	case 12:
		lcd.home();
		lcd.print("LINE_RL:       ");
		lcd.setCursor(8, 0);
		lcd.print(LINE_RL);
		lcd.setCursor(0, 1);
		lcd.print("L:up D:down R:next");
		if (L&&D) {
			LINE_RL = 150;
		}
		else if (L) {
			LINE_RL += 10;
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 13;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			LINE_RL -= 10;
			delay(UI_Delay);
		}
		break;
	case 13:
		lcd.home();
		lcd.print("LINE_RL2:       ");
		lcd.setCursor(9, 0);
		lcd.print(LINE_RL2);
		lcd.setCursor(0, 1);
		lcd.print("L:up D:down R:next");
		if (L&&D) {
			LINE_RL2 = 150;
		}
		else if (L) {
			LINE_RL2 += 10;
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 14;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			LINE_RL2 -= 10;
			delay(UI_Delay);
		}
		break;
	case 14:
		lcd.home();
		lcd.print("LINE_FB:       ");
		lcd.setCursor(8, 0);
		lcd.print(LINE_FB);
		lcd.setCursor(0, 1);
		lcd.print("L:up D:down R:next");
		if (L&&D) {
			LINE_FB = 50;
		}
		else if (L) {
			LINE_FB += 10;
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 15;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			LINE_FB -= 10;
			delay(UI_Delay);
		}
		break;
	case 15:
		lcd.home();
		lcd.print("LINE_FB2:       ");
		lcd.setCursor(9, 0);
		lcd.print(LINE_FB2);
		lcd.setCursor(0, 1);
		lcd.print("L:up D:down R:next");
		if (L&&D) {
			LINE_FB2 = 50;
		}
		else if (L) {
			LINE_FB2 += 10;
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 2;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			LINE_FB2 -= 10;
			delay(UI_Delay);
		}
		break;
	default:
		lcd.clear();
		lcd.print("ERRER AUTO REPAIR");
		delay(1000);
		UI_status = 2;
		break;
	}

}

void VL53L0X_Get() {
	VL53L0X_R = sensor1.readRangeContinuousMillimeters();
	VL53L0X_L = sensor2.readRangeContinuousMillimeters();
	VL53L0X_F = sensor3.readRangeContinuousMillimeters();
	VL53L0X_B = sensor4.readRangeContinuousMillimeters();


	Vl53L0X_LINE = 0;
	if (VL53L0X_L >= LINE_RL&&VL53L0X_L < 3500) {//左の間隔が広くなる(右がLINEを踏む
		bitSet(Vl53L0X_LINE, 0);
		if (VL53L0X_L >= LINE_RL2) {//さらに左が広くなる(右がLINEを超えた
			bitSet(Vl53L0X_LINE, 4);
		}
	}
	if (VL53L0X_R >= LINE_RL&&VL53L0X_R < 3500) {//右の間隔が広くなる(左がLINEを踏む
		bitSet(Vl53L0X_LINE, 1);
		if (VL53L0X_R >= LINE_RL2) {//さらに右が広くなる(左がLINEを超えた
			bitSet(Vl53L0X_LINE, 5);
		}
	}
	if (VL53L0X_B <= LINE_FB&&VL53L0X_B < 3500) {//後ろの間隔が狭くなる
		bitSet(Vl53L0X_LINE, 2);
		if (VL53L0X_B <= LINE_FB2) {//さらに後ろの間隔が狭くなる
			bitSet(Vl53L0X_LINE, 6);
		}
	}
	if (VL53L0X_F <= LINE_FB&&VL53L0X_F < 3500) {//前の間隔が狭くなる
		bitSet(Vl53L0X_LINE, 3);
		if (VL53L0X_F <= LINE_FB2) {//さらに前の間隔が狭くなる
			bitSet(Vl53L0X_LINE, 7);
		}
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

void HMC_Start() {
	HMC.setRange(HMC5883L_RANGE_1_3GA);	// Set measurement range
	HMC.setMeasurementMode(HMC5883L_CONTINOUS);	// Set measurement mode
	HMC.setDataRate(HMC5883L_DATARATE_75HZ);	// Set data rate
	HMC.setSamples(HMC5883L_SAMPLES_8);	// Set number of samples averaged
	HMC.setOffset(0, 0);	// Set calibration offset. See HMC5883L_calibration.ino
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
	EEPROM.write(1, buf[0]);
	EEPROM.write(2, buf[1]);
}

void VL53L0X_Start() {
	pinMode(12, OUTPUT);
	pinMode(13, OUTPUT);
	pinMode(11, OUTPUT);
	digitalWrite(12, LOW);
	digitalWrite(13, LOW);
	digitalWrite(11, LOW);


	sensor4.init();//後ろ
	sensor4.setTimeout(500);
	sensor4.setSignalRateLimit(0.1);
	sensor4.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
	sensor4.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
	sensor4.setMeasurementTimingBudget(200000);
	sensor4.startContinuous();
	sensor4.setAddress(1);

	digitalWrite(12, HIGH);//右
	sensor1.init();
	sensor1.setTimeout(500);
	sensor1.setSignalRateLimit(0.1);
	sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
	sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
	sensor1.setMeasurementTimingBudget(200000);
	sensor1.startContinuous();
	sensor1.setAddress(2);

	digitalWrite(11,HIGH);//左
	sensor2.init();
	sensor2.setTimeout(500);
	sensor2.setSignalRateLimit(0.1);
	sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
	sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
	sensor2.setMeasurementTimingBudget(200000);
	sensor2.startContinuous();
	sensor2.setAddress(3);

	digitalWrite(13, HIGH);//前
	sensor3.init();
	sensor3.setTimeout(500);
	sensor3.setSignalRateLimit(0.1);
	sensor3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
	sensor3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
	sensor3.setMeasurementTimingBudget(200000);
	sensor3.startContinuous();
	sensor3.setAddress(4);
}