//////////////////////////////////
///RCJ  Revolution_Include 2017///
///      Main　永山和樹        ///
//////////////////////////////////


/*Include*/
#include "Moter.h"
#include <Servo.h>
#include <Utility.h>
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <LiquidCrystal_I2C.h>
/*ここまで*/

/*define*/
#define M_sw 0
#define SPEAKER 13
#define BEEP 100
#define Old_Persent 0
#define L_sw 4 
#define D_sw 7
#define R_sw 10
#define LED_L A3
#define LED_M A2
#define LED_R A1
#define Check_Delay 100
#define UI_Delay 200
#define IR_offset 0

#define LED(a) digitalWrite(a, HIGH)
#define LEDoff(a) digitalWrite(a, LOW)
#define Servo_idel Dri_P=85
#define Servo_Dri Dri_P=120
/*ここまで*/

/*川野さんからコピペ関数宣言*/


MPU6050 mpu;
Servo myServo1;
Servo myServo2;
static uint8_t mpuIntStatus;
static bool dmpReady = false;  // set true if DMP init was successful
static uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
/*ここまで*/

/*リキッドクリスタル関数宣言*/
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address													
/*ここまで*/

/*変数宣言*/
int16_t  Gyro_Now = 0, Gyro = 0, Gyro_Offset = 0, old_M_D = 0;
uint8_t LINE_Status = 0, UI_status = 0;
uint16_t IR_F = 0, IR_D = 0, old_M_F = 0;

boolean change1 = true, change2 = false;
/*ここまで*/

//プロトタイプ宣言(不必要のため途中からコメントアウト　エラー起こしたらしてみて
/*
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
	// Serial.begin(115200);
	Wire.begin();
	i2c_faster();

	PID_Start();

	lcd_Start("2.1.2 LINE");//lcd初期化関数

	Gryo_Start();

	Servo_Start();
	
	LINE_Set(48);

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
		IR_Get();
		// LINE_Get();
		Motion_System(IR_F, IR_D);
		//Moter.nomal(IR_F, IR_D);
	}
	else {
		if (change2) {
			lcd.backlight();
			change1 = true;
			change2 = false;
			myServo1.write(0);
		}
		Moter.off();
		UI();
	}
}

/*--自作関数--*/
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

void IR_Get() {
	Wire.requestFrom(9, 4);
	while (Wire.available()) {
		IR_F = (Wire.read() << 8) | Wire.read(); // Force Read
		IR_D = (Wire.read() << 8) | Wire.read(); // Degree Read
	}/*
	 Serial.print(IR_F);
	 Serial.println(IR_D);*/
}

void Motion_System(uint8_t Force, int16_t Degree) { //挙動制御 Force=IR_F Degree=IR_D
	int16_t Deg = 0,Dri_P;
	uint8_t	For = 200;
	Degree = Degree - IR_offset;
	if (Force != 0) {  // Ball Found                                           //ここから挙動制御
		if ((270 <= Degree) && (Degree < 290)) {                        //a
			Deg = Degree - 55;
			Servo_idel;
		}
		else if ((290 <= Degree) && (Degree < 340)) {              //b
			Deg = Degree - 42;
			Servo_idel;
		}
		else if ((340 <= Degree) && (Degree < 360)) {               //c
			Deg = Degree - 40;
			Servo_idel;
		}
		else if ((0 <= Degree) && (Degree < 50)) {               //d
			Deg = Degree - 45;
			Servo_idel;
		}
		else if ((50 <= Degree) && (Degree < 60)) {              //e
			Deg = Degree - 20;
			Servo_idel;
		}
		else if ((60 <= Degree) && (Degree < 70)) {              //f
			Deg = Degree - 14;
			Servo_idel;
		}
		else if ((70 <= Degree) && (Degree < 80)) {              //g
			Deg = Degree - 8;
			Servo_idel;
		}
		else if ((80 <= Degree) && (Degree < 100)) {               //真ん中
			Deg = 84;
			Servo_idel;
		}
		else if ((100 <= Degree) && (Degree < 110)) {              //g
			Deg = Degree + 8;
			Servo_idel;
		}
		else if ((110 <= Degree) && (Degree < 120)) {              //f
			Deg = Degree + 14;
			Servo_idel;
		}
		else if ((120 <= Degree) && (Degree < 130)) {              //e
			Deg = Degree + 20;
			Servo_idel;
		}
		else if ((130 <= Degree) && (Degree < 180)) {               //d
			Deg = Degree + 45;
			Servo_idel;
		}
		else if ((180 <= Degree) && (Degree < 200)) {               //c
			Deg = Degree + 40;
			Servo_idel;
		}
		else if ((200 <= Degree) && (Degree < 250)) {             //b
			Deg = Degree + 42;
			Servo_idel;
		}
		else if ((250 <= Degree) && (Degree < 270)) {              //a
			Deg = Degree + 55;
			Servo_Dri;
		}
		if (Deg < 0) {
			Deg = 360 + Deg;
		}
	}
	else {
		Deg = 90;
		For = 0;
		myServo1.write(0);
		myServo2.write(0);
		Dri_P = 0;
	}
	myServo1.write(Dri_P);
	Moter.nomal(For, Deg);
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

	if (bitRead(LINE_Status, 4) == 1) {
		digitalWrite(LED_L, HIGH);
		if (bitRead(LINE_Status, 0) == 1) {//右
			if (bitRead(LINE_Status, 1) == 1) {//右かつ後ろ
				Moter.nomal(255, 135);
			}
			else if (bitRead(LINE_Status, 2) == 1) {//右かつ左
				return;
			}
			else if (bitRead(LINE_Status, 3) == 1) { //右かつ前
				Moter.nomal(255, 215);
			}
			else {//右のみ
				Moter.nomal(255, 180);
			}
		}
		else if (bitRead(LINE_Status, 1) == 1) { //後ろ
			if (bitRead(LINE_Status, 3) == 1) { //後ろかつ前
				return;
			}
			else if (bitRead(LINE_Status, 2) == 1) { //後ろかつ左
				Moter.nomal(255, 45);
			}
			else {//後ろのみ
				Moter.nomal(255, 90);
			}
		}
		else if (bitRead(LINE_Status, 2) == 1) {//左
			if (bitRead(LINE_Status, 3) == 1) { //左かつ前
				Moter.nomal(255, 315);
			}
			else { //左のみ
				Moter.nomal(255, 0);
			}
		}
		else if (bitRead(LINE_Status, 3) == 1) {//前のみ
			Moter.nomal(255, 270);
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
			UI_status = 1;
			lcd.clear();
			delay(UI_Delay);
		}
		break;


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

	case 2:
		lcd.home();
		lcd.print("Dribler test");
		lcd.setCursor(0, 1);
		lcd.print("L:1 D:next R:2");
		if (L) {
			UI_status = 3;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 4;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
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
		lcd.print("L: D:next R:Yes");
		if (R) {
			LED_Check();
		}
		else if (D) {
			UI_status = 6;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 6:
		lcd.home();
		lcd.print("Gyro&IR Monitor");
		lcd.setCursor(0, 1);
		lcd.print("L: D:exit R:Yes");
		if (R) {
			UI_status = 7;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			UI_status = 1;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 7:
		GyroGet();
		IR_Get();
		lcd.clear();
		lcd.home();
		lcd.print(IR_D);
		lcd.setCursor(0, 1);
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
		lcd.print("L: D:next R:Yes");
		if (R) {
			UI_status = 9;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			UI_status = 2;
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
		lcd.print("L:Reset D:set R:exit");
		delay(100);
		if (D) {
			Gyro_Offset = 180 - Gyro_Now;
			lcd.setCursor(0, 1);
			lcd.print("Set completed!");
			delay(1000);
			UI_status = 1;
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
	default:
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
	uint8_t buf[2];
	buf[0] = (val >> 8) & 0x00ff;
	buf[1] = val & 0x00ff;
	Wire.beginTransmission(11);
	Wire.write(buf, 2);
	Wire.endTransmission();
}