//////////////////////////////////
///RCJ  Revolution_Include 2017///
///      Main�@�i�R�a��        ///
//////////////////////////////////


/*Include*/
#include <HMC5883L.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Utility.h>
#include <Wire.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
/*�����܂�*/

/*define*/
#define Gyro_Mode false //�W���C�����[�htrue�Ȃ�W���C���Afalse�Ȃ�R���p�X
#define M_sw 0
#define SPEAKER A0
#define BEEP 100
#define Old_Persent 0.1  //1�ȉ��I Moter�̉ߋ��̒l�̊��� 
#define L_sw 4 
#define D_sw 7
#define R_sw 10
#define LED_L A3
#define LED_M A2
#define LED_R A1
#define Check_Delay 100
#define IR_offset 0
#define UI_Delay 200
#define HC_F 11//�O
#define HC_B 12//���
#define HC_L 13//��
#define HC_R 5//�E

#define LED(a) digitalWrite(a, HIGH)
#define LEDoff(a) digitalWrite(a, LOW)
/*�����܂�*/

/*��삳�񂩂�R�s�y�֐��錾*/
static double Setpoint, Input, Output;
static const double Kp = 3.5, Ki = 0, Kd = 0.01;
#if Gyro_Mode
MPU6050 mpu;
#else // Gyro_Mode
HMC5883L HMC;
#endif
Servo myServo1;
Servo myServo2;
static uint8_t mpuIntStatus;
static bool dmpReady = false;  // set true if DMP init was successful
static uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
/*�����܂�*/

/*���L�b�h�N���X�^���֐��錾*/
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
/*�����܂�*/

/*�ϐ��錾*/
#if Gyro_Mode
int16_t  Gyro_Now = 0, Gyro = 0, Gyro_Offset = 0;
uint16_t fifoCount;
uint8_t fifoBuffer[64]; // FIFO storage buffer							   // orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#else
int16_t HMC_Now = 0, HMC_val = 0, HMC_Offset = 0;
#endif

int16_t old_Moter_D = 0, Gyro_Old=0, Gyro_Old_ms = 0;
uint8_t LINE_Status = 0, UI_status = 0, Errer_Flag_Status = 0,
LINE_F = 0, LINE_R = 0, LINE_B = 0, LINE_L = 0, Back_count=0;
uint16_t IR_F = 0, IR_D = 0, old_Moter_F = 0, M_P,LINE_NOW, F=30, B=30, L=30, R=30;

bool change1 = true, change2 = false,Errer_Flag = false;
/*�����܂�*/

//�v���g�^�C�v�錾(�s�K�v�̂��ߓr������R�����g�A�E�g�@�G���[�N�������炵�Ă݂�
extern void setup();
extern void loop();
extern void moter(uint8_t Force, int16_t Degree, bool PID = true);
extern void GyroGet();
extern void HMC_Get();
extern void sleep();
extern inline void IR_Get();
extern inline void Motion_System(uint8_t Force, int16_t Degree);
extern void Melody(uint8_t mode);
extern void LINE_Get();
extern uint32_t HC_Get(uint8_t pin);
extern void UI();
extern inline void lcd_Start(char* ver);
extern inline void LED_Check();
extern void Servo_Start();
extern void Gryo_Start();
extern void HMC_Start();
extern void PID_Start();
/*�����܂�*/

/*--�v���O����--*/
void setup() {
	//Serial.begin(115200);
	Wire.begin();
	i2c_faster();

	PID_Start();

	M_P = EEPROM.read(1); // M_P臒l�ǂݍ���

	uint16_t val = EEPROM.read(4) << 8 | EEPROM.read(5);
	LINE_Set(val);

	Servo_Start();
	
	lcd_Start("2.3.1_Neo_NM");//lcd�������֐�

#if Gyro_Mode
	Gryo_Start();
#else
	HMC_Start();
#endif

	while (digitalRead(M_sw) == LOW) {
		Melody(1);
		delay(1000);
		lcd.setCursor(0, 1);
		lcd.print("Please turn off");
	}

	LED_Check();

	pinMode(A6, INPUT);
	pinMode(A7, INPUT);

	lcd.setCursor(0, 1);
	lcd.print("               ");
	lcd.setCursor(0, 1);
	lcd.print("ALL OK");

	UI_status = 2;
	Melody(0);
}

void loop() {
	if (digitalRead(M_sw) == LOW&&!Errer_Flag) {
		if (change1) {
			lcd.noBacklight();
			change1 = false;
			change2 = true;
			LINE_Set(LINE_NOW);
			LEDoff(LED_R);
			LEDoff(LED_M);
			LEDoff(LED_L);
		}
		IR_Get();
		Motion_System(IR_F, IR_D);
	}
	else {
		if (Errer_Flag) {
			delay(100);
			Errer_Flag = false;
			LEDoff(LED_R);
		}
		else {
			if (change2) {
				lcd.backlight();
				myServo1.write(0);
				myServo2.write(0);
				change1 = true;
				change2 = false;
			}
			sleep();
			UI();
		}
	}
}

/*--����֐�--*/
void moter(uint8_t Force, int16_t Degree,bool PID) { //�ꉞ��ǂ������������قǂ͂킩���B�Ƃ肠���������`�Ȃ炻�̂܂܂�����

	static int16_t Gyro_Old;

	int16_t m1, m2;
	old_Moter_D = old_Moter_D*Old_Persent + Degree*(1 - Old_Persent);
	old_Moter_F = old_Moter_F*Old_Persent + Force*(1 - Old_Persent);

	int16_t m1_D = old_Moter_D - 45;
	if (m1_D < 0) m1_D = m1_D + 360;
	else if (m1_D > 359) m1_D = m1_D - 360;

	m1 = sin((float)m1_D * 0.01745329) * old_Moter_F; // sin �ł�cos����Ȃ��Ɨ���s�\

	int16_t m2_D = old_Moter_D - 315;
	if (m2_D < 0) m2_D = m2_D + 360;
	else if (m2_D > 359) m2_D = m2_D - 360;

	m2 = sin((float)m2_D * 0.01745329) * old_Moter_F;

	int16_t F_max = abs(m1);
	if (F_max < abs(m2)) F_max = abs(m2);

	float k = (float)old_Moter_F / F_max;//�e���[�^�[�̔��ۂ��Ȃ���ő�l��225��
	m1 = m1 * k;
	m2 = m2 * k;

	int16_t m3 = -m1;//���΂Ɉʒu���Ă��邩�甽�]
	int16_t m4 = -m2;
	if (PID){
#if Gyro_Mode
		GyroGet();//�W���C���̃f�[�^���擾
	Input = Gyro;
#else
		HMC_Get();
	Input = HMC_val;
#endif
	myPID.Compute(); //pid�v�Z
}

	m1 = m1 - Output;//pid�̃f�[�^�����[�^�[�ɓ˂�����
	m2 = m2 - Output;
	m3 = m3 - Output;
	m4 = m4 - Output;
	m1 = constrain(m1, -255, 255);
	m2 = constrain(m2, -255, 255);
	m3 = constrain(m3, -255, 255);
	m4 = constrain(m4, -255, 255);

	/*lcd.home();
	lcd.print(m1);
	lcd.print(",");
	lcd.print(m2);
	lcd.setCursor(0, 1);
	lcd.print(m3);
	lcd.print(",");
	lcd.print(m4);
	lcd.print(",");*/

	uint8_t buf[5];//���M
	bitSet(buf[4], 4); //���[�^�̓d��on

	if (m1 < 0) bitSet(buf[4], 0);
	else bitClear(buf[4], 0);
	buf[0] = abs(m1);

	if (m2 < 0) bitSet(buf[4], 1);//���[�^�[�̔z�����ԈႦ��
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
#if Gyro_Mode
		if ((Gyro <= 140 || Gyro >= 220)&&Errer_Flag_Status==0) {
				uint16_t i = millis();
				if ((i - Gyro_Old_ms < 400)&&i - Gyro_Old_ms > 100 && ((180-Gyro_Old > 0) != (180-Gyro > 0))) {
					Errer_Flag = true;
					LED(LED_R);
				}
				Gyro_Old_ms = i;
				Gyro_Old = Gyro;
		}
		else if(Errer_Flag_Status!=0){
			Errer_Flag_Status--;
		}
#endif
}

#if Gyro_Mode
void GyroGet() {
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

#else
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
#endif

void sleep() {
	uint8_t buf[5];//���[�^�ɑ��M
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	bitClear(buf[4], 4); //���[�^�[�d��off

	Wire.beginTransmission(10);
	Wire.write(buf, 5);
	Wire.endTransmission();
}

inline void IR_Get() {
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

inline void Motion_System(uint8_t Force, int16_t Degree) { //�������� Force=IR_F Degree=IR_D
	int16_t M_Degree = 0, Dri1_Power = 0, Dri2_Power = 0;
	uint8_t	M_Force = M_P;
	bool Ball1 = analogRead(A6) >= 1000;
	//bool Ball2 = analogRead(A7) >= 1000;
	if (Ball1) {
		LED(LED_M);
	}
	else {
		LEDoff(LED_M);
	}

#define Servo_idel Dri1_Power=0;Dri2_Power=Dri1_Power
#define Servo1_Dri Dri1_Power=180;Dri2_Power=0
#define Servo2_Dri Dri2_Power=0;Dri1_Power=85

	if (Force != 0) {  // Ball Found      //�������狓������                       //a
		Back_count = 0;
		if ((270 <= Degree) && (Degree < 290)) {					//5
			M_Degree = Degree-70;
			Servo2_Dri;
		}
		else if ((290 <= Degree) && (Degree < 330)) {				//6
			M_Degree = Degree-65;
			Servo_idel;
		}
		else if ((330 <= Degree) && (Degree < 360)) {               //7
			M_Degree = Degree - 60;
		}
		else if ((0 <= Degree) && (Degree < 20)) {               //d
			M_Degree = Degree - 65;
		}
		else if ((20 <= Degree) && (Degree < 50)) {               //d
			M_Degree = Degree - 55;
		}
		else if ((50 <= Degree) && (Degree < 70)) {              //e
			M_Degree = Degree - 40;
		}
		else if ((70 <= Degree) && (Degree < 80)) {              //g
			M_Degree = Degree - 15;
		}
		else if ((80 <= Degree) && (Degree < 100)) {               //�^��
			M_Degree = 95;
		}
		else if ((100 <= Degree) && (Degree < 110)) {              //g
			M_Degree = Degree + 15;
		}
		else if ((110 <= Degree) && (Degree < 130)) {              //f
			M_Degree = Degree + 40;
		}
		else if ((130 <= Degree) && (Degree < 160)) {              //e
			M_Degree = Degree + 55;
		}
		else if ((160 <= Degree) && (Degree < 180)) {               //d
			M_Degree = Degree + 65;
		}
		else if ((180 <= Degree) && (Degree < 210)) {               //c
			M_Degree = Degree + 60;
		}
		else if ((210 <= Degree) && (Degree < 250)) {             //b
			M_Degree = Degree + 65;
			Servo_idel;
		}
		else if ((250 <= Degree) && (Degree < 270)) {
			M_Degree = Degree + 70;
			Servo2_Dri;
		}		//��������

		if (M_Degree < 0) {
			M_Degree = 360 + M_Degree;
		}
		else if (M_Degree >= 360) {
			M_Degree = 360 - M_Degree;
		}


	}
	else {
		M_Degree = 90;
		M_Force = 0;
		Dri1_Power = 0;
		Dri2_Power = 0;
#if !Gyro_Mode
		uint32_t i;/*= HC_Get(HC_F);
		if (i<90 && i != 0) {
			M_Degree = 280;
			M_Force = 150;
		}
		if (i > 130 && i == 0) {
			M_Degree = 90;
			M_Force = 150;
		}*/
		if (Back_count < 5) {
			M_Degree = 270;
			M_Force = 150;
			Back_count++;
		}
		i = HC_Get(HC_L);
		if (i > 100) {
			M_Degree = 180;
			M_Force = 150;
			Back_count = 0;
		}
		i = HC_Get(HC_R);
		if (i > 100) {
			M_Degree = 0;
			M_Force = 150;
			Back_count = 0;
		}

#endif

	}

	if (bitRead(LINE_Status, 4) == 1 && LINE_B == 0 && LINE_F == 0 && LINE_L == 0 && LINE_R == 0) {

#define LINE_EscapeCount 10 //LINE�̓�����J�E���g

		if (bitRead(LINE_Status, 0) == 1) {//�E
			LINE_R = LINE_EscapeCount + 10;
		}//�O���o�����
		if (bitRead(LINE_Status, 1) == 1) {//���
			LINE_B = LINE_EscapeCount;
		}
		if (bitRead(LINE_Status, 2) == 1) {//��
			LINE_L = LINE_EscapeCount + 10;
		}
		if (bitRead(LINE_Status, 3) == 1) {//�O
			LINE_F = LINE_EscapeCount + 30;
		}
	}

	LINE_Get();
	uint8_t count = 0;
	if (LINE_R != 0) {//�E
		while (count < 50)
		{
			LINE_Get();
			if (bitRead(LINE_Status, 1) == 1) {//���
				M_Degree = 135;
			}
			else if (bitRead(LINE_Status, 3) == 1) {//�O
				M_Degree = 225;
			}
			else {
				M_Degree = 180;
			}
			moter(255, M_Degree);
			count++;
		}
		LINE_R = 0;
	}
	if (LINE_B != 0) {//���
		while (count < 50)
		{
			LINE_Get();
			if (bitRead(LINE_Status, 0) == 1) {//�E
				M_Degree = 135;
			}
			else if (bitRead(LINE_Status, 2) == 1) {//��
				M_Degree = 45;
			}
			else {
				M_Degree = 90;
			}
			moter(255, M_Degree);
			count++;
		}
		LINE_B = 0;
	}
	if (LINE_L != 0) {//��
		while (count < 70)
		{
			LINE_Get();
			if (bitRead(LINE_Status, 1) == 1) {//���
				M_Degree = 45;
			}
			else if (bitRead(LINE_Status, 3) == 1) {//�O
				M_Degree = 315;
			}
			else {
				M_Degree = 0;
			}
			moter(255, M_Degree);
			count++;
		}
		LINE_L = 0;
	}
	if (LINE_F != 0) {//�O

		while (count < 70)
		{
			LINE_Get();
			if (bitRead(LINE_Status, 0) == 1) {//�E
				M_Degree = 215;
			}
			else if (bitRead(LINE_Status, 2) == 1) {//��
				M_Degree = 315;
			}
			else {
				M_Degree = 280;
			}
			moter(255, M_Degree);
			count++;
		}
		LINE_F = 0;
	}
	moter(M_Force, M_Degree);
	myServo1.write(Dri1_Power);
	myServo2.write(Dri2_Power);
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
	else if (mode == 2) {
		tone(SPEAKER, 1319, BEEP);
		delay(BEEP);
		noTone(SPEAKER);
	}
}

inline void LINE_Get() {
	Wire.requestFrom(11, 1);
	uint8_t buf;
	while (Wire.available()) {
		buf = Wire.read();
	}
	LINE_Status = buf;
	if (bitRead(LINE_Status, 4) == 1) {
		LED(LED_L);
	}
	else {
		LEDoff(LED_L);
	}
	//Serial.println(LINE_Status, BIN);
}

uint32_t HC_Get(uint8_t pin) {
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	delayMicroseconds(2);
	digitalWrite(pin, HIGH);
	delayMicroseconds(10);
	digitalWrite(pin, LOW);
	pinMode(pin, INPUT);
	return (pulseIn(pin, HIGH, 15000) / 58);//75�Z���`
}

void UI() {
	bool L = digitalRead(L_sw) == HIGH;
	bool D = digitalRead(D_sw) == HIGH;
	bool R = digitalRead(R_sw) == HIGH;
	switch (UI_status)
	{
	case 0:
		lcd.home();
		lcd.print("Dribler test");
		lcd.setCursor(0, 1);
		lcd.print("L:1 D:2 R:next");
		if (L) {
			LED(LED_R);
			myServo1.write(180);
			myServo2.write(0);
		}
		else if (D) {
			LED(LED_R);
			myServo1.write(0);
			myServo2.write(180);
		}
		else if (R) {
			LEDoff(LED_R);
			myServo1.write(0);
			myServo2.write(0);
			UI_status = 1;
			lcd.clear();
			delay(UI_Delay);
		}
		else {
			LEDoff(LED_R);
			myServo1.write(0);
			myServo2.write(0);
		}
		break;
	case 1:
#if Gyro_Mode
		GyroGet();
#else
		HMC_Get();
#endif
		IR_Get();
		lcd.clear();
		lcd.home();
		lcd.print("IR   ");
		lcd.print(IR_D);
		lcd.print(" , ");
		lcd.print(IR_F);
		lcd.setCursor(0, 1);
#if Gyro_Mode
		lcd.print("Gyro ");
		lcd.print(Gyro);
#else
		lcd.print("HMC ");
		lcd.print(HMC_val);
#endif 
		if (L || D || R) {
			UI_status = 2;
			lcd.clear();
			delay(UI_Delay);
		}
		delay(30);
		break;
	case 2:
		lcd.home();
#if Gyro_Mode
		lcd.print("Gyro             ");
		GyroGet();
		lcd.setCursor(6, 0);
		lcd.print(Gyro_Now);
#else
		lcd.print("HMC             ");
		HMC_Get();
		lcd.setCursor(5, 0);
		lcd.print(HMC_val);
#endif 
		lcd.setCursor(0, 1);
		lcd.print("L:Re D:set R:next");
		delay(100);
		if (D) {
#if Gyro_Mode
			Gyro_Offset = 180 - Gyro_Now;
#else
			HMC_Offset = 180 - HMC_Now;
#endif
			lcd.setCursor(0, 1);
			lcd.print("                ");
			lcd.setCursor(0, 1);
			lcd.print("Set completed!");
			delay(1000);
		}
		else if (L) {
#if Gyro_Mode
			Gryo_Start();
#else
			HMC_Start();
#endif 
		}
		else if (R) {
			UI_status = 3;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 3:
		lcd.home();
		lcd.print("LINE_Val:");
		lcd.print(LINE_NOW);
		lcd.setCursor(0, 1);
		lcd.print("L:up D:down R:next");
		if (L&&D) {
			LINE_Set(140);
		}
		else if (L) {
			LINE_Set(LINE_NOW + 5);
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 4;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			LINE_Set(LINE_NOW - 5);
			delay(UI_Delay);
		}
		break;
		/*static uint8_t count_UI;
		lcd.home();
		lcd.print("LINE_Val_set:");
		lcd.setCursor(0, 1);
		lcd.print("L:re D:set R:next");
		if (D) {
			Wire.beginTransmission(11);
			Wire.write(0);
			Wire.endTransmission();
			Melody(2);
			lcd.setCursor(13, 0);
			lcd.print(count_UI + 1);
			count_UI++;
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 4;
			count_UI = 0;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (L) {
			count_UI = 0;
			Wire.beginTransmission(11);
			Wire.write(1);
			Wire.endTransmission();
			delay(UI_Delay);
			Melody(1);
		}
		break;*/
	case 4:
		lcd.home();
		lcd.print("HC_Val:       ");
		lcd.setCursor(0, 1);
		lcd.print("                ");
		lcd.setCursor(0, 1);
		lcd.print(HC_Get(HC_F));
		lcd.setCursor(4, 1);
		lcd.print(HC_Get(HC_B));
		lcd.setCursor(8, 1);
		lcd.print(HC_Get(HC_L));
		lcd.setCursor(12, 1);
		lcd.print(HC_Get(HC_R));
		delay(100);
		if (R || D || L) {
			UI_status = 5;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	case 5:
		lcd.home();
		lcd.print("Moter_P:       ");
		lcd.setCursor(9, 0);
		lcd.print(M_P);
		lcd.setCursor(0, 1);
		lcd.print("L:up D:down R:next");
		if (L&&D) {
			M_P = 255;
		}
		else if (L) {
			M_P += 5;
			delay(UI_Delay);
		}
		else if (R) {
			UI_status = 0;
			lcd.clear();
			delay(UI_Delay);
		}
		else if (D) {
			M_P -= 5;
			delay(UI_Delay);
		}
		EEPROM.write(1, M_P);
		break;
	default:
		lcd.clear();
		lcd.print("ERRER");
		lcd.setCursor(0, 1);
		lcd.print("AUTO REPAIR");
		delay(1000);
		UI_status = 0;
		break;
	}

}


/*�������֐�*/
inline void lcd_Start(char* ver) {

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

inline void LED_Check() {
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

#if Gyro_Mode
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
	mpu.setXGyroOffset(-1571);
	mpu.setYGyroOffset(-436);
	mpu.setZGyroOffset(-130);
	mpu.setZAccelOffset(1458);
	/*mpu.setXGyroOffset(-27);
	mpu.setYGyroOffset(6);
	mpu.setZGyroOffset(5);
	mpu.setZAccelOffset(2805);*/
	mpu.setDMPEnabled(true);
	mpuIntStatus = mpu.getIntStatus();
	dmpReady = true;
	packetSize = mpu.dmpGetFIFOPacketSize();
}
#else
void HMC_Start() {
	HMC.begin();
	HMC.setRange(HMC5883L_RANGE_1_3GA);	// Set measurement range
	HMC.setMeasurementMode(HMC5883L_CONTINOUS);	// Set measurement mode
	HMC.setDataRate(HMC5883L_DATARATE_75HZ);	// Set data rate
	HMC.setSamples(HMC5883L_SAMPLES_8);	// Set number of samples averaged
	HMC.setOffset(101, -127);	// Set calibration offset. See HMC5883L_calibration.ino}
#endif

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
	EEPROM.write(4, buf[0]);
	EEPROM.write(5, buf[1]);
}
