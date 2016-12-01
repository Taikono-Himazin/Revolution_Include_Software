//////////////////////////////////
///RCJ  Revolution_Include 2017///
///      Main�@�i�R�a��        ///
//////////////////////////////////


/*Include*/
#include <Servo.h>
#include <Utility.h>
#include <Wire.h>
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <LiquidCrystal_I2C.h>
/*�����܂�*/

/*define*/
#define M_sw 0
#define SPEAKER 13
#define BEEP 100
#define Old_Persent 0.75
#define L_sw 4 
#define D_sw 7
#define R_sw 10
#define LED_L A3
#define LED_M A2
#define LED_R A1
#define Check_Delay 100
#define UI_Delay 200
/*�����܂�*/

/*��삳�񂩂�R�s�y�֐��錾*/
static double Setpoint=0, Input, Output;
static const double Kp = 2, Ki = 0, Kd = 0.001;
MPU6050 mpu;
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
int16_t ax, ay, az, Gyro_Now = 0, Gyro = 0, Gyro_Offset = 0;
uint8_t LINE_Status = 0, UI_status = 0;
uint16_t IR_F = 0, IR_D = 0, old_M_F = 0, old_M_D = 0;

boolean change1 = true, change2 = false;
/*�����܂�*/

//�v���g�^�C�v�錾(�s�K�v�̂��ߓr������R�����g�A�E�g�@�G���[�N�������炵�Ă݂�
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
/*�����܂�*/

/*--�v���O����--*/
void setup() {
	// Serial.begin(115200);
	Wire.begin();
	i2c_faster();
	mpu.initialize();
	myPID.SetOutputLimits(-255, 255);
	myPID.SetMode(AUTOMATIC);
	myPID.SetSampleTime(15);
	Setpoint = 180;
	lcd_Start("1.2.0 PID");//lcd�������֐�
	if (mpu.testConnection() != true) {
		lcd.setCursor(0, 1);
		lcd.print("MPU disconection");
		while (true){}
	}
	if (mpu.dmpInitialize() != 0) {
		lcd.setCursor(0, 1);
		lcd.print("MPU break");
		while (true){}
	}
	mpu.setXGyroOffset(106);
	mpu.setYGyroOffset(-7);
	mpu.setZGyroOffset(-3);
	mpu.setZAccelOffset(1108);
	mpu.setDMPEnabled(true);
	mpuIntStatus = mpu.getIntStatus();
	dmpReady = true;
	packetSize = mpu.dmpGetFIFOPacketSize();

	myServo1.attach(6, 1, 2);
	myServo1.write(0);//esc������ �o�̓s����PWM�Ή��s���̂�
	myServo2.attach(3, 1, 2);
	myServo2.write(0);
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
		//Motion_System(IR_F, IR_D);
		moter(IR_F, IR_D);
	}
	else {
		if (change2) {
			lcd.backlight();
			change1 = true;
			change2 = false;
		}
		lcd.clear();
		lcd.home();
		lcd.print("Gyro");
		lcd.setCursor(0, 1);
		GyroGet();
		lcd.print(ay);
		sleep();
		UI();
		delay(200);
	}
}

/*--����֐�--*/
void moter(uint8_t Force, uint16_t Degree) { //�ꉞ��ǂ������������قǂ͂킩���B�Ƃ肠���������`�Ȃ炻�̂܂܂�����
	int16_t m1, m2;
	old_M_D = old_M_D*Old_Persent + Degree*(1- Old_Persent);
	old_M_F = old_M_F*0.9 + Force*0.1;
  int16_t m1_Degree = old_M_D - 45;
  if (m1_Degree < 0) m1_Degree = m1_Degree + 360;
  else if (m1_Degree > 359) m1_Degree = m1_Degree - 360;

  int16_t m2_Degree = old_M_D - 315;
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
   m1 = sin((float)m1_Degree * 0.01745329) * Force; // sin �ł�cos����Ȃ��Ɨ���s�\
   m2 = sin((float)m2_Degree * 0.01745329) * Force;
  //  int m3 = sin((float)m3_Degree * 0.01745329) * Force;
  //  int m4 = sin((float)m4_Degree * 0.01745329) * Force;

 int16_t Force_max = abs(m1);
  if (Force_max < abs(m2)) Force_max = abs(m2);
  //  if ( Force_max < abs(m3) ) Force_max = abs(m3);
  //  if ( Force_max < abs(m4) ) Force_max = abs(m4);

  float k = (float)Force / Force_max;//�e���[�^�[�̔��ۂ��Ȃ���ő�l��225��
  m1 = m1 * k;
  m2 = m2 * k;
  //  m3 = m3 * k;
  //  m4 = m4 * k;

  int16_t m3 = -m1;//���΂Ɉʒu���Ă��邩�甽�]
  int16_t m4 = -m2;

  GyroGet();//�W���C���̃f�[�^���擾
  Input = ay; //�W���C���̃f�[�^��Input�֐��ɓ˂�����
    myPID.Compute(); //pid�v�Z
    m1 = m1 - Output;//pid�̃f�[�^�����[�^�[�ɓ˂�����
    m2 = m2 - Output;
    m3 = m3 - Output;
    m4 = m4 - Output;
    m1 = constrain(m1, -255, 255);
    m2 = constrain(m2, -255, 255);
    m3 = constrain(m3, -255, 255);
    m4 = constrain(m4, -255, 255);
	
  uint8_t buf[5];//���M
  bitSet(buf[4], 4); //���[�^�̓d��on
  if (m1 < 0) bitSet(buf[4], 0);
  else bitClear(buf[4], 0);
  buf[0] = abs(m1);
  if (m2 < 0) bitSet(buf[4], 1);
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
  /*
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
    data->MotorPWM[2] = m3;     //data�Ƃ����^�̒���MoterPWM[]�Ƃ����ϐ�������炵���B�悭������Ȃ��B�l�b�g�����̍ۂ�typedef��
    data->MotorPWM[3] = m4;
  */
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
	}else if (mpuIntStatus & 0x02) {
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		Gyro_Now = degrees(ypr[0]) + 180;
		ay = Gyro_Now + Gyro_Offset;
		//    if (ay < 0)   ay += 360;
		if (ay > 359) ay -= 360;
	}
}

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
	}/*
	 Serial.print(IR_F);
	 Serial.println(IR_D);*/
}

inline void Motion_System(uint8_t Force, uint16_t Degree) { //�������� Force=IR_F Degree=IR_D
	int16_t Dir = 0;
	uint8_t	For = 255;
	if (Force != 0) {  // Ball Found                                           //�������狓������
		if ((270 <= Degree) && (Degree < 290)) {                        //a
			Dir = Degree - 55;
		}
		else if ((290 <= Degree) && (Degree < 340)) {              //b
			Dir = Degree - 42;
		}
		else if ((340 <= Degree) && (Degree < 360)) {               //c
			Dir = Degree - 40;
		}
		else if ((0 <= Degree) && (Degree < 50)) {               //d
			Dir = Degree - 45;
		}
		else if ((50 <= Degree) && (Degree <  60)) {              //e
			Dir = Degree - 20;
		}
		else if ((60 <= Degree) && (Degree <  70)) {              //f
			Dir = Degree - 14;
		}
		else if ((70 <= Degree) && (Degree <  80)) {              //g
			Dir = Degree - 8;
		}
		else if ((80 <= Degree) && (Degree <  100)) {               //�^��
			Dir = 84;
		}
		else if ((100 <= Degree) && (Degree <  110)) {              //g
			Dir = Degree + 8;
		}
		else if ((110 <= Degree) && (Degree <  120)) {              //f
			Dir = Degree + 14;
		}
		else if ((120 <= Degree) && (Degree <  130)) {              //e
			Dir = Degree + 20;
		}
		else if ((130 <= Degree) && (Degree < 180)) {               //d
			Dir = Degree + 45;
		}
		else if ((180 <= Degree) && (Degree < 200)) {               //c
			Dir = Degree + 40;
		}
		else if ((200 <= Degree) && (Degree < 250)) {             //b
			Dir = Degree + 42;
		}
		else if ((250 <= Degree) && (Degree < 270)) {              //a
			Dir = Degree + 55;
		}
		else {
			Dir = 90;
			For = 0;
		}
		if (Dir < 0) {
			Dir = 360 + Dir;
		}
		//  Serial.println(Dir);
		moter(For, Dir);
	}
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
		if (bitRead(LINE_Status, 0) == 1) {//�E
			if (bitRead(LINE_Status, 1) == 1) {//�E�����
				moter(255, 135);
			}
			else if (bitRead(LINE_Status, 2) == 1) {//�E����
				return;
			}
			else if (bitRead(LINE_Status, 3) == 1) { //�E���O
				moter(255, 215);
			}
			else {//�E�̂�
				moter(255, 180);
			}
		}
		else if (bitRead(LINE_Status, 1) == 1) { //���
			if (bitRead(LINE_Status, 3) == 1) { //��납�O
				return;
			}
			else if (bitRead(LINE_Status, 2) == 1) { //��납��
				moter(255, 45);
			}
			else {//���̂�
				moter(255, 90);
			}
		}
		else if (bitRead(LINE_Status, 2) == 1) {//��
			if (bitRead(LINE_Status, 3) == 1) { //�����O
				moter(255, 315);
			}
			else { //���̂�
				moter(255, 0);
			}
		}
		else if (bitRead(LINE_Status, 3) == 1) {//�O�̂�
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

inline void LED(uint8_t pin) {
	digitalWrite(pin, HIGH);

}

inline void LEDoff(uint8_t pin) {
	digitalWrite(pin, LOW);
}

void UI() {
	bool L = digitalRead(L_sw) == HIGH;
	bool D = digitalRead(D_sw) == HIGH;
	bool R = digitalRead(R_sw) == HIGH;
	switch (UI_status)
	{
	case 0:
		/*lcd.home();
		lcd.print("Revolution ");
		lcd.setCursor(0, 1);
		lcd.print("         Include");*/

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
			UI_status = 2;
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
		lcd.print("L: D:exit R:Yes");
		if (R) {
			LED_Check();
		}
		else if (D) {
			UI_status = 1;
			lcd.clear();
			delay(UI_Delay);
		}
		break;
	default:
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