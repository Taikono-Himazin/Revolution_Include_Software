//////////////////////////////////
///RCJ  Revolution_Include 2017///
///      Main　永山和樹        ///
//////////////////////////////////
/*プログラムモード*/
#define PIDmode false


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
#define Old_Persent 0.75
#define L_sw 4 
#define D_sw 7
#define R_sw 10
#define LED_L A3
#define LED_M A2
#define LED_R A1
#define Check_Delay 100
#define UI_Delay 200
/*ここまで*/

/*川野さんからコピペ関数宣言*/
static double Setpoint, Input, Output;
static const double Kp = 2.8, Ki = 0, Kd = 0.01;
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
uint8_t LINE_Status = 0, UI_status = 0;
uint16_t IR_F = 0, IR_D = 0, old_M_F = 0, old_M_D = 0, Gyro_Now = 0, Gyro = 0, Gyro_Offset = 0;
boolean change1 = true, change2 = false;
/*ここまで*/

/*プロトタイプ宣言(不必要のため途中からコメントアウト　エラー起こしたらしてみて
void moter(uint8_t Force, uint16_t Degree);
void sleep();
void IR_Get();
void GyroGet();
void Motion_System(uint8_t Force, uint16_t Degree);
void Melody(uint8_t mode);
void LINE_Get();
void lcd_Start(char* ver);
void UI();
void LED_Check();
ここまで*/

/*--プログラム--*/
void setup() {
  // Serial.begin(115200);
  Wire.begin();
  i2c_faster();
  /* myPID.SetOutputLimits(-255, 255);
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(15);
  */
  lcd_Start("1.1.7 LED");//lcd初期化関数
  myServo1.attach(6, 1, 2);
  myServo1.write(0);//esc初期化 出力ピンはPWM対応ピンのみ
  myServo2.attach(3, 1, 2);
  myServo2.write(0);
  while (digitalRead(M_sw) == LOW) {
    Melody(1);
    delay(1000);
    lcd.setCursor(0,1);
    lcd.print("Please turn off");
  }
  LED_Check();
  lcd.setCursor(0, 1);
  lcd.print("               ");
    lcd.setCursor(0,1);
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
		/*
		GyroGet();
		Motion_System(IR_F, IR_D);
		*/
		moter(IR_F, IR_D);
	}else {
		if (change2) {
			lcd.backlight();
			change1 = true;
			change2 = false;
		}
		sleep();
		UI();
	}
}

/*--自作関数--*/
void moter(uint8_t Force, uint16_t Degree) { //一応解読したがいじれるほどはわからん。とりあえず同じ形ならそのままいこう
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
  int16_t m1 = sin((float)m1_Degree * 0.01745329) * Force; // sin でもcosじゃないと理解不能
  int16_t m2 = sin((float)m2_Degree * 0.01745329) * Force;
  //  int m3 = sin((float)m3_Degree * 0.01745329) * Force;
  //  int m4 = sin((float)m4_Degree * 0.01745329) * Force;

  int16_t Force_max = abs(m1);
  if (Force_max < abs(m2)) Force_max = abs(m2);
  //  if ( Force_max < abs(m3) ) Force_max = abs(m3);
  //  if ( Force_max < abs(m4) ) Force_max = abs(m4);

  float k = (float)Force / Force_max;//各モーターの比を保ちながら最大値を225に
  m1 = m1 * k;
  m2 = m2 * k;
  //  m3 = m3 * k;
  //  m4 = m4 * k;

  int16_t m3 = -m1;//反対に位置しているから反転
  int16_t m4 = -m2;
/*
  GyroGet();//ジャイロのデータを取得
    Input = //ジャイロのデータをInput関数に突っ込む
    myPID.Compute(); //pid計算
    m1 = m1 - Output;//pidのデータをモーターに突っ込む
    m2 = m2 - Output;
    m3 = m3 - Output;
    m4 = m4 - Output;
    m1 = constrain(m1, -255, 255);
    m2 = constrain(m2, -255, 255);
    m3 = constrain(m3, -255, 255);
    m4 = constrain(m4, -255, 255);
	*/
  uint8_t buf[5];//送信
  bitSet(buf[4], 4); //モータの電源on
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

inline void IR_Get() {
  Wire.requestFrom(9, 4);
  while (Wire.available()) {
    IR_F = (Wire.read() << 8) | Wire.read(); // Force Read
    IR_D = (Wire.read() << 8) | Wire.read(); // Degree Read
  }/*
	 Serial.print(IR_F);
	 Serial.println(IR_D);*/
}

static inline void GyroGet()
{
  static uint16_t fifoCount;
  static uint8_t fifoBuffer[64]; // FIFO storage buffer
  // orientation/motion vars
  static Quaternion q;           // [w, x, y, z]         quaternion container
  static VectorFloat gravity;    // [x, y, z]            gravity vector
  static float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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
    //    if (Gyro < 0)   Gyro += 360;
    if (Gyro > 359) Gyro -= 360;
  }
}

inline void Motion_System(uint8_t Force, uint16_t Degree) { //挙動制御 Force=IR_F Degree=IR_D
  int16_t Dir = 0;
  if (Force != 0) {  // Ball Found                                           //ここから挙動制御
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
    else if ((80 <= Degree) && (Degree <  100)) {               //真ん中
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
		moter(0, Degree);
    }
    if (Dir < 0) {
      Dir = 360 + Dir;
    }
    //  Serial.println(Dir);
    moter(255, Dir);
  }
}

void Melody(uint8_t mode) {
  if (mode==0) {   // StartMelody
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
	  digitalWrite(LED_L,HIGH);
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
		lcd.home();
		lcd.print("Revolution ");
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
		lcd.print("Main Menw");
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
			LED(LED_L);
			delay(1000);
			LED(LED_M);
			delay(1000);
			LED(LED_R);
			delay(1000);
			}
			else if (D) {
				LEDoff(LED_L);
				LEDoff(LED_M);
				LEDoff(LED_R);
				UI_status = 1;
				lcd.clear();
				delay(UI_Delay);
			}
			else {
				LEDoff(LED_L);
				LEDoff(LED_M);
				LEDoff(LED_R);
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
	lcd.print("Helo, Revolution");
	delay(1000);
	lcd.setCursor(0, 1);        // go to the next line
	lcd.print("Main ver");
	lcd.print(ver);
	delay(1000);
	lcd.clear();
	delay(500);
	lcd.print("Chaking");
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