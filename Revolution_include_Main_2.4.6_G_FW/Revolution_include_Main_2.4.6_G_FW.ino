//////////////////////////////////
///RCJ  Revolution_Include 2017///
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
/*ここまで*/

/*define*/
#define Gyro_Mode false //ジャイロモードtrueならジャイロ、falseならコンパス
#define M_sw 0
#define SPEAKER A0
#define BEEP 100
#define Old_Persent 0.15 //1以下！ Moterの過去の値の割合 
#define L_sw 4
#define D_sw 7
#define R_sw 10
#define LED_L A3
#define LED_M A2
#define LED_R A1
#define Check_Delay 100
#define IR_offset 0
#define IR_Cut 200
#define UI_Delay 200
#define HC_F 11//前
#define HC_B 12//後ろ
#define HC_L 13//左
#define HC_R 2//右
#if Gyro_Mode
#define Gyro_X -1582
#define Gyro_Y -644
#define Gyro_Z -132
#define Accel_Z 1432
#else
#define HMC_X 86
#define HMC_Y 261
#endif

#define LED(a) digitalWrite(a, HIGH)
#define LEDoff(a) digitalWrite(a, LOW)
/*ここまで*/

/*川野さんからコピペ関数宣言*/
static double Setpoint, Input, Output;
static const double Kp = 2.2, Ki = 0, Kd = 0.01;
MPU6050 mpu;
HMC5883L HMC;
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
#if Gyro_Mode
int16_t  Gyro_Now = 0, Gyro = 0, Gyro_Offset = 0, Gyro_Offset_Difo = 0;
uint16_t fifoCount;
uint8_t fifoBuffer[64]; // FIFO storage buffer							   // orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
#else
int16_t HMC_Now = 0, HMC_val = 0, HMC_Offset = 0, HMC_Offset_Difo = 0;
#endif

int16_t old_Moter_D = 0, Gyro_Old = 0, Gyro_Old_ms = 0;
uint8_t LINE_Status = 0, UI_status = 0, Errer_Flag_Status = 0, Back_count = 0, Reset_Count = 0;
uint16_t IR_F = 0, IR_D = 0, old_Moter_F = 0, M_P, LINE_NOW;
uint32_t F, B, L, R;

bool change1 = true, change2 = false, Errer_Flag = false, LINE = false, LINE_F, LINE_R, LINE_B, LINE_L;
/*ここまで*/

//プロトタイプ宣言(不必要のため途中からコメントアウト　エラー起こしたらしてみて
extern void setup();
extern void loop();
extern void moter(uint8_t Force, int16_t Degree, bool PID=true);
extern void GyroGet();
extern void HMC_Get();
extern void sleep();
extern inline void IR_Get();
extern inline void Motion_System(uint8_t Force, int16_t Degree);
extern void Spin(bool D = true);
extern void Melody(uint8_t mode);
extern inline void LINE_Get();
extern inline void LINE_Move();
extern void UI();
extern inline void lcd_Start(char* ver);
extern inline void LED_Check();
extern void Servo_Start();
extern void Gryo_Start();
extern void HMC_Start();
extern void PID_Start();
extern void LINE_Set(uint16_t val);
extern uint16_t HC_Get(uint8_t pin);
extern void Spin(bool D = true);
extern void Spin_F(bool D = true);

/*ここまで*/

/*--プログラム--*/
void setup() {
  //Serial.begin(115200);
  Wire.begin();
  i2c_faster();
  PID_Start();
  M_P = EEPROM.read(1); // M_P閾値読み込み

  uint16_t val = EEPROM.read(4) << 8 | EEPROM.read(5);
  LINE_Set(val);
  Servo_Start();
  lcd_Start("2.4.6_G_FW");//lcd初期化関数

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
  if (digitalRead(M_sw) == LOW && !Errer_Flag) {
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
#if Gyro_Mode
        Gyro_Offset = Gyro_Offset_Difo;
#else
        HMC_Offset = HMC_Offset_Difo;
#endif
      }
      sleep();
      UI();
    }
  }
}

/*--自作関数--*/
void moter(uint8_t Force, int16_t Degree, bool PID) { //一応解読したがいじれるほどはわからん。とりあえず同じ形ならそのままいこう

  static int16_t Gyro_Old;

  int16_t m1, m2;
  old_Moter_D = old_Moter_D * Old_Persent + Degree * (1 - Old_Persent);
  old_Moter_F = old_Moter_F * Old_Persent + Force * (1 - Old_Persent);

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
#if Gyro_Mode
  int16_t hosei;
#endif
  if (PID) {
#if Gyro_Mode
    GyroGet();
    hosei = (180 - Gyro) * 2;
    if (abs(hosei) < 5) {
      hosei = 0;
    }
#else
    HMC_Get();
    Input = HMC_val;
#endif
    myPID.Compute(); //pid計算
  }
#if  Gyro_Mode
  m1 = m1 - hosei;//pidのデータをモーターに突っ込む
  m2 = m2 - hosei;
  m3 = m3 - hosei;
  m4 = m4 - hosei;
#else
  m1 = m1 - Output;//pidのデータをモーターに突っ込む
  m2 = m2 - Output;
  m3 = m3 - Output;
  m4 = m4 - Output;
#endif
  m1 = constrain(m1, -255, 255);
  m2 = constrain(m2, -255, 255);
  m3 = constrain(m3, -255, 255);
  m4 = constrain(m4, -255, 255);

  m2=-m2;

  /*lcd.home();
    lcd.print(m1);
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
#if Gyro_Mode
  if ((Gyro <= 140 || Gyro >= 220) && Errer_Flag_Status == 0) {
    uint16_t i = adjustMillis();
    if ((i - Gyro_Old_ms < 400) && i - Gyro_Old_ms > 100 && ((180 - Gyro_Old > 0) != (180 - Gyro > 0))) {
      Errer_Flag = true;
      LED(LED_R);
    }
    Gyro_Old_ms = i;
    Gyro_Old = Gyro;
  }

  if ((Gyro > 300 || Gyro < 60) && Errer_Flag_Status == 0) {
    if (Reset_Count >= 100) {
      Errer_Flag = true;
      LED(LED_R);
    }
    Reset_Count++;
  }
  else {
    Reset_Count = 0;
  }

  if (Errer_Flag_Status != 0) {
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
  }
  IR_D = IR_D + IR_offset;
  if (IR_D < 0) {
    IR_D = IR_D + 360;
  }
  else if (IR_D > 360) {
    IR_D = IR_D - 360;
  }
  
  if (IR_F > IR_Cut) {
    IR_F = IR_F - IR_Cut;
  } else {
    IR_F = 0;
  }
  /*
    Serial.print(IR_F);
    Serial.println(IR_D);*/
}

inline void Motion_System(uint8_t Force, int16_t Degree) { //挙動制御 Force=IR_F Degree=IR_D
  int16_t M_Degree = 0, Dri1_Power = 0, Dri2_Power = 0;
  uint8_t	M_Force = M_P;
  static uint16_t Ball_Count = 0, Count_Reset;
  bool Ball2 = analogRead(A6) >= 900;//後ろ
  bool Ball1 = false;
  if (Ball2 || Ball1) {
    LED(LED_M);
  }
  else {
    LEDoff(LED_M);
  }

#define Servo1_idel Dri2_Power=0;Dri1_Power=90
#define Servo2_idel Dri1_Power=0;Dri2_Power=50
#define Servo1_Dri Dri1_Power=180;Dri2_Power=0
#define Servo2_Dri Dri2_Power=180;Dri1_Power=0

  if (Force != 0) {  // Ball Found      //ここから挙動制御
    Back_count = 0;
	if ((265 <= Degree) && (Degree < 275)) {                 //後ろ
		Servo2_Dri;
		M_Degree = 274;
		M_Force = M_P - 80;
	}
	else if ((275 <= Degree) && (Degree < 300)) {            //i
      M_Degree = Degree + 20;
      M_Force = M_P - 50;
      Servo2_Dri;
    } 
	else if ((300 <= Degree) && (Degree < 320)) {             //h
      M_Degree = Degree + 40;
      M_Force = M_P - 50;
      Servo2_idel;
    } 
	else if ((320 <= Degree) && (Degree < 340)) {             //g
      M_Degree = Degree + 50;
    } 
	else if (340 <= Degree) {                                 //f
      M_Degree = Degree - 40;
    } 
	else if ((0 <= Degree) && (Degree < 25)) {               //e
      M_Degree = Degree - 30;
    } 
	else if ((25 <= Degree) && (Degree < 50)) {            //d
      M_Degree = Degree - 50;
    }
	else if ((50 <= Degree) && (Degree < 60)) {              //c
      M_Degree = Degree - 45;
    }
	else if ((60 <= Degree) && (Degree < 70)) {              //b
      M_Degree = Degree - 40;
      Servo1_idel;
    }
	else if ((70 <= Degree) && (Degree < 80)) {              //a
      M_Degree = Degree - 32;
      Servo1_Dri;
    }
	else if ((80 <= Degree) && (Degree < 100)) {               //真ん中
      M_Degree = 85;
      Servo1_Dri;
    }
	else if ((100 <= Degree) && (Degree < 110)) {              //a
      M_Degree = Degree + 32;
      Servo1_Dri;
    }
	else if ((110 <= Degree) && (Degree < 120)) {              //b
      M_Degree = Degree + 40;
      Servo1_idel;
    }
	else if ((120 <= Degree) && (Degree < 130)) {              //c
      M_Degree = Degree + 45;
    }
	else if ((130 <= Degree) && (Degree < 155)) {              //d
      M_Degree = Degree + 50;
    }
	else if ((155 <= Degree) && (Degree < 180)) {               //e
      M_Degree = Degree + 30;
    }
	else if ((180 <= Degree) && (Degree < 200)) {               //f
      M_Degree = Degree + 40;
    }
	else if ((200 <= Degree) && (Degree < 220)) {             //g
      M_Degree = Degree - 50;
    }
	else if ((220 <= Degree) && (Degree < 240)) {             //h
      M_Degree = Degree - 40;
      M_Force = M_P - 50;
      Servo2_idel;
    }
	else if ((240 <= Degree) && (Degree < 265)) {             //i
      M_Degree = Degree - 20;
      M_Force = M_P - 50;
      Servo2_Dri;
    }
    
    if (M_Degree < 0) {
      M_Degree = 360 + M_Degree;
    }

    //ボールカウント
    if ((Degree > 260 && Degree < 280) && Ball2) {
      Ball_Count = 0;
      while (bitRead(LINE_Status, 4) == 0 && Ball_Count < 100 && Ball2&&digitalRead(M_sw)==LOW)
      {
        Ball2 = analogRead(A6) >= 900;
        LINE_Get();
        Ball_Count++;
        moter(0, 0);
      }
    }
  }
  
  else {                                        //no ball
    Dri1_Power = 0;
    Dri2_Power = 0;
    Ball_Count = 0;
#if Gyro_Mode
    if (Gyro > 170 && Gyro < 190) {
#endif
    B = HC_Get(HC_B), L = HC_Get(HC_L), R = HC_Get(HC_R);
    if( L > 90 ){
      M_Degree = 180;
      M_Force = 120;
     } 
	else if ( R > 90 ){
      M_Degree = 0;
      M_Force = 120;
     } 
	else if ( B > 25 ){
		 M_Degree = 270;
      M_Force = 120;
     } 
	else {
      M_Degree = 90;
      M_Force = 0;
     }

     //M_Degree = 90;//とまる gohomeの時無効化しなければならない
     //M_Force = 0;

#if Gyro_Mode
    }
#endif
  }

  LINE_Get();
  if (bitRead(LINE_Status, 4) == 1) {

    LINE = true;
  }
  else {
    LINE = false;
  }

  if (Ball_Count >= 30 && !LINE) {
    uint16_t i;
    if (Ball2 == true) {
      i = HC_Get(HC_L);
      if (i != 0 && i < 70) {
        Spin(false);
      }
      else {
        Spin();
      }
      Ball_Count = 0;
      Errer_Flag_Status = 1000;
    }
  }
  else if (LINE) {
	  LINE_Move();
  }
  moter(M_Force, M_Degree);
  myServo1.write(Dri1_Power);
  myServo2.write(Dri2_Power);
}

inline void LINE_Move() {
	uint8_t M_Force = 0;
	uint16_t M_Degree = 0;
	int16_t LINE_count = 0;
	bool OK_Flag = false, First_Flag = true;
	while (digitalRead(M_sw) == LOW) {
		if (First_Flag) {
			if (bitRead(LINE_Status, 0) == 1) {//右
				if (bitRead(LINE_Status, 1) == 1) {//右かつ後ろ
					M_Degree = 135;
					M_Force = 255;
				}
				else if (bitRead(LINE_Status, 3) == 1) { //右かつ前
					M_Degree = 215;
					M_Force = 255;
				}
				else {//右のみ
					M_Degree = 180;
					M_Force = 255;
				}
			}
			else if (bitRead(LINE_Status, 1) == 1) { //後ろ
				if (bitRead(LINE_Status, 2) == 1) { //後ろかつ左
					M_Degree = 45;
					M_Force = 255;
				}
				else {//後ろのみ
					M_Degree = 90;
					M_Force = 255;
				}
			}
			else if (bitRead(LINE_Status, 2) == 1) {//左
				if (bitRead(LINE_Status, 3) == 1) { //左かつ前
					M_Degree = 315;
					M_Force = 255;
				}
				else { //左のみ
					M_Degree = 0;
					M_Force = 255;
				}
			}
			else if (bitRead(LINE_Status, 3) == 1) {//前のみ
				M_Degree = 270;
				M_Force = 255;
			}
			First_Flag = false;
			LINE_count = 50;
		}
		else if (LINE_count <= 0) {
			/*uint16_t i = HC_Get(HC_F);
			LINE_F = i < 40 && i != 0;
			i = HC_Get(HC_B);
			LINE_B = i < 40 && i != 0;
			i = HC_Get(HC_L);
			LINE_L = i < 40 && i != 0;
			i = HC_Get(HC_R);
			LINE_R = i < 40 && i != 0;
			M_Force = 255;
			if (LINE_B) {
			M_Degree = 90;
			}
			if (LINE_L) {
			M_Degree = 0;
			}
			if (LINE_R) {
			M_Degree = 180;
			}
			if (LINE_F) {
			M_Degree = 280;
			}
			if (LINE_B&&LINE_L) {
			M_Degree = 45;
			}
			if (LINE_B&&LINE_R) {
			M_Degree = 135;
			}
			if (LINE_F&&LINE_L) {
			M_Degree = 315;
			}
			if (LINE_F&&LINE_R) {
			M_Degree = 225;
			}
			if (!LINE_F && !LINE_B && !LINE_R && !LINE_L) {
			LINE_Get();
			if (bitRead(LINE_Status, 4) == 0) {
			if (OK_Flag) {
			break;
			}
			else {
			OK_Flag = true;
			}
			}
			else {
			First_Flag = true;
			}*/
			LINE_Get();
			if (bitRead(LINE_Status, 0) == 1) {//右
				if (bitRead(LINE_Status, 1) == 1) {//右かつ後ろ
					M_Degree = 135;
					M_Force = 255;
				}
				else if (bitRead(LINE_Status, 3) == 1) { //右かつ前
					M_Degree = 215;
					M_Force = 255;
				}
				else {//右のみ
					M_Degree = 180;
					M_Force = 255;
				}
			}
			else if (bitRead(LINE_Status, 1) == 1) { //後ろ
				if (bitRead(LINE_Status, 2) == 1) { //後ろかつ左
					M_Degree = 45;
					M_Force = 255;
				}
				else {//後ろのみ
					M_Degree = 90;
					M_Force = 255;
				}
			}
			else if (bitRead(LINE_Status, 2) == 1) {//左
				if (bitRead(LINE_Status, 3) == 1) { //左かつ前
					M_Degree = 315;
					M_Force = 255;
				}
				else { //左のみ
					M_Degree = 0;
					M_Force = 255;
				}
			}
			else if (bitRead(LINE_Status, 3) == 1) {//前のみ
				M_Degree = 270;
				M_Force = 255;
			}
			if (bitRead(LINE_Status, 4) == 0) {
				if (OK_Flag) {
					break;
				}
				else {
					OK_Flag = true;
				}
			}
			else {
				OK_Flag = false;
			}
			LINE_count = 30;
		}
		LINE_count--;
		moter(M_Force, M_Degree);
	}
}

void Spin(bool D = true) {
  myServo2.write(180);
  int16_t m1, m2, m3, m4;
  int16_t i[2];
  if (D) {
    i[0] = 60;    //初動パワー
    i[1] = 255;   //本回転パワー
  }
  else {
    i[0] = -60;
    i[1] = -255;
  }
  uint8_t Spin_Move, Spin_Force;
  if (D) {
    Spin_Move = 330, Spin_Force = 40;//Moveが角度　Forceが速さ
  }
  else {
    Spin_Move = 210, Spin_Force = 40;
  }
  int16_t m1_D = Spin_Move - 45;
  if (m1_D < 0) m1_D = m1_D + 360;
  else if (m1_D > 359) m1_D = m1_D - 360;

  m1 = sin((float)m1_D * 0.01745329) * Spin_Force; // sin でもcosじゃないと理解不能

  int16_t m2_D = Spin_Move - 315;
  if (m2_D < 0) m2_D = m2_D + 360;
  else if (m2_D > 359) m2_D = m2_D - 360;

  m2 = sin((float)m2_D * 0.01745329) * Spin_Force;

  int16_t F_max = abs(m1);
  if (F_max < abs(m2)) F_max = abs(m2);

  float k = (float)Spin_Force / F_max;//各モーターの比を保ちながら最大値を225に
  m1 = m1 * k;
  m2 = m2 * k;

  m3 = -m1;//反対に位置しているから反転
  m4 = -m2;

  m1 += -10;
  m2 += i[0];
  m3 += i[0];
  m4 += i[0];
  m1 = constrain(m1, -255, 255);
  m2 = constrain(m2, -255, 255);
  m3 = constrain(m3, -255, 255);
  m4 = constrain(m4, -255, 255);

  m2=-m2;

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

  uint16_t j = 0;
  while (j < 2500&& digitalRead(M_sw) == LOW)                             //初動の待機
  {
    LINE_Get();
    if (bitRead(LINE_Status, 4) == 1) {
      return;
    }
    j++;
  }

  myServo2.write(180);

  m1 = i[1];
  m2 = i[1];
  m3 = i[1];
  m4 = i[1];

  m2=-m2;

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

  j = 0;
  while (j < 1000&& digitalRead(M_sw) == LOW)                        //本回転待機
  {
    LINE_Get();
    if (bitRead(LINE_Status, 4) == 1) {
      return;
    }
    j++;
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
}

uint16_t HC_Get(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  Wire.requestFrom(90, 2);
  uint16_t val;
  while (Wire.available()) {
    val = (Wire.read() << 8) | Wire.read();
  }
  digitalWrite(pin, LOW);
  return val;
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
      lcd.print(Gyro);
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
        Gyro_Offset_Difo = Gyro_Offset;
#else
        HMC_Offset = 180 - HMC_Now;
        HMC_Offset_Difo = HMC_Offset;
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
      if (L && D) {
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
      if (L && D) {
        M_P = 255;
      }
      else if (L) {
        M_P += 5;
        delay(UI_Delay);
      }
      else if (R) {
        UI_status = 6;
        lcd.clear();
        delay(UI_Delay);
      }
      else if (D) {
        M_P -= 5;
        delay(UI_Delay);
      }
      EEPROM.write(1, M_P);
      break;
    case 6:
      lcd.home();
      lcd.print("Spin");
      lcd.setCursor(0, 1);
      lcd.print("L:Spin D:Spin2 R:next");
      if (R) {
        UI_status = 7;
        lcd.clear();
        delay(UI_Delay);
      }
      else if (D) {
        delay(1000);
        myServo2.write(180);
        delay(100);
        Spin();
        delay(UI_Delay);
      }
      else if (L) {
        delay(1000);
        myServo2.write(180);
        delay(100);
        Spin(false);
        delay(UI_Delay);
      }
      myServo2.write(0);
      sleep();
      break;
    case 7:
      lcd.home();
      lcd.print("Ball Sensor");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcd.setCursor(0, 1);
      lcd.print(analogRead(A7));
      lcd.setCursor(6, 1);
      lcd.print(analogRead(A6));
      if (R) {
        UI_status = 0;
        lcd.clear();
        delay(UI_Delay);
      }
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


/*初期化関数*/
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
  mpu.setXGyroOffset(Gyro_X);
  mpu.setYGyroOffset(Gyro_Y);
  mpu.setZGyroOffset(Gyro_Z);
  mpu.setZAccelOffset(Accel_Z);
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
  HMC.setOffset(HMC_X, HMC_Y);	// Set calibration offset. See HMC5883L_calibration.ino
}
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
