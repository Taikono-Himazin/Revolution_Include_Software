#include <Servo.h>

Servo myservo1;
void setup() {
  Serial.begin(9600);
  myservo1.attach(6,1,2);
myservo1.write(0);//esc初期化 出力ピンはPWM対応ピンのみ
/*  myservo2.attach(11,1,2);
myservo2.write(0);//esc初期化 出力ピンはPWM対応ピンのみ
*/
delay(1000);
}

void loop() {
  for(int i=0;i<200;i++){
  myservo1.write(i);//最低107あたり、最高は180
  Serial.println(i);
  delay(200);
  }
}

