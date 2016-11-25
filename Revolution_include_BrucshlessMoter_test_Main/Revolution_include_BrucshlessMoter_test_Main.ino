#include <Servo.h>

#define BrucshlessMoter1 6//出力ピンはPWM対応ピンのみ
#define BrucshlessMoter2 3//出力ピンはPWM対応ピンのみ

Servo myservo1;
Servo myservo2;
void setup() {
  Serial.begin(9600);
  myservo1.attach(BrucshlessMoter1, 1, 2);
  myservo2.attach(BrucshlessMoter2, 1, 2);
  myservo1.write(0);
  myservo2.write(0);
  delay(1000);
}

void loop() {
/*  myservo1.write(180);
  myservo2,write(180);*/
  
    for(int i=0;i<200;i++){
    myservo1.write(i);//最低107あたり、最高は180
    myservo2.write(i);
    Serial.println(i);
    delay(200);
    if(i==179){
    delay(5000);
    }
    }
}

