#define sw1 4
#define sw2 7
#define sw3 10

#define M_sw 0

#define led1 A3
#define led2 A2
#define led3 A1

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(sw1,INPUT);
pinMode(sw2,INPUT);
pinMode(sw3,INPUT);
pinMode(led1,OUTPUT);
pinMode(led2,OUTPUT);
pinMode(led3,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(M_sw)==LOW){
digitalWrite(led1,digitalRead(sw1));
digitalWrite(led2,digitalRead(sw2));
digitalWrite(led3,digitalRead(sw3));
  }
}
