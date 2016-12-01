#include <Utility.h>
#include <Wire.h>

#define LINE1 1 //右
#define LINE2 0
#define LINE3 2
#define LINE4 3 //右
#define LINE5 4 //前
#define LINE6 5
#define LINE7 6
#define LINE8 7 //前
#define LINE9 8 //左
#define LINE10 9
#define LINE11 10
#define LINE12 11 //左
#define LINE13 13 //後ろ
#define LINE14 14
#define LINE15 15
#define LINE16 16//後ろ

void setup() {
  Serial.begin(9600);
  Wire.begin(11);
  i2c_faster();
  Wire.onRequest(requestEvent);
  int pin[16] = { LINE1,LINE2,LINE3,LINE4,LINE5,LINE6,LINE7,LINE8,LINE9,LINE10,LINE11,LINE12,LINE13,LINE14,LINE15,LINE16 };
  forPinMode(pin, 16,INPUT);
}
 uint8_t LINE_interrupt = 0;

void loop() {
  noInterrupts();
 uint8_t LINE_interrupt = 0;
  if ((digitalRead(LINE1) == HIGH) || (digitalRead(LINE2) == HIGH) || (digitalRead(LINE3) == HIGH) || (digitalRead(LINE4) == HIGH)) { //右が反応したら
    bitSet(LINE_interrupt, 0);
    bitSet(LINE_interrupt, 4);
  }
  if ((digitalRead(LINE5) == HIGH) || (digitalRead(LINE6) == HIGH) || (digitalRead(LINE7) == HIGH) || (digitalRead(LINE8) == HIGH)) { //前が反応したら
    bitSet(LINE_interrupt, 3);
    bitSet(LINE_interrupt, 4);
  }
  if ((digitalRead(LINE9) == HIGH) || (digitalRead(LINE10) == HIGH) || (digitalRead(LINE11) == HIGH) || (digitalRead(LINE12) == HIGH)) { //左が反応したら
    bitSet(LINE_interrupt, 2);
    bitSet(LINE_interrupt, 4);
  }
  if ((digitalRead(LINE13) == HIGH) || (digitalRead(LINE14) == HIGH) || (digitalRead(LINE15) == HIGH) || (digitalRead(LINE16) == HIGH)) { //後ろが反応したら
    bitSet(LINE_interrupt, 1);
    bitSet(LINE_interrupt, 4);
  }

  interrupts();

  Serial.println(LINE_interrupt,BIN);

}

void requestEvent() {
  uint8_t buf;
  buf = LINE_interrupt;
  Wire.write(buf);
}

