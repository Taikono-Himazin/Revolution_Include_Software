#include <Utility.h>
#include <Wire.h>

#define HC_F 13
#define HC_B 13
#define HC_L 13
#define HC_R 13
#define HC_F_M 13
#define HC_B_M 13
#define HC_L_M 13
#define HC_R_M 13

uint16_t F_val, B_val, L_val, R_val;

void setup() {
  // put your setup code here, to run once:
	Wire.begin(15);
	i2c_faster();
	Wire.onRequest(Request_Event);
}

void loop() {
	// put your main code here, to run repeatedly:
	B_val = HC_Get(HC_B);
	F_val = HC_Get(HC_F);
	R_val = HC_Get(HC_R);
	L_val = HC_Get(HC_L);
}

uint32_t HC_Get(uint8_t pin) {
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	delayMicroseconds(2);
	digitalWrite(pin, HIGH);
	delayMicroseconds(10);
	digitalWrite(pin, LOW);
	pinMode(pin, INPUT);
	return (pulseIn(pin, HIGH, 15000) / 58);//75ƒZƒ“ƒ`
}

void Request_Event() {
	if (digitalRead(HC_B_M) == HIGH) {
		uint8_t buf[2];
		buf[0] = (B_val >> 8) & 0x00ff;
		buf[1] = B_val & 0x00ff;
		Wire.write(buf, 2);
	}
	else if(digitalRead(HC_F_M) == HIGH) {
		uint8_t buf[2];
		buf[0] = (F_val >> 8) & 0x00ff;
		buf[1] = F_val & 0x00ff;
		Wire.write(buf, 2);
	}
	else if (digitalRead(HC_L_M) == HIGH) {
		uint8_t buf[2];
		buf[0] = (L_val >> 8) & 0x00ff;
		buf[1] = L_val & 0x00ff;
		Wire.write(buf, 2);
	}
	else if (digitalRead(HC_R_M) == HIGH) {
		uint8_t buf[2];
		buf[0] = (R_val >> 8) & 0x00ff;
		buf[1] = R_val & 0x00ff;
		Wire.write(buf, 2);
	}
	else {
		uint8_t buf[2];
		buf[0] =0;
		buf[1] =0;
		Wire.write(buf, 2);
	}
}