#include <SPI.h>
#include <Utility.h>
#include <Wire.h>

#define mcp1SS 10
#define mcp2SS 9

void setup() {
  Serial.begin(9600);
  Wire.begin(11);
  i2c_faster();
  Wire.onRequest(requestEvent);
  pinMode(mcp1SS, OUTPUT);		//CS(10)
  pinMode(mcp2SS, OUTPUT);		//CS(9)
  digitalWrite(mcp1SS, HIGH);
  digitalWrite(mcp2SS, HIGH);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.begin();
}
uint8_t LINE_status = 0;
int16_t LINE[16];

void loop() {
	for (uint8_t i = 0; i < 8; i++) {
		if ((i == 7)||(i==6)) {
			Serial.print("0,");
		}
		else {

			LINE[i] = mcp1Get(i);
			Serial.print(LINE[i]);
			Serial.print(",");
		}
	}
	for (uint8_t i = 8; i < 16; i++) {
		LINE[i] = mcp2Get(i - 8);
		Serial.print(LINE[i]);
		Serial.print(",");
		}
	Serial.println("");
// Serial.println(LINE_interrupt,BIN);.
}

int16_t mcp1Get(uint8_t ch)
{
	uint8_t data[2] = { 0, 0 };
	int16_t analog_data = 0;
	digitalWrite(mcp1SS, LOW);		//SS LOW
	SPI.transfer(0x01);
	data[0] = SPI.transfer((ch << 4) | 0x80);
	data[1] = SPI.transfer(0);
	analog_data = ((data[0] & 0x03) << 8) | data[1];
	digitalWrite(mcp1SS, HIGH);		//SS HIGH
	return analog_data;
}

int16_t mcp2Get(uint8_t ch)
{
	uint8_t data[2] = { 0, 0 };
	int16_t analog_data = 0;
	digitalWrite(mcp2SS, LOW);		//SS LOW
	SPI.transfer(0x01);
	data[0] = SPI.transfer((ch << 4) | 0x80);
	data[1] = SPI.transfer(0);
	analog_data = ((data[0] & 0x03) << 8) | data[1];
	digitalWrite(mcp2SS, HIGH);		//SS HIGH
	return analog_data;
}

void requestEvent() {
  uint8_t buf;
  buf = LINE_status;
  Wire.write(buf);
}

