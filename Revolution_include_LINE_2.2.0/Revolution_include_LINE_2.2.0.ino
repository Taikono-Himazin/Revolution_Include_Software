#include <SPI.h>
#include <Utility.h>
#include <Wire.h>

#define mcp1SS 9
#define mcp2SS 10

uint8_t LINE_status = 0, Old_LINE = 0;
int16_t LINE[16], LINE_val;// LINE_val = 300;
bool i2c_flag = false;

void setup() {
	Serial.begin(115200);
	Wire.begin(11);
	i2c_faster();
	Wire.onRequest(requestEvent);
	Wire.onReceive(receiveEvent);

	pinMode(mcp1SS, OUTPUT);		//CS(10)
	pinMode(mcp2SS, OUTPUT);		//CS(9)
	digitalWrite(mcp1SS, HIGH);
	digitalWrite(mcp2SS, HIGH);
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.setDataMode(SPI_MODE0);
	SPI.begin();
}


void loop() {
	LINE_status = 0;
	for (uint8_t i = 0; i < 8; i++) {
			LINE[i] = mcp1Get(i);
		Serial.print(LINE[i]);
		Serial.print(",");
	}
	for (uint8_t i = 8; i < 16; i++) {
			LINE[i] = mcp2Get(i - 8);
		Serial.print(LINE[i]);
		Serial.print(",");
	}
	Serial.println("");

	for (uint8_t i = 0; i < 16; i++) {
		switch (i)
		{
		case 4://前
		case 5:
		case 6:
		case 7:
			if (LINE[i]<LINE_val) {// >黒反応(寮部屋モード) <白反応(学習室・大会モード)
				bitSet(LINE_status, 3);
			}
			break;

		case 0://左
		case 1:
		case 2:
		case 3:
			if (LINE[i] < LINE_val) {
				bitSet(LINE_status, 2);
			}
			break;

		case 8://後ろ
		case 9:
		case 10:
		case 11:
			if (LINE[i] < LINE_val) {
				bitSet(LINE_status, 1);
			}
			break;

		case 12://右
		case 13:
		case 14:
		case 15:
			if (LINE[i] < LINE_val) {
				bitSet(LINE_status, 0);
			}
			break;
		default:
			break;
		}
		if (bitRead(LINE_status, 0) == 1 || bitRead(LINE_status, 1) == 1 || bitRead(LINE_status, 2) == 1 || bitRead(LINE_status, 3) == 1) {
			bitSet(LINE_status, 4);
		}
	}
	Old_LINE = LINE_status;
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
	Wire.write(Old_LINE);
}

void receiveEvent(int x) {

	LINE_val= (Wire.read() << 8) | Wire.read();
	/*送り方
	buf[0] = (IR_Degree >> 8) & 0x00ff;
	buf[1] = IR_Degree & 0x00ff;
	*/
}
