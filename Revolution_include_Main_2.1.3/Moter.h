// Moter.h

#ifndef _MOTER_h
#define _MOTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#include <Wire.h>
#include <PID_v1.h>
#else
	#include "WProgram.h"
#endif

class MoterClass
{
 protected:
	 double Setpoint, Input, Output;
	 double Kp = 2, Ki = 0, Kd = 0.01;
	 int16_t m1, m2, m3, m4;

	 void Compute(uint8_t  Force, int16_t Degree, bool PID_Mode);
	 void Send();
 public:
	void nomal(uint8_t Force, int16_t Degree);
	void off();
	void Spin(uint8_t Force, uint8_t D_Power);
};

extern MoterClass Moter;

#endif

