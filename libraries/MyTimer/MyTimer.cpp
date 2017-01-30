#include "arduino.h"
#include "MyTimer.h"


void MyTimer::start(void)
{
	if (TimerState == 0) {
		msTime = millis();
		usTime = micros();
	}
	TimerState = 1;
	Stopflag = 0;
	Resetflag = 0;
}

void MyTimer::stop(void)
{
	TimerState = 0;
	if (Stopflag == 0) {
		msTime = millis()-msTime;
		usTime = micros()-usTime;
	}
	Stopflag = 1;
}

void MyTimer::reset(void)
{
	Resetflag = 1;
}

unsigned long MyTimer::read_ms(void)
{
	if (Resetflag) return 0;
	if (Stopflag) return (unsigned long)msTime;
	return (unsigned long)(millis()-msTime);
}

unsigned long MyTimer::read_us(void)
{
	if (Resetflag) return 0;
	if (Stopflag) return (unsigned long)usTime;
	return (unsigned long)(micros()-usTime);
}

