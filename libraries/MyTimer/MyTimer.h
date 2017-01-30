#ifndef MyTimer_H_
#define MyTimer_H_
#include "arduino.h"

class MyTimer
{
public:
	void start(void);
	void stop(void);
	void reset(void);
	uint32_t read_ms(void);
	uint32_t read_us(void);
private:
	uint32_t msTime, usTime;
	bool TimerState, Stopflag, Resetflag;
};

#endif