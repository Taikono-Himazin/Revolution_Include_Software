// 
// 
// 

#include "Moter.h"
#include <Wire.h>
#include <PID_v1.h>
double Setpoint, Input, Output;
double Kp = 2, Ki = 0, Kd = 0.01;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void MoterClass::Compute(uint8_t Force, int16_t Degree, bool PID_Mode)
{
	int16_t m1, m2;
	old_M_D = old_M_D*Old_Persent + Degree*(1 - Old_Persent);
	old_M_F = old_M_F*Old_Persent + Force*(1 - Old_Persent);

	int16_t m1_D = old_M_D - 45;
	if (m1_D < 0) m1_D = m1_D + 360;
	else if (m1_D > 359) m1_D = m1_D - 360;

	m1 = sin((float)m1_D * 0.01745329) * old_M_F; // sin �ł�cos����Ȃ��Ɨ���s�\

	int16_t m2_D = old_M_D - 315;
	if (m2_D < 0) m2_D = m2_D + 360;
	else if (m2_D > 359) m2_D = m2_D - 360;

	m2 = sin((float)m2_D * 0.01745329) * old_M_F;

	int16_t F_max = abs(m1);
	if (F_max < abs(m2)) F_max = abs(m2);

	float k = (float)old_M_F / F_max;//�e���[�^�[�̔��ۂ��Ȃ���ő�l��225��
	m1 = m1 * k;
	m2 = m2 * k;

	int16_t m3 = -m1;//���΂Ɉʒu���Ă��邩�甽�]
	int16_t m4 = -m2;

	if (PID_Mode) 
	{
		GyroGet();//�W���C���̃f�[�^���擾
		Input = Gyro; //�W���C���̃f�[�^��Input�֐��ɓ˂�����
		myPID.Compute(); //pid�v�Z

		m1 = m1 - Output;//pid�̃f�[�^�����[�^�[�ɓ˂�����
		m2 = m2 - Output;
		m3 = m3 - Output;
		m4 = m4 - Output;
		m1 = constrain(m1, -255, 255);
		m2 = constrain(m2, -255, 255);
		m3 = constrain(m3, -255, 255);
		m4 = constrain(m4, -255, 255);
	}
}

void MoterClass::Send()
{
	uint8_t buf[5];//���M
	bitSet(buf[4], 4); //���[�^�̓d��on
	if (m1 < 0) bitSet(buf[4], 0);
	else bitClear(buf[4], 0);
	buf[0] = abs(m1);
	if (m2< 0) bitSet(buf[4], 1);//���[�^�[�̔z�����ԈႦ��
	else bitClear(buf[4], 1);
	buf[1] = abs(m2);
	if (m3 < 0) bitSet(buf[4], 2);
	else bitClear(buf[4], 2);
	buf[2] = abs(m3);
	if (m4 < 0) bitSet(buf[4], 3);
	else bitClear(buf[4], 3);
	buf[3] = abs(m4);

	Wire.beginTransmission(10);
	Wire.write(buf, 5);
	Wire.endTransmission();
}

void MoterClass::nomal(uint8_t Force, int16_t Degree)
{
	Compute(Force,Degree,true);
	Send();
}

void MoterClass::off()
{
	uint8_t buf[5];//���[�^�ɑ��M
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	bitClear(buf[4], 4); //���[�^�[�d��off

	Wire.beginTransmission(10);
	Wire.write(buf, 5);
	Wire.endTransmission();
}

void MoterClass::Spin(uint8_t Force, uint8_t D_Power)
{
	m1 = Force;
	m2 = Force;
	m3 = Force;
	m4 = Force;

	Send();
}


MoterClass Moter;