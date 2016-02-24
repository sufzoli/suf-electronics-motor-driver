#include "motor.h"
#include "expwm.h"
#include "pwm.h"
#include "main.h"
#include "sys.h"

expwmtype MotorPWM;
unsigned int _Motor_duty = 0;

void Motor_Init()
{
	MotorPWM.Channel = 6;
	MotorPWM.Prescaler = 1;
	MotorPWM.Divider = PWM_CSR_DIV1;
	MotorPWM.Duty = 0;
	MotorPWM.Period = 249;
	MotorPWM.Port = EXPWM_PORT_DISABLE;	// Start with disabled output
	EXPWM_Init(&MotorPWM);
	MotorPWM.Port = EXPWM_PORT2; // Set to PORT2 - The port will be first initialized when the duty changed from 0
}

void _Motor_SetZero()
{
	_Motor_duty = 0;
	EXPWM_PortDisable(&MotorPWM);
	// May switch to GPIO and set output 0
}

void _Motor_EnablePWM()
{
	EXPWM_PortEnable(&MotorPWM);
}

unsigned long Motor_GetDuty()
{
	return _Motor_duty;
}

void Motor_SetDuty(unsigned int duty)
{
	if(duty == 0)
	{
		_Motor_SetZero();
	}
	else
	{
		// The motor was previously switched off, so need to enable
		if(_Motor_duty == 0)
		{
			_Motor_duty = duty;
			EXPWM_SetDuty(&MotorPWM, duty - 1);
			_Motor_EnablePWM();
		}
		else
		{
			_Motor_duty = duty;
			EXPWM_SetDuty(&MotorPWM, duty - 1);
		}
	}



	_Motor_duty = duty;
	if(_Motor_duty == 0)
	{

	}
	else
	{
		EXPWM_SetDuty(&MotorPWM, duty);
	}
}

void Motor_GracefullStop()
{
	unsigned long i,j;
	for(i = MotorPWM.Period; i >0 ; i--)
	{
		Motor_SetDuty(i);
		//MotorPWM.Duty = i;
		GLOBAL_MOTOR_DUTY_CHANGED = 1;
		for(j = 0; j < MOTOR_GRACEFULL_STEPTIME; j++)
		{
			SYS_SysTickDelay(1000);
		}
	}
}
