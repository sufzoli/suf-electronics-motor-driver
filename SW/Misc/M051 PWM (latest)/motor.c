#include "motor.h"
#include "expwm.h"
#include "pwm.h"
#include "main.h"
#include "sys.h"

expwmtype MotorPWM;

void Motor_Init()
{
	MotorPWM.Chanell = 0;
	MotorPWM.Prescaler = 1;
	MotorPWM.Divider = PWM_CSR_DIV1;
	MotorPWM.Duty = 0;
	MotorPWM.Period = 249;
	MotorPWM.Port = EXPWM_PORT2;
	EXPWM_Init(&MotorPWM);
}

unsigned long Motor_GetDuty()
{
	return MotorPWM.Duty;
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
