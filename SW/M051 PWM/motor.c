#include "expwm.h"
#include "pwm.h"

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
