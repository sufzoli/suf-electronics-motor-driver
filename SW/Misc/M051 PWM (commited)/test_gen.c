
#include "pwm.h"
#include "expwm.h"

expwmtype TestPWM;

void TEST_INIT()
{
	TestPWM.Chanell = 2;
	TestPWM.Prescaler = 9;
	TestPWM.Divider = PWM_CSR_DIV1;
	TestPWM.Duty = 499;
	TestPWM.Period = 999;
	TestPWM.Port = EXPWM_PORT2;
	EXPWM_Init(&TestPWM);
}
