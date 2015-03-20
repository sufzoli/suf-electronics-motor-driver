// This code is used to measure the unloaded rotational speed versus pwm duty cycle

#include "sys.h"
#include "fmc.h"
#include "motor_pwm.h"
#include "rpm_count.h"

void PWM_RPM_Collect()
{
	unsigned long duty;
	unsigned long period;
	int i;
	for(duty = 0; duty < 250; duty++)
	{
		// Set the duty cycle
		MOTOR_PWM_SetDuty(duty);
		// Wait until the motor speed up (5s)
		SYS_SysTickDelay(5000000);
		// read and create an average of the rotational speed
		period = 0;
		for(i=0;i<32;i++)
		{
			period += RPM_COUNT_GetPeriodLength();
			SYS_SysTickDelay(10000);
		}
		// store/display data
		// display duty, 3000000000 / (period >> 5);
	}
}
