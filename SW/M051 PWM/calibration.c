// This code is used to measure the unloaded rotational speed versus pwm duty cycle

#include "sys.h"
#include "fmc.h"
#include "motor.h"
#include "rpm_count.h"
#include "calibration.h"
#include "display.h"
#include "serial.h"
#include "main.h"

void CALIBRATION_START()
{
	unsigned long duty;
	unsigned long period;
	int i;
	DISPLAY_Mode(DISPLAY_MODE_CAL);

	for(duty = 0; duty < 250; duty++)
	{
		// Set the duty cycle
		Motor_SetDuty(duty);
		GLOBAL_MOTOR_DUTY_CHANGED = 1;
		// Wait until the motor speed up
		for(i=0;i<PWM_RPM_SET_TIME;i++)
			SYS_SysTickDelay(1000);

		// read and create an average of the rotational speed
		period = 0;
		for(i=0;i<32;i++)
		{
			period += RPM_COUNT_GetPeriodLength();
			SYS_SysTickDelay(10000);
		}
		period = 3000000000uL / (period >> 5);

		// store/display data

		// display duty, 3000000000 / (period >> 5);
		// DISPLAY_CAL_DATA(period, duty);
		DISPLAY_CAL_DATA(period, duty);

		SERIAL_SendULong(duty);
		SERIAL_SendChar(',');
		SERIAL_SendULong(period);
		SERIAL_SendStr("\r\n");
	}
	DISPLAY_Mode(DISPLAY_MODE_NORMAL);
	GLOBAL_MOTOR_RPMPRESET_CHANGED = 1;
	Motor_GracefullStop();
}
