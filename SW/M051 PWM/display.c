#include "hd44780.h"
#include "display.h"

unsigned char display_busy;

void DISPLAY_Init()
{
	HD44780_Init();
	display_busy = 0;
	HD44780_LocationSet(DISPLAY_POS_RPM_X + 6, DISPLAY_POS_RPM_Y);
	HD44780_DisplayString("rpm");
	HD44780_LocationSet(DISPLAY_POS_DUTY_X, DISPLAY_POS_DUTY_Y);
	HD44780_DisplayString("duty:");
	HD44780_LocationSet(DISPLAY_POS_DUTY_X+ 10, DISPLAY_POS_DUTY_Y);
	HD44780_DisplayString("%");
//	HD44780CursorOn(xfalse);
//	HD44780CursorOff();
}

void DISPLAY_RPM(unsigned long rpm)
{
	if(!display_busy)
	{
		display_busy = 1;
		HD44780_DisplayN_POS(rpm, DISPLAY_POS_RPM_X + 4, DISPLAY_POS_RPM_Y,8,0);
		display_busy = 0;
	}
//	HD44780_DisplayN_POS(rpm, DISPLAY_POS_RPM_X + 4, DISPLAY_POS_RPM_Y,7,0);
}

void DISPLAY_DUTY(unsigned int duty)
{
	unsigned long duty_percent;
	duty_percent = (duty + 1) << 2;	// the PWM has 250 steps. If I shift by 2 means x4. The result is percentage x10
	if(!display_busy)
	{
		display_busy = 1;
		HD44780_DisplayN_POS(duty_percent, DISPLAY_POS_DUTY_X + 9, DISPLAY_POS_DUTY_Y,5,1);
		display_busy = 0;
	}
}
