#include "hd44780.h"
#include "display.h"
void DISPLAY_Init()
{
	HD44780_Init();
	HD44780_LocationSet(DISPLAY_POS_RPM_X + 6, DISPLAY_POS_RPM_Y);
	HD44780_DisplayString("rpm");
	HD44780_LocationSet(DISPLAY_POS_DUTY_X, DISPLAY_POS_DUTY_Y);
	HD44780_DisplayString("duty:");
	HD44780_LocationSet(DISPLAY_POS_DUTY_X+ 13, DISPLAY_POS_DUTY_Y);
	HD44780_DisplayString("%");
//	HD44780CursorOn(xfalse);
//	HD44780CursorOff();
}

void DISPLAY_RPM(unsigned long rpm)
{
	HD44780_DisplayN_POS(rpm, DISPLAY_POS_RPM_X + 4, DISPLAY_POS_RPM_Y,5,0);
}

void DISPLAY_DUTY(unsigned int duty)
{
	unsigned long duty_percent;
	duty_percent = duty << 2;	// the PWM has 250 steps. If I shift by 2 means x4. The result is percentage x10
	HD44780_DisplayN_POS(duty_percent, DISPLAY_POS_DUTY_X + 11, DISPLAY_POS_DUTY_Y,5,1);
}
