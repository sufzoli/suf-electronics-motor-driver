#include "hd44780.h"
#include "display.h"
void DISPLAY_Init()
{
	HD44780Init();
	HD44780LocationSet(DISPLAY_POS_RPM_X + 6, DISPLAY_POS_RPM_Y);
	HD44780DisplayString("rpm");
	HD44780CursorOn(xfalse);
	HD44780CursorOff();
}

void DISPLAY_RPM(unsigned long rpm)
{
	HD44780DisplayN_POS(rpm, DISPLAY_POS_RPM_X + 4, DISPLAY_POS_RPM_Y,5);
}
