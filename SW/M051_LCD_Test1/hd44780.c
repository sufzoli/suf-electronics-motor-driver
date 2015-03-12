#include "GPIO.h"
#include "SYS.h"
#include "hd44780.h"


void HD44780_Pulse()
{
	HD44780_PIN_E = HD44780_E_DISABLE;
	SYS_SysTickDelay(HD44780_EN_DELAY);
	HD44780_PIN_E = HD44780_E_ENABLE;
	SYS_SysTickDelay(HD44780_EN_DELAY);
	HD44780_PIN_E = HD44780_E_DISABLE;
	SYS_SysTickDelay(HD44780_EN_DELAY);
}

void HD44780_ClearPins()
{
	HD44780_PIN_RS = 0;
	HD44780_PIN_E = 0;
    HD44780_PIN_D7 = 0;
    HD44780_PIN_D6 = 0;
    HD44780_PIN_D5 = 0;
    HD44780_PIN_D4 = 0;
}

void HD44780_Write(unsigned char rs_mode, unsigned char data)
{
	HD44780_ClearPins();

    HD44780_PIN_D7 = (data >> 7) & 0x01;
    HD44780_PIN_D6 = (data >> 6) & 0x01;
    HD44780_PIN_D5 = (data >> 5) & 0x01;
    HD44780_PIN_D4 = (data >> 4) & 0x01;
	HD44780_PIN_RS = rs_mode;
	HD44780_Pulse();

    HD44780_PIN_D7 = (data >> 3) & 0x01;
    HD44780_PIN_D6 = (data >> 2) & 0x01;
    HD44780_PIN_D5 = (data >> 1) & 0x01;
    HD44780_PIN_D4 = data & 0x01;
	HD44780_PIN_RS = rs_mode;
	HD44780_Pulse();
}

void HD44780_CLS()
{
	HD44780_Write(HD44780_RS_COMMAND, 0x01);
	SYS_SysTickDelay(2000);
	HD44780_Write(HD44780_RS_COMMAND, 0x02);
	SYS_SysTickDelay(2000);
}

void HD44780_Init()
{
	HD44780_ClearPins();
	SYS_SysTickDelay(HD44780_INIT_DELAY);
	// Set 4 bit mode
	HD44780_PIN_D5 = 1;
	HD44780_Pulse();
    // set 4-bit input - second time.
    // (as reqd by the spec.)
	HD44780_Write(HD44780_RS_COMMAND, 0x28);
	// 2. Display on, cursor on, blink cursor
	HD44780_Write(HD44780_RS_COMMAND, 0x0E);
	// 3. Cursor move auto-increment
	HD44780_Write(HD44780_RS_COMMAND, 0x06);
}

void HD44780_DisplayString(const char* pcString)
{
    int i = 0;
    while(pcString[i] != 0)
    {
        HD44780_Write(HD44780_RS_DATA, pcString[i]);
        i++;
    }
}

