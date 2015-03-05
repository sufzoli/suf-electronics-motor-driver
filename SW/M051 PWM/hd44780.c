#include "hd44780.h"
#include "GPIO.h"
#include "SYS.h"

void HD44780_Write(unsigned char rs_mode, unsigned char data)
{
	HD44780_PIN_RS = rs_mode;
	HD44780_PIN_E = HD44780_E_ENABLE;

    HD44780_PIN_D7 = (data >> 7) & 0x01;
    HD44780_PIN_D6 = (data >> 6) & 0x01;
    HD44780_PIN_D5 = (data >> 5) & 0x01;
    HD44780_PIN_D4 = (data >> 4) & 0x01;

    HD44780_PIN_E = HD44780_E_DISABLE;
    SYS_SysTickDelay(HD44780_EN_DELAY);
    HD44780_PIN_E = HD44780_E_ENABLE;

    HD44780_PIN_D7 = (data >> 3) & 0x01;
    HD44780_PIN_D6 = (data >> 2) & 0x01;
    HD44780_PIN_D5 = (data >> 1) & 0x01;
    HD44780_PIN_D4 = data & 0x01;

    SYS_SysTickDelay(HD44780_EN_DELAY);
    HD44780_PIN_E = HD44780_E_DISABLE;
}

void HD44780_Reset()
{
	HD44780_PIN_RS = rs_mode;
	HD44780_PIN_E = HD44780_E_ENABLE;

    HD44780_PIN_D7 = 0;
    HD44780_PIN_D6 = 0;
    HD44780_PIN_D5 = 0;
    HD44780_PIN_D4 = 1;

    HD44780_PIN_E = HD44780_E_DISABLE;
    SYS_SysTickDelay(HD44780_EN_DELAY);
}


void HD44780_Init()
{
	HD44780_PIN_E = HD44780_E_DISABLE;
	HD44780_PIN_D4 = 0;
	HD44780_PIN_D5 = 0;
	HD44780_PIN_D6 = 0;
	HD44780_PIN_D7 = 0;
	HD44780_PIN_RS = 0;
//	SYS_SysTickDelay(HD44780_INIT_DELAY);
//	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_RESET);
	SYS_SysTickDelay(HD44780_INIT_DELAY);
	// Set entry mode:  Interface Data Length, Character Font, Display Line
	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_FUNC | HD44780_FUNC_BUS | HD44780_FUNC_LINES | HD44780_FUNC_FONT);
	SYS_SysTickDelay(HD44780_EN_DELAY);
	// Display on & Cursor Blink
//	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_DISP | HD44780_DISP_ON | HD44780_DISP_BLINK_ON);
	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_DISP | HD44780_DISP_ON);
	SYS_SysTickDelay(HD44780_EN_DELAY);
	// Clear Screen
	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_CLEAR);
	SYS_SysTickDelay(HD44780_EN_DELAY);
	// Cursor Move Mode: Increment
	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_ENTRY_MODE | HD44780_ENTRY_ADDR_INC);
	SYS_SysTickDelay(HD44780_EN_DELAY);
}

void HD44780_DisplayString(const char* pcString)
{
    int i = 0;
    while(pcString[i] != 0)
    {
    	SYS_SysTickDelay(HD44780_EN_DELAY);
        HD44780_Write(HD44780_RS_DATA, pcString[i]);
        i++;
    }
}

void HD44780_DisplayN_POS(unsigned long n, unsigned char x, unsigned char y, unsigned char len)
{
    unsigned char xpos;
    xpos = x;
    if (n == 0)
    {
    	HD44780_LocationSet(xpos, y);
    	SYS_SysTickDelay(HD44780_EN_DELAY);
        HD44780_Write(HD44780_RS_DATA,'0');
    }
    else
    {
    	while(n != 0)
    	{
    		HD44780_LocationSet(xpos, y);
        	SYS_SysTickDelay(HD44780_EN_DELAY);
    		HD44780_Write(HD44780_RS_DATA, (n % 10) + '0');
        	n /= 10;
        	xpos--;
    	}
    	while(x-xpos < len)
    	{
    		HD44780_LocationSet(xpos, y);
        	SYS_SysTickDelay(HD44780_EN_DELAY);
    		HD44780_Write(HD44780_RS_DATA, ' ');
    		xpos--;
    	}
    }
}


void HD44780_LocationSet(unsigned char x, unsigned char y)
{
    unsigned char ulAddr = 0;

#if (HD44780_FUNC_LINES == HD44780_FUNC_LINES_1)
    ulAddr = x;
#else
    ulAddr = 0x40 * y + x;
#endif

    // Set the DDRAM address.
    SYS_SysTickDelay(HD44780_EN_DELAY);
    HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_DDRAM_ADDR | ulAddr);
}


