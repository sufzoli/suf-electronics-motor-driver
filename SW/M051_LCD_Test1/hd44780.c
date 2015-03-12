#include "GPIO.h"
#include "SYS.h"
#include "hd44780.h"


// static unsigned int _hd44780_queue[HD44780_QUEUE_LEN];

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
	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_CLEAR);
	SYS_SysTickDelay(2000);
	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_RETURN_HOME);
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
//	HD44780_Write(HD44780_RS_COMMAND, 0x28);
	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_FUNC | HD44780_FUNC_LINES);
	// 2. Display on, cursor off, blink off
//	HD44780_Write(HD44780_RS_COMMAND, 0x0E);
	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_DISP | HD44780_DISP_ON);
	// 3. Cursor move auto-increment
//	HD44780_Write(HD44780_RS_COMMAND, 0x06);
	HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_ENTRY_MODE | HD44780_ENTRY_ADDR_INC);
	// Clear screen
	HD44780_CLS();
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
/*
void HD44780_DisplayN_POS(unsigned long n, unsigned char x, unsigned char y, unsigned char len, unsigned char dp)
{
    unsigned char xpos;
    xpos = x;
    if (n == 0)
    {
    	HD44780_LocationSet(xpos, y);
        HD44780_Write(HD44780_RS_DATA,'0');
    }
    else
    {
    	while(n != 0 || (dp != 0 && xpos + dp + 1  > x))
    	{
        	if(x-xpos == dp && dp != 0)
        	{
        		HD44780_LocationSet(xpos, y);
        		HD44780_Write(HD44780_RS_DATA, '.');
               	xpos--;
        	}
    		HD44780_LocationSet(xpos, y);
        	if(n != 0)
        	{
				HD44780_Write(HD44780_RS_DATA, (n % 10) + '0');
				n /= 10;
        	}
        	else
        	{
        		HD44780_Write(HD44780_RS_DATA, '0');
        	}
        	xpos--;
    	}
    	while(x-xpos < len)
    	{
    		HD44780_LocationSet(xpos, y);
    		HD44780_Write(HD44780_RS_DATA, ' ');
    		xpos--;
    	}
    }
}
*/
void HD44780_DisplayN_POS(unsigned long n, unsigned char x, unsigned char y, unsigned char len, unsigned char dp)
{
    unsigned char xpos;
    xpos = x;

    HD44780_LocationSet(xpos, y);
    HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_ENTRY_MODE | HD44780_ENTRY_ADDR_DEC);

    if (n == 0)
    {
        HD44780_Write(HD44780_RS_DATA,'0');
    }
    else
    {
    	while(n != 0 || (dp != 0 && xpos + dp + 1  > x))
    	{
        	if(x-xpos == dp && dp != 0)
        	{
        		HD44780_Write(HD44780_RS_DATA, '.');
               	xpos--;
        	}
        	if(n != 0)
        	{
				HD44780_Write(HD44780_RS_DATA, (n % 10) + '0');
				n /= 10;
        	}
        	else
        	{
        		HD44780_Write(HD44780_RS_DATA, '0');
        	}
        	xpos--;
    	}
    	while(x-xpos < len)
    	{
    		HD44780_Write(HD44780_RS_DATA, ' ');
    		xpos--;
    	}
    }
    HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_ENTRY_MODE | HD44780_ENTRY_ADDR_INC);
}


void HD44780_LocationSet(unsigned char x, unsigned char y)
{
    unsigned char ulAddr = 0;

    // need to handle, the 4 line display
#if (HD44780_FUNC_LINES == HD44780_FUNC_LINES_1)
    ulAddr = x;
#else
    ulAddr = 0x40 * y + x;
#endif

    // Set the DDRAM address.
    HD44780_Write(HD44780_RS_COMMAND, HD44780_CMD_DDRAM_ADDR | ulAddr);
}

