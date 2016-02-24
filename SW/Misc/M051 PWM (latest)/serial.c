#include "sys.h"
#include "uart.h"
#include "serial.h"

void SERIAL_Init(void)
{
    // Enable UART clock
    SYSCLK->APBCLK |= SYSCLK_APBCLK_UART1_EN_Msk;

    // UART clock source
    SYSCLK->CLKSEL1 = (SYSCLK->CLKSEL1 & ~SYSCLK_CLKSEL1_UART_S_Msk) | SYSCLK_CLKSEL1_UART_PLL;

    /* Set P3 multi-function pins for UART1 RXD and UART1 TXD */
    SYS->P1_MFP =  (SYS->P1_MFP & ~(SYS_MFP_P12_Msk | SYS_MFP_P13_Msk)) | SYS_MFP_P12_RXD1 | SYS_MFP_P13_TXD1;

    // Reset UART
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART1_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART1_RST_Msk;

    // Configure UART1 and set UART1 Baudrate
    UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_DIV_MODE2(50000000, 115200);
    _UART_SET_DATA_FORMAT(UART1, UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1);
}

void SERIAL_SendStr(char* str)
{
	int i = 0;
	while(str[i] != 0)
	{
		SERIAL_SendChar(str[i]);
		i++;
	}
}

void SERIAL_SendULong(unsigned long n)
{
	char data[11];
	int i = 0;
	if(n != 0)
	{
		while(n>0)
		{
			data[i] = n % 10;
			n /= 10;
			i++;
		}
		for(;i>0;i--)
		{
			SERIAL_SendChar(data[i-1] + '0');
		}
	}
	else
	{
		SERIAL_SendChar('0');
	}
}

