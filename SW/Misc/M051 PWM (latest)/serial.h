#include "uart.h"

#define SERIAL_SendChar(x) _UART_SENDBYTE(UART1,x)

extern void SERIAL_Init(void);
extern void SERIAL_SendStr(char* str);
extern void SERIAL_SendULong(unsigned long n);
