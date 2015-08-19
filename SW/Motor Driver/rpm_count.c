#include "sys.h"
#include "timer.h"
#include "rpm_count.h"


volatile static unsigned long prevccr; // Previous value of the capture register
volatile static unsigned long counter;
volatile static unsigned long count_result;

volatile static unsigned int count_pointer;
volatile static unsigned char countset_ready;
volatile static unsigned long countset_result;
volatile static unsigned long count_result_set[13];

// Interrupt interlock error handling
volatile static unsigned char race_error;

void RPM_Clear()
{
	int i;
	counter = 0;
	countset_ready = 0;
	count_pointer = 0;
	countset_result = 0;
	for(i=0; i<13; i++)
	{
		count_result_set[i] = 0;
	}
}

void RPM_Add(unsigned long value)
{
	countset_result += value;
	count_result_set[count_pointer] = value;
	count_pointer = ((count_pointer == 12) ? 0 : (count_pointer + 1));
	countset_result -= count_result_set[count_pointer];
	if(count_pointer == 0)
	{
		countset_ready = 1;
	}
}


void TMR2_IRQHandler(void)
{
	unsigned long ccrvalue;
	if(_TIMER_GET_CMP_INT_FLAG(TIMER2))
	{

		if(counter < 0x7EFFFFFF && race_error == 0)
		{
			counter += 0x1000000;
		}
		race_error = 0;
		_TIMER_CLEAR_CMP_INT_FLAG(TIMER2);
		if(counter > 0x2000000)
		{
			RPM_Clear();
		}
	}
	if(_TIMER_GET_CAP_INT_FLAG(TIMER2))
	{
		/*
		_TIMER_CLEAR_CAP_INT_FLAG(TIMER2);

		if(tmr_int_handler_running)
		{
			while(serial_busy);
			serial_busy = 1;
			SERIAL_SendStr("e\r\n");
			serial_busy = 0;
		}
		else
		{
			tmr_int_handler_running = 1;
			*/
			// get the capture data
			ccrvalue = TIMER2->TCAP & 0xFFFFFF;

			// if there is a race condition between the two function of the
			// interrupt handler
			// when a compare and the capture event happens almost the same time
			// and the capture event wins the counter value kept at 0 (the compare event increase it)
			// the previous value is higher because the counter reset already happened.
			if(prevccr > ccrvalue && counter == 0)
			{
				counter += 0x1000000;
				race_error = 1;
			}
			// store the actual value of the counter
			count_result = counter;
			// clear master counter
			counter = 0;

			// process data
			//
			//          || <--------------------------------------------------- count_result ------------------------------------------------> ||
			//  prevccr || 0x10000 - prevccr | 0x10000 overflow | 0x10000 overflow | ..... | 0x10000 overflow | 0x10000 overflow | ccr ||
			count_result += ccrvalue;
			count_result -=	prevccr;
			// store current ccr for the next count
			prevccr = ccrvalue;

			if(count_result > 0x1000000)
			{
				count_result -= 0x1000000;
			}
			// countset_result++;
			RPM_Add(count_result);
			// Clear TIMER2 Capture Interrupt Flag
			_TIMER_CLEAR_CAP_INT_FLAG(TIMER2);
			/*
			if(CONTROL_FUNCTION == CONTROL_FUNCTION_NORMAL)
			{
				while(serial_busy);
				serial_busy = 1;
				SERIAL_SendULong(count_result);
				SERIAL_SendStr("\r\n");
				serial_busy = 0;
			}
			tmr_int_handler_running = 0;
		}
		*/
	}
}





void RPM_Init()
{
    // Enable IP clock
    SYSCLK->APBCLK |= SYSCLK_APBCLK_TMR2_EN_Msk;
    // IP clock source
    SYSCLK->CLKSEL1 = (SYSCLK->CLKSEL1 & ~SYSCLK_CLKSEL1_TMR2_S_Msk) | SYSCLK_CLKSEL1_TMR2_HCLK;

    SYS->P4_MFP = (SYS->P4_MFP & ~SYS_MFP_P40_Msk) | SYS_MFP_P40_T2EX;
    NVIC_EnableIRQ(TMR2_IRQn);
	_TIMER_RESET(TIMER2);

    // Enable TIMER1 counter input and capture function
    TIMER2->TCMPR = 0;
    TIMER2->TCSR = TIMER_TCSR_CEN_ENABLE | TIMER_TCSR_IE_ENABLE | TIMER_TCSR_MODE_PERIODIC | TIMER_TCSR_PRESCALE(1);
    TIMER2->TEXCON = TIMER_TEXCON_MODE_CAP | TIMER_TEXCON_TEXIEN_ENABLE | TIMER_TEXCON_TEX_EDGE_RISING | TIMER_TEXCON_TEXEN_ENABLE | TIMER_TEXCON_TEXDB_ENABLE;

    _TIMER_CLEAR_CAP_INT_FLAG(TIMER2);
    _TIMER_CLEAR_CMP_INT_FLAG(TIMER2);
}
