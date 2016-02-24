/**
 *	Project template 
 *
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://majerle.eu
 *	@version	v1.0
 *	@gcc		v4.7 20013qr3
 *	@ide		CooCox CoIDE v1.7.6
 */
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "tm_stm32f4_usb_vcp.h"

uint8_t IsData;
uint32_t Period;
uint32_t PrevCounter;

/*
void uint32_to_str(unsigned char str[], uint32_t src)
{
	char const digit[] = "0123456789";
	unsigned long tmp;

	unsigned char *i;
	if(src == 0)
	{
		*str = '0';
	}
	for(i=str; src > 0;i++)
	{
		tmp = src % 10;
		*i = digit[(unsigned char)tmp];
		src = src / 10;
	}
	i++;
	*i = 0;
	return;
}
*/
void uint32_to_str(uint32_t i, char b[])
{
    char const digit[] = "0123456789";
    char* p = b;
    int shifter = i;
    do
    { //Move to where representation ends
        ++p;
        shifter = shifter/10;
    }
    while(shifter);
    *p = '\0';
    do
    { //Move back, inserting digits as u go
        *--p = digit[i%10];
        i = i/10;
    }
    while(i);
}



void CaptureTimerInit(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_ICInitTypeDef TIM_ICStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	// Clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	// Timer
	TIM_BaseStruct.TIM_Prescaler = 0;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = 0xFFFFFFFF;
	TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseStruct.TIM_RepetitionCounter = 0;
	// Init
	TIM_TimeBaseInit(TIM2, &TIM_BaseStruct);
	// Start
	TIM_Cmd(TIM2, ENABLE);

	// Capture
	TIM_ICStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICStruct.TIM_ICPolarity = TIM_OCPolarity_High;
	TIM_ICStruct.TIM_ICFilter = 0;
	TIM_ICStruct.TIM_ICPrescaler = 0;
	TIM_ICStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	// Init
	TIM_ICInit(TIM2, &TIM_ICStruct);

	// I/O

	// Alternating functions for pins
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);

	// Set pin
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	// GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;		// Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;	// No Pull-Up/Down
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;		// Alternate function
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

    // Init
    GPIO_Init(GPIOA, &GPIO_InitStruct);

}

// Clock generation TIM10 - PF6

void TestSignalInit(void)
{
	// Definition
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	// Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

	TIM_BaseStruct.TIM_Prescaler = 0;
    TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseStruct.TIM_Period = 33599; /* 5kHz PWM */
    TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_BaseStruct.TIM_RepetitionCounter = 0;
	// Init TIM10
    TIM_TimeBaseInit(TIM10, &TIM_BaseStruct);
	// Start TIM10
    TIM_Cmd(TIM10, ENABLE);

    // Setup PWM
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCStruct.TIM_Pulse = 16799; // 50% duty cycle
	// Init PWM
	TIM_OC1Init(TIM10, &TIM_OCStruct);
	TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);

	// GPIO
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_TIM10);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;		// Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;	// No Pull-Up/Down
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;		// Alternate function
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOF, &GPIO_InitStruct);
}

// TIM2 Interrupt handler (override)
void TIM2_IRQHandler(void)
{
	IsData = 1;

}


int main(void)
{
	uint8_t c;
	unsigned char buff[11];
	//168MHz clock
	SystemInit();
	TM_USB_VCP_Init();

	IsData = 0;
	Period = 23465789;

	uint32_to_str(Period, buff);
	// sprintf(buff, "%d", Period);

	TestSignalInit();
    while (1)
    {
    	if (TM_USB_VCP_GetStatus() == TM_USB_VCP_CONNECTED)
    	{
    		if (TM_USB_VCP_Getc(&c) == TM_USB_VCP_DATA_OK)
    		{
    			while(TM_USB_VCP_Getc(&c) == TM_USB_VCP_DATA_EMPTY)
    			{
    				if(IsData)
    				{
    					IsData = 0;
    					uint32_to_str(Period, buff);
						TM_USB_VCP_Puts(buff);
						TM_USB_VCP_Puts("\r\n");
    				}
    			}
    		}
    	}
    }
}
