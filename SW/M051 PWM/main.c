
/*
 * Port configuration
 *
 * P0.0 - Current Measure ADC (40)
 * P0.1 - Voltage Measure ADC (41)
 * P0.2 - Rotary Encoder A (38)
 * P0.3 - Rotary Encoder B (37)
 *
 * P1.0 - TFT D/C (43)
 * P1.1 - TFT Reset (44)
 *
 *
 * P1.4 - TFT CS (45)
 * P1.5 - TFT MOSI
 * P1.6 - TFT MISO
 * P1.7 - TFT SCK
 *
 *
 * P2.0 - Motor Drive PWM (19)
 *
 * P2.2 - Software Control Signal Measure (22) - PWM Capture
 *
 *
 * P3.0 - LCD D4 (5)
 * P3.1 - LCD D5 (7)
 * P3.2 - LCD D6 (8)
 * P3.3 - LCD D7 (9)
 * P3.4 - LCD E (10)
 * P3.5 - LCD RS (11)
 *
 * P4.0 - Motor Rotation Measure (20) - TMR2 Capture
 *
 * P4.4 - I2C SCL (28)
 * P4.5 - I2C SDA (29)
 *
 */




#include "M051Series.h"
#include "sys.h"
#include "pwm.h"
#include "gpio.h"
#include "timer.h"


#include "display.h"

// #include "hd44780.h"
#include "ili9341.h"
#include "ili9341_fonts.h"
#include "encoder.h"
#include "pwm_freq.h"
#include "expwm.h"

#define PLLCON_SETTING      SYSCLK_PLLCON_50MHz_XTAL
#define PLL_CLOCK           50000000


void SYS_Init(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

    // Unlock protected registers
    SYS_UnlockReg();

    // Enable Internal RC clock
//    SYSCLK->PWRCON |= SYSCLK_PWRCON_IRC22M_EN_Msk;

    // Waiting for IRC22M clock ready
//    SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_IRC22M_STB_Msk);

    // Switch HCLK clock source to Internal RC
//    SYSCLK->CLKSEL0 = SYSCLK_CLKSEL0_HCLK_IRC22M;

    // Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.
//    SYSCLK->PLLCON |= SYSCLK_PLLCON_PD_Msk;

    // Enable external 12MHz XTAL, internal 22.1184MHz
//    SYSCLK->PWRCON |= SYSCLK_PWRCON_XTL12M_EN_Msk | SYSCLK_PWRCON_IRC22M_EN_Msk;
    SYSCLK->PWRCON |= SYSCLK_PWRCON_XTL12M_EN_Msk;

    // Enable PLL and Set PLL frequency
    SYSCLK->PLLCON = PLLCON_SETTING;

    // Waiting for clock ready
//    SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_PLL_STB_Msk | SYSCLK_CLKSTATUS_XTL12M_STB_Msk | SYSCLK_CLKSTATUS_IRC22M_STB_Msk);
    SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_PLL_STB_Msk | SYSCLK_CLKSTATUS_XTL12M_STB_Msk);

    // Switch HCLK clock source to PLL, STCLK to HCLK/2
    SYSCLK->CLKSEL0 = SYSCLK_CLKSEL0_STCLK_HCLK_DIV2 | SYSCLK_CLKSEL0_HCLK_PLL;

    // Enable IP clock
//    SYSCLK->APBCLK = SYSCLK_APBCLK_PWM01_EN_Msk | SYSCLK_APBCLK_PWM23_EN_Msk | SYSCLK_APBCLK_TMR2_EN_Msk;
    SYSCLK->APBCLK = SYSCLK_APBCLK_TMR2_EN_Msk | SYSCLK_APBCLK_SPI0_EN_Msk;
    // IP clock source
//    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_PWM01_HCLK | SYSCLK_CLKSEL1_PWM23_HCLK | SYSCLK_CLKSEL1_TMR2_HCLK;
    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_TMR2_HCLK | SYSCLK_CLKSEL1_SPI0_HCLK;
    // IP clock source
    // SYSCLK->CLKSEL2 = SYSCLK_CLKSEL2_PWM01_XTAL|SYSCLK_CLKSEL2_PWM23_XTAL;

    // Reset PWMA channel0~channel3
//    SYS->IPRSTC2 = SYS_IPRSTC2_PWM03_RST_Msk;
//    SYS->IPRSTC2 = 0;

    // Update System Core Clock
    // User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically.
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    // Init LCD GPIO
    _GPIO_SET_PIN_MODE(P3, 0, GPIO_PMD_OUTPUT);
    _GPIO_SET_PIN_MODE(P3, 1, GPIO_PMD_OUTPUT);
    _GPIO_SET_PIN_MODE(P3, 2, GPIO_PMD_OUTPUT);
    _GPIO_SET_PIN_MODE(P3, 3, GPIO_PMD_OUTPUT);
    _GPIO_SET_PIN_MODE(P3, 4, GPIO_PMD_OUTPUT);
    _GPIO_SET_PIN_MODE(P3, 5, GPIO_PMD_OUTPUT);

    // TFT GPIO
    _GPIO_SET_PIN_MODE(P1, 0, GPIO_PMD_OUTPUT);
    _GPIO_SET_PIN_MODE(P1, 1, GPIO_PMD_OUTPUT);

    // Init Encoder GPIO
    _GPIO_SET_PIN_MODE(P0, 2, GPIO_PMD_INPUT);
    _GPIO_SET_PIN_MODE(P0, 3, GPIO_PMD_INPUT);

    GPIO_EnableInt(P0, 2, GPIO_INT_RISING);
    GPIO_EnableInt(P0, 3, GPIO_INT_RISING);

    NVIC_SetPriority(TMR2_IRQn,0);
    NVIC_SetPriority(SysTick_IRQn,1);
    NVIC_SetPriority(GPIO_P0P1_IRQn,2);
    NVIC_SetPriority(PWMA_IRQn,3);

    NVIC_EnableIRQ(GPIO_P0P1_IRQn);
    NVIC_EnableIRQ(TMR2_IRQn);


/*---------------------------------------------------------------------------------------------------------*/
/* Init I/O Multi-function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
    // Set P3 multi-function pins for UART0 RXD and TXD
    // SYS->P3_MFP = SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0;
    // Set P2 multi-function pins for PWMB Channel0~3
//    SYS->P2_MFP = SYS_MFP_P20_PWM0 | SYS_MFP_P22_PWM2; // P2.2 teszt jel, késõbb leszedendõ
    SYS->P4_MFP = SYS_MFP_P40_T2EX;

    // Set P1.4, P1.5, P1.6, P1.7 for SPI0
    SYS->P1_MFP = SYS_MFP_P14_SPISS0 | SYS_MFP_P15_MOSI_0 | SYS_MFP_P16_MISO_0 | SYS_MFP_P17_SPICLK0;


    // Lock protected registers
    SYS_LockReg();
}


const signed char ENCODER_TABLE[] = {0,-1,+1,0,+1,0,0,-1,-1,0,0,+1,0,+1,-1,0};
static unsigned char ENCODER_PREV_STATE;
ENCODER_Callback_Type ENCODER_USER_Callback;
//static unsigned long rpm_count;
volatile static unsigned long prevccr; // Previous value of the capture register
volatile static unsigned long counter;
volatile static unsigned long count_result;
// unsigned char result_ready;
unsigned char duty_changed;
unsigned int duty_pwm;

// RPM Counter
volatile static unsigned int count_pointer;
volatile static unsigned char countset_ready;
volatile static unsigned long countset_result;
volatile static unsigned long count_result_set[13];

// Interrupt interlock error handling
volatile static unsigned char race_error;

expwmtype TestPWM;
expwmtype MotorPWM;
expwmtype DisplayPWM;

void RPM_Clear()
{
	int i;
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
//	int i;
	unsigned long ccrvalue;
	if(_TIMER_GET_CMP_INT_FLAG(TIMER2))
	{
		// countset_result++;
		// Itt kellene még kezelni az underflow-t
		// ha bekövetkezik, meghívni az RPM_Clear-t

		if(counter < 0x7EFFFFFF && race_error == 0)
		{
			counter += 0x1000000;
		}
		race_error = 0;
		_TIMER_CLEAR_CMP_INT_FLAG(TIMER2);


		if(counter > 0x2000000)
		{
			RPM_Clear();
			counter = 0;
		}
	}

	if(_TIMER_GET_CAP_INT_FLAG(TIMER2))
	{
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
	}

}

void DisplayPWM_Callback(void)
{
	unsigned long disp_count;
	unsigned int disp_duty;
	// _PWM_CLEAR_TIMER_PERIOD_INT_FLAG(PWMA,PWM_CH3);
	if(countset_ready && (countset_result > 0))
	{
		disp_count = 3000000000u / countset_result;
	}
	else
	{
		disp_count = 0;
	}
	DISPLAY_RPM(disp_count);

	if(duty_changed)
	{
		disp_duty = duty_pwm;
		duty_changed = 0;
		DISPLAY_DUTY(disp_duty);
	}

}

unsigned long RPM_COUNT_GetPeriodLength()
{
	return countset_ready ? countset_result : 0;
}

void GPIOP0P1_IRQHandler(void)
{
	SYS_SysTickDelay(200);
    ENCODER_PREV_STATE = (ENCODER_PIN_B << 1) | ENCODER_PIN_A | (ENCODER_PREV_STATE << 2);
    _GPIO_CLEAR_INT_STATUS(P0,2);
    _GPIO_CLEAR_INT_STATUS(P0,3);

    switch (ENCODER_TABLE[(ENCODER_PREV_STATE & 0x0f)])
    {
    	case -1:
    		ENCODER_USER_Callback(-1);
    		break;
    	case 1:
    		ENCODER_USER_Callback(1);
    	default:
    		break;
    }
}

void ENCODER_Init(ENCODER_Callback_Type CallBackFunc)
{
	ENCODER_USER_Callback = CallBackFunc;
	ENCODER_PREV_STATE = (ENCODER_PIN_B << 1) | ENCODER_PIN_A | (ENCODER_PREV_STATE << 2);
}

void ENCODER_Callback(signed char cDirection)
{

	signed long temp;
	temp = duty_pwm;
	temp += cDirection;
	if(temp >= 0 && temp < 250)
	{
		EXPWM_SetDuty(&MotorPWM, temp);
		// PWMA->CMR0 = temp;
		duty_pwm = temp;
		duty_changed = 1;
	}
}

void RPM_Init()
{
    _TIMER_RESET(TIMER2);

    // Enable TIMER1 counter input and capture function
    TIMER2->TCMPR = 0;
    TIMER2->TCSR = TIMER_TCSR_CEN_ENABLE | TIMER_TCSR_IE_ENABLE | TIMER_TCSR_MODE_PERIODIC | TIMER_TCSR_PRESCALE(1);
    TIMER2->TEXCON = TIMER_TEXCON_MODE_CAP | TIMER_TEXCON_TEXIEN_ENABLE | TIMER_TEXCON_TEX_EDGE_RISING | TIMER_TEXCON_TEXEN_ENABLE;

    _TIMER_CLEAR_CAP_INT_FLAG(TIMER2);
    _TIMER_CLEAR_CMP_INT_FLAG(TIMER2);
}

void TEST_INIT()
{
	TestPWM.Chanell = 2;
	TestPWM.Prescaler = 9;
	TestPWM.Divider = PWM_CSR_DIV1;
	TestPWM.Duty = 499;
	TestPWM.Period = 999;
	TestPWM.Port = EXPWM_PORT2;
	EXPWM_Init(&TestPWM);
}

void Motor_Init()
{
	MotorPWM.Chanell = 0;
	MotorPWM.Prescaler = 1;
	MotorPWM.Divider = PWM_CSR_DIV1;
	MotorPWM.Duty = 0;
	MotorPWM.Period = 249;
	MotorPWM.Port = EXPWM_PORT2;
	EXPWM_Init(&MotorPWM);
}

void DisplayPWM_Init()
{
	DisplayPWM.Chanell = 3;
	DisplayPWM.Prescaler = 255;
	DisplayPWM.Divider = PWM_CSR_DIV16;
	DisplayPWM.Duty = 1249;
	DisplayPWM.Period = 2499;
	DisplayPWM.Port = EXPWM_PORT_DISABLE;
	DisplayPWM.Callback = DisplayPWM_Callback;
	EXPWM_Init(&DisplayPWM);
}

int main(void)
{
	SYS_Init();
	DISPLAY_Init();
	// DISPLAY_Mode(DISPLAY_MODE_CAL);
	DISPLAY_Mode(DISPLAY_MODE_NORMAL);

	counter = 0;
	RPM_Clear();

	ENCODER_Init(ENCODER_Callback);

	Motor_Init();

    duty_pwm = 0;
    duty_changed = 1;

    // ---- PWM Timer used as display update

    DisplayPWM_Init();

    // PWM timer used for test signal
    TEST_INIT();
    // Timer Rotational Speed Measurement
    RPM_Init();

    // PWM_RPM_Collect();

    // TFT Test

    ILI9341_Init();

    int j;

//    ILI9341_PrintStr(&Font16x26, "Hello World!", 10, 10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    ILI9341_PrintStr(&Font16x26, "Hello", 10, 10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
    // ILI9341_PrintChar(&Font16x26, 'c', 10, 10, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
/*
    for(j=100; j<200;j++)
    	ILI9341_DrawPixel(100, j, ILI9341_COLOR_MAGENTA);
*/
    while(1)
    {
//    	ILI9341_SendDataWord(0x5555);
    }
}
