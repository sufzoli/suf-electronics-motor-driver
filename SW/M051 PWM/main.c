
/*
 * Port configuration
 *
 * P0.0 - Current Measure ADC (40)
 * P0.1 - Voltage Measure ADC (41)
 * P0.2 - Rotary Encoder A (38)
 * P0.3 - Rotary Encoder B (37)
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

#include "hd44780.h"
#include "encoder.h"

#define PLLCON_SETTING      SYSCLK_PLLCON_50MHz_XTAL
#define PLL_CLOCK           50000000


void SYS_Init(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC clock */
    SYSCLK->PWRCON |= SYSCLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC */
    SYSCLK->CLKSEL0 = SYSCLK_CLKSEL0_HCLK_IRC22M;

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    SYSCLK->PLLCON |= SYSCLK_PLLCON_PD_Msk;

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    SYSCLK->PWRCON |= SYSCLK_PWRCON_XTL12M_EN_Msk | SYSCLK_PWRCON_IRC22M_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    SYSCLK->PLLCON = PLLCON_SETTING;

    /* Waiting for clock ready */
    SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_PLL_STB_Msk | SYSCLK_CLKSTATUS_XTL12M_STB_Msk | SYSCLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    SYSCLK->CLKSEL0 = SYSCLK_CLKSEL0_STCLK_HCLK_DIV2 | SYSCLK_CLKSEL0_HCLK_PLL;

    /* Enable IP clock */
    SYSCLK->APBCLK = SYSCLK_APBCLK_PWM01_EN_Msk | SYSCLK_APBCLK_PWM23_EN_Msk | SYSCLK_APBCLK_TMR2_EN_Msk;
    /* IP clock source */
    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_PWM01_HCLK | SYSCLK_CLKSEL1_PWM23_HCLK | SYSCLK_CLKSEL1_TMR2_HCLK;
    /* IP clock source */
    // SYSCLK->CLKSEL2 = SYSCLK_CLKSEL2_PWM01_XTAL|SYSCLK_CLKSEL2_PWM23_XTAL;

    /* Reset PWMA channel0~channel3 */
    SYS->IPRSTC2 = SYS_IPRSTC2_PWM03_RST_Msk;
    SYS->IPRSTC2 = 0;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
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

    // Init Encoder GPIO
    _GPIO_SET_PIN_MODE(P0, 2, GPIO_PMD_INPUT);
    _GPIO_SET_PIN_MODE(P0, 3, GPIO_PMD_INPUT);
    /*
    _GPIO_ENABLE_DEBOUNCE(P0, 2);
    _GPIO_ENABLE_DEBOUNCE(P0, 3);
    _GPIO_SET_DEBOUNCE_TIME(GPIO_DBNCECON_DBCLKSRC_HCLK, GPIO_DBNCECON_DBCLKSEL_16);
    */
    GPIO_EnableInt(P0, 2, GPIO_INT_RISING);
    GPIO_EnableInt(P0, 3, GPIO_INT_RISING);

    NVIC_SetPriority(TMR2_IRQn,0);
    NVIC_SetPriority(SysTick_IRQn,1);
    NVIC_SetPriority(GPIO_P0P1_IRQn,2);
    NVIC_SetPriority(PWMA_IRQn,3);

    NVIC_EnableIRQ(GPIO_P0P1_IRQn);
    NVIC_EnableIRQ(TMR2_IRQn);
    NVIC_EnableIRQ(PWMA_IRQn);


/*---------------------------------------------------------------------------------------------------------*/
/* Init I/O Multi-function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD  */
    // SYS->P3_MFP = SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0;
    /* Set P2 multi-function pins for PWMB Channel0~3  */
    SYS->P2_MFP = SYS_MFP_P20_PWM0 | SYS_MFP_P22_PWM2; // P2.2 teszt jel, késõbb leszedendõ
    SYS->P4_MFP = SYS_MFP_P40_T2EX;


    /* Lock protected registers */
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
	int i;
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

void PWMA_IRQHandler(void)
{
	unsigned long disp_count;
	unsigned int disp_duty;
	_PWM_CLEAR_TIMER_PERIOD_INT_FLAG(PWMA,PWM_CH3);
	if(countset_ready && (countset_result > 0))
	{
		disp_count = 3000000000 / countset_result;
//		disp_count = countset_result;
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

void GPIOP0P1_IRQHandler(void)
{
	SYS_SysTickDelay(100);
    ENCODER_PREV_STATE = (ENCODER_PIN_B << 1) | ENCODER_PIN_A | (ENCODER_PREV_STATE << 2);
    _GPIO_CLEAR_INT_STATUS(P0,2);
    _GPIO_CLEAR_INT_STATUS(P0,3);
    // encoder_count += ENCODER_TABLE[(ENCODER_PREV_STATE & 0x0f)];  /* Index into table */

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
		PWMA->CMR0 = temp;
		duty_pwm = temp;
		duty_changed = 1;
	}
}


int main(void)
{
	SYS_Init();
	DISPLAY_Init();

	counter = 0;
	RPM_Clear();

	ENCODER_Init(ENCODER_Callback);

	// ------ PWM -------

    /*Set Pwm mode*/
    _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMA,PWM_CH0);

    /*Set PWM Timer clock prescaler*/
    _PWM_SET_TIMER_PRESCALE(PWMA,PWM_CH0, 1); // Divided by 2

    /*Set PWM Timer clock divider select*/
    _PWM_SET_TIMER_CLOCK_DIV(PWMA,PWM_CH0,PWM_CSR_DIV1);

    /*Set PWM Timer duty*/
    PWMA->CMR0 = 0;
    duty_pwm = 0;
    duty_changed = 1;

    /*Set PWM Timer period*/
    PWMA->CNR0 = 249;

    /* Enable PWM Output pin */
    _PWM_ENABLE_PWM_OUT(PWMA, PWM_CH0);

    /* Enable Timer period Interrupt */
//    _PWM_ENABLE_TIMER_PERIOD_INT(PWMB, PWM_CH0);

    /* Enable PWMB NVIC */
//    NVIC_EnableIRQ((IRQn_Type)(PWMB_IRQn));

    /* Enable PWM Timer */
    _PWM_ENABLE_TIMER(PWMA, PWM_CH0);

    // ---- PWM Timer used as display update

    /*Set Pwm mode*/
    _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMA,PWM_CH3);

    /*Set PWM Timer clock prescaler*/
    _PWM_SET_TIMER_PRESCALE(PWMA,PWM_CH3, 255); // Divided by 256

    /*Set PWM Timer clock divider select*/
    _PWM_SET_TIMER_CLOCK_DIV(PWMA,PWM_CH3,PWM_CSR_DIV16);

    /*Set PWM Timer duty*/
    PWMA->CMR3 = 1250;

    /*Set PWM Timer period*/
    PWMA->CNR3 = 2500;

    /* Enable PWM Output pin */
//    _PWM_ENABLE_PWM_OUT(PWMA, PWM_CH0);

    /* Enable Timer period Interrupt */
    _PWM_ENABLE_TIMER_PERIOD_INT(PWMA, PWM_CH3);

    /* Enable PWMA NVIC */
    // NVIC_EnableIRQ(PWMA_IRQn);

    /* Enable PWM Timer */
    _PWM_ENABLE_TIMER(PWMA, PWM_CH3);



    // PWM timer used for test signal

    //	Set Pwm mode
    _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMA,PWM_CH2);
    // Set PWM Timer clock prescaler
    _PWM_SET_TIMER_PRESCALE(PWMA,PWM_CH2, 9); // Divided by 10
    // Set PWM Timer clock divider select
    _PWM_SET_TIMER_CLOCK_DIV(PWMA,PWM_CH2,PWM_CSR_DIV1);
    // Set PWM Timer duty
    PWMA->CMR2 = 499;
    // Set PWM Timer period
    PWMA->CNR2 = 999;
    // Enable PWM Output pin
    _PWM_ENABLE_PWM_OUT(PWMA, PWM_CH2);
    // Enable Timer period Interrupt
    // _PWM_ENABLE_TIMER_PERIOD_INT(PWMA, PWM_CH3);
    // Enable PWMA NVIC
    // NVIC_EnableIRQ(PWMA_IRQn);
    // Enable PWM Timer
    _PWM_ENABLE_TIMER(PWMA, PWM_CH2);

    // -------- Timer (Rotational Speed Measurement)  ------

    _TIMER_RESET(TIMER2);

    // Enable TIMER1 counter input and capture function
//    TIMER2->TCMPR = 0xFFFFFF;
    TIMER2->TCMPR = 0;
    TIMER2->TCSR = TIMER_TCSR_CEN_ENABLE | TIMER_TCSR_IE_ENABLE | TIMER_TCSR_MODE_PERIODIC | TIMER_TCSR_PRESCALE(1);
//    TIMER2->TCSR = TIMER_TCSR_CEN_Msk | TIMER_TCSR_IE_Msk | TIMER_TCSR_MODE_PERIODIC | TIMER_TCSR_TDR_EN_Msk | TIMER_TCSR_PRESCALE(1);
    TIMER2->TEXCON = TIMER_TEXCON_MODE_CAP | TIMER_TEXCON_TEXIEN_ENABLE | TIMER_TEXCON_TEX_EDGE_RISING | TIMER_TEXCON_TEXEN_ENABLE;

    _TIMER_CLEAR_CAP_INT_FLAG(TIMER2);
    _TIMER_CLEAR_CMP_INT_FLAG(TIMER2);

    // DISPLAY_DUTY(PWMA->CMR0);

    while(1) {}
}
