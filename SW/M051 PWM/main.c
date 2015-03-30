
/*
 * Port configuration
 *
 * P0.0 - Current Measure ADC (40)
 * P0.1 - Voltage Measure ADC (41)
 * P0.2 - Rotary Encoder A (38)
 * P0.3 - Rotary Encoder B (37)
 * P0.4 - RPM Mode Set (35)
 * P0.5 - Auto Mode Set (34)
 * P0.6 - Calibration start (33)
 *
 * P1.0 - TFT D/C (43)
 * P1.1 - TFT Reset (44)
 * P1.2 - UART RXD (45)
 * P1.3 - UART TXD (46)
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

#include "ili9341.h"
#include "ili9341_fonts.h"
#include "encoder.h"
#include "calibration.h"
#include "expwm.h"
#include "control.h"
#include "motor.h"
#include "serial.h"

#define PLLCON_SETTING      SYSCLK_PLLCON_50MHz_XTAL
#define PLL_CLOCK           50000000


// Global variables

unsigned char GLOBAL_MOTOR_DUTY_CHANGED;	// set to true if somewhere the motor pwm duty cycle changed

const signed char ENCODER_TABLE[] = {0,-1,+1,0,+1,0,0,-1,-1,0,0,+1,0,+1,-1,0};
static unsigned char ENCODER_PREV_STATE;
ENCODER_Callback_Type ENCODER_USER_Callback;
//static unsigned long rpm_count;
volatile static unsigned long prevccr; // Previous value of the capture register
volatile static unsigned long counter;
volatile static unsigned long count_result;
// unsigned char result_ready;
unsigned char rpm_set_changed;
//unsigned int duty_pwm;

// RPM Counter
volatile static unsigned int count_pointer;
volatile static unsigned char countset_ready;
volatile static unsigned long countset_result;
volatile static unsigned long count_result_set[13];

// Interrupt interlock error handling
volatile static unsigned char race_error;

expwmtype TestPWM;
// expwmtype MotorPWM;
expwmtype DisplayPWM;




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
    SYSCLK->APBCLK = SYSCLK_APBCLK_TMR2_EN_Msk;
    // IP clock source
//    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_PWM01_HCLK | SYSCLK_CLKSEL1_PWM23_HCLK | SYSCLK_CLKSEL1_TMR2_HCLK;
    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_TMR2_HCLK;
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

    NVIC_SetPriority(TMR2_IRQn,0);
    NVIC_SetPriority(SysTick_IRQn,1);
    NVIC_SetPriority(GPIO_P0P1_IRQn,2);
    NVIC_SetPriority(PWMA_IRQn,3);

    // Lock protected registers
    SYS_LockReg();
}


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
	unsigned long disp_buff;
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

	if(GLOBAL_MOTOR_DUTY_CHANGED)
	{
		disp_buff = Motor_GetDuty();
		DISPLAY_DUTY(disp_buff);
		GLOBAL_MOTOR_DUTY_CHANGED = 0;
	}
	if(rpm_set_changed && CONTROL_MODE != CONTROL_MODE_DUTY)
	{
		disp_buff = CONTROL_RPM_SET;
		DISPLAY_RPM_SET(disp_buff);
		rpm_set_changed = 0;
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
    // Init Encoder GPIO
    _GPIO_SET_PIN_MODE(P0, 2, GPIO_PMD_INPUT);
    _GPIO_SET_PIN_MODE(P0, 3, GPIO_PMD_INPUT);

	ENCODER_USER_Callback = CallBackFunc;
	ENCODER_PREV_STATE = (ENCODER_PIN_B << 1) | ENCODER_PIN_A | (ENCODER_PREV_STATE << 2);

	// Init Encoder Interrupt
    GPIO_EnableInt(P0, 2, GPIO_INT_RISING);
    GPIO_EnableInt(P0, 3, GPIO_INT_RISING);

    NVIC_EnableIRQ(GPIO_P0P1_IRQn);
}

void ENCODER_Callback(signed char cDirection)
{
	signed long temp;

	if(CONTROL_MODE == CONTROL_MODE_DUTY)
	{
		temp = Motor_GetDuty();
		temp += cDirection;
		if(temp >= 0 && temp < 250)
		{
			Motor_SetDuty(temp);
			// PWMA->CMR0 = temp;
			// duty_pwm = temp;
			GLOBAL_MOTOR_DUTY_CHANGED = 1;
		}
	}
	if(CONTROL_MODE == CONTROL_MODE_RPM)
	{
		temp = CONTROL_RPM_SET;
		temp += cDirection * 100;
		if(temp >= 0 && temp <= 25000)
		{
			CONTROL_RPM_SET = temp;
			rpm_set_changed = 1;
		}
	}
}

void RPM_Init()
{
    SYS->P4_MFP = (SYS->P4_MFP & ~SYS_MFP_P40_Msk) | SYS_MFP_P40_T2EX;
    NVIC_EnableIRQ(TMR2_IRQn);
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
/*
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
*/
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
    CONTROL_MODE = CONTROL_MODE_RPM;
    display_rpm_set_state = 1;

	counter = 0;
	RPM_Clear();

	ENCODER_Init(ENCODER_Callback);

	Motor_Init();

    // duty_pwm = 0;
    GLOBAL_MOTOR_DUTY_CHANGED = 1;

    rpm_set_changed = 1;

    // ---- PWM Timer used as display update

    DisplayPWM_Init();

    SERIAL_Init();
    //SERIAL_SendULong(25000);
    //SERIAL_SendStr("\r\n");

    CONTROL_INIT();



    // PWM timer used for test signal
    TEST_INIT();


    // Timer Rotational Speed Measurement
    RPM_Init();


    DISPLAY_CTRL_MODE(CONTROL_MODE);



    CONTROL_WORKER();

    /*
    while(1)
    {
//    	ILI9341_SendDataWord(0x5555);
    }
    */
}
