
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



#include "main.h"
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
#include "pid.h"

#define PLLCON_SETTING      SYSCLK_PLLCON_50MHz_XTAL
#define PLL_CLOCK           50000000


// Global variables

volatile unsigned char GLOBAL_MOTOR_DUTY_CHANGED;	// set to true if somewhere the motor pwm duty cycle changed
volatile unsigned char GLOBAL_MOTOR_RPMPRESET_CHANGED; // set to true if somewhere the motor rpm preset changed
// volatile unsigned char GLOBAL_SEMAPHORE_RPM_DISP_AVG;	// semaphore to wait for reading the values for the rpm average
// volatile unsigned char GLOBAL_SEMAPHORE_RPM_WRITE_AVG;	// semaphore to wait for writing the values for the rpm average

volatile unsigned char tmr_int_handler_running;
volatile unsigned char serial_busy;

/*
const signed char ENCODER_TABLE[] = {0,-1,+1,0,+1,0,0,-1,-1,0,0,+1,0,+1,-1,0};
static unsigned char ENCODER_PREV_STATE;
ENCODER_Callback_Type ENCODER_USER_Callback;
*/
//static unsigned long rpm_count;
volatile static unsigned long prevccr; // Previous value of the capture register
volatile static unsigned long counter;
volatile static unsigned long count_result;
// unsigned char result_ready;
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

    // Enable external 12MHz XTAL
    SYSCLK->PWRCON |= SYSCLK_PWRCON_XTL12M_EN_Msk;

    // Enable PLL and Set PLL frequency
    SYSCLK->PLLCON = PLLCON_SETTING;

    // Waiting for clock ready
    SYS_WaitingForClockReady(SYSCLK_CLKSTATUS_PLL_STB_Msk | SYSCLK_CLKSTATUS_XTL12M_STB_Msk);

    // Switch HCLK clock source to PLL, STCLK to HCLK/2
    SYSCLK->CLKSEL0 = SYSCLK_CLKSEL0_STCLK_HCLK_DIV2 | SYSCLK_CLKSEL0_HCLK_PLL;

    /*
    // Enable IP clock
    SYSCLK->APBCLK = SYSCLK_APBCLK_TMR2_EN_Msk;
    // IP clock source
    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_TMR2_HCLK;
    */
    // Update System Core Clock
    // User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically.
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    // Set interrupt priorities
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
	unsigned long ccrvalue;
	if(_TIMER_GET_CMP_INT_FLAG(TIMER2))
	{
//		_TIMER_CLEAR_CMP_INT_FLAG(TIMER2);

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
	if(GLOBAL_MOTOR_RPMPRESET_CHANGED && CONTROL_MODE != CONTROL_MODE_DUTY)
	{
		disp_buff = CONTROL_RPM_SET;
		DISPLAY_RPM_SET(disp_buff);
		GLOBAL_MOTOR_RPMPRESET_CHANGED = 0;
	}
}

unsigned long RPM_COUNT_GetPeriodLength()
{
	return countset_ready ? countset_result : 0;
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

	// Enable de-bounce for the GPIO. It is a question if this timer is works for the
	// timer external capture pin also
	_GPIO_SET_DEBOUNCE_TIME(GPIO_DBNCECON_DBCLKSRC_HCLK, GPIO_DBNCECON_DBCLKSEL_512);

	DISPLAY_Init();
	// DISPLAY_Mode(DISPLAY_MODE_CAL);
	DISPLAY_Mode(DISPLAY_MODE_NORMAL);
    CONTROL_MODE = CONTROL_MODE_DUTY;
    display_rpm_set_state = 1;

	counter = 0;
	RPM_Clear();

	ENCODER_Init(ENCODER_Callback);

	Motor_Init();

    // duty_pwm = 0;
    GLOBAL_MOTOR_DUTY_CHANGED = 1;

    GLOBAL_MOTOR_RPMPRESET_CHANGED = 1;

    // ---- PWM Timer used as display update

    DisplayPWM_Init();

    SERIAL_Init();

    CONTROL_INIT();

    // PWM timer used for test signal
    // TEST_INIT();


    // Timer Rotational Speed Measurement
    RPM_Init();

    DISPLAY_CTRL_MODE(CONTROL_MODE);

    PID_Init(&MotorPWM);

    CONTROL_WORKER();

}
