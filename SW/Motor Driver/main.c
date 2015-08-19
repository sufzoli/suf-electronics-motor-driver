
/*
 * System port configuration
 *
 * P0.0 - UART TXD (40)
 * P0.1 - UART RXD (39)
 * P0.2 - Rotary Encoder A (38)
 * P0.3 - Rotary Encoder B (37)
 * P0.4
 * P0.5
 * P0.6
 * P0.7
 *
 * P1.0
 * P1.1
 * P1.2
 * P1.3
 * P1.4
 * P1.5
 * P1.6
 * P1.7
 *
 * P2.0 - TFT D/C (19)
 * P2.1 - TFT Reset (20)
 *
 * P4.0 - Motor Rotation Measure (24) - TMR2 Capture
 *
 *
 */


// Headers
#include "main.h"
#include "M051Series.h"
#include "sys.h"
#include "gpio.h"

#include "pwm.h"

#include "display.h"
#include "control.h"

#include "expwm.h"
#include "encoder.h"

#include "test_gen.h"

// #include "ili9341.h"
// #include "ili9341_fonts.h"

// Global variables
volatile unsigned char GLOBAL_MOTOR_DUTY_CHANGED;	// set to true if somewhere the motor pwm duty cycle changed
volatile unsigned char GLOBAL_MOTOR_RPMPRESET_CHANGED; // set to true if somewhere the motor rpm preset changed



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

    /* Commented out here, if it works without system lock, must be removed permanently
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
    NVIC_SetPriority(TMR2_IRQn,0);			// Measurement timer for the motor rotation
    NVIC_SetPriority(SysTick_IRQn,1);
    NVIC_SetPriority(GPIO_P0P1_IRQn,2);		// GPIO Interrupt used by the rotary encoder
    NVIC_SetPriority(PWMA_IRQn,3);			// ????

    // Lock protected registers
    SYS_LockReg();
}

void DisplayPWM_Callback(void)
{
	unsigned long disp_count;
	unsigned long disp_buff;
	// _PWM_CLEAR_TIMER_PERIOD_INT_FLAG(PWMA,PWM_CH3);

	/*
	if(countset_ready && (countset_result > 0))
	{
		disp_count = 3000000000u / countset_result;
	}
	else
	{
		disp_count = 0;
	}
	DISPLAY_RPM(disp_count);
	*/
	if(GLOBAL_MOTOR_DUTY_CHANGED)
	{
		disp_buff = Motor_GetDuty();
		DISPLAY_DUTY(disp_buff);
		GLOBAL_MOTOR_DUTY_CHANGED = 0;
	}
	/*
	if(GLOBAL_MOTOR_RPMPRESET_CHANGED && CONTROL_MODE != CONTROL_MODE_DUTY)
	{
		disp_buff = CONTROL_RPM_SET;
		DISPLAY_RPM_SET(disp_buff);
		GLOBAL_MOTOR_RPMPRESET_CHANGED = 0;
	}
	*/
}

void DisplayPWM_Init()
{
	DisplayPWM.Channel = 4;
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
	// Initialize clock and interrupt system
	SYS_Init();
	// Enable de-bounce counter. It will be used by the GPIO and Timer inputs
	_GPIO_SET_DEBOUNCE_TIME(GPIO_DBNCECON_DBCLKSRC_HCLK, GPIO_DBNCECON_DBCLKSEL_512);

	// Initialize the display
	DISPLAY_Init();

	// Set display mode to normal. Calibration not running
	DISPLAY_Mode(DISPLAY_MODE_NORMAL);

	// Set control mode to duty cycle (may require to changed when the controls in place)
	CONTROL_MODE = CONTROL_MODE_DUTY;

	// Clear the rpm counter
	RPM_Clear();

	ENCODER_Init(ENCODER_Callback);

	Motor_Init();

	// Used to enforce displaying the initial data on the display
    GLOBAL_MOTOR_DUTY_CHANGED = 1;
    GLOBAL_MOTOR_RPMPRESET_CHANGED = 1;

	DisplayPWM_Init();

	TEST_INIT();

	// Left commented out until the serial communication setup
	// SERIAL_Init();

	CONTROL_INIT();

    // Timer Rotational Speed Measurement
    RPM_Init();

    // Write to the screen the current control mode
    DISPLAY_CTRL_MODE(CONTROL_MODE);

	CONTROL_WORKER();

/*
	ILI9341_Init();
	ILI9341_FillScreen();
	ILI9341_PrintStr(&Font16x26, "Hello World!", 5, 5, ILI9341_COLOR_WHITE, ILI9341_COLOR_BLACK);
*/

    while(1)
    {
    }
}
