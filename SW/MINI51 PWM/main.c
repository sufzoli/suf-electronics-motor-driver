
/*
 * Port configuration
 *
 * P0.0 - Current Measure ADC (40)
 * P0.1 - Voltage Measure ADC (41)
 * P0.2 - Rotary Encoder A (38)
 * P0.3 - Rotary Encoder B (37)
 *
 * P2.0 - Motor Drive PWM (19)
 * P2.1 - Motor Rotation Measure (20) - PWM Capture
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
 * P4.4 - I2C SCL (28)
 * P4.5 - I2C SDA (29)
 *
 */




#include "M051Series.h"
#include "sys.h"
#include "pwm.h"
#include "gpio.h"

#include "display.h"

#include "hd44780.h"

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
    SYSCLK->APBCLK = SYSCLK_APBCLK_PWM01_EN_Msk;
    /* IP clock source */
    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_PWM01_HCLK;
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
    _GPIO_ENABLE_DEBOUNCE(P0, 2);
    _GPIO_ENABLE_DEBOUNCE(P0, 3);
    _GPIO_SET_DEBOUNCE_TIME(GPIO_DBNCECON_DBCLKSRC_IRC10K, GPIO_DBNCECON_DBCLKSEL_4);



/*---------------------------------------------------------------------------------------------------------*/
/* Init I/O Multi-function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD  */
    // SYS->P3_MFP = SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0;
    /* Set P2 multi-function pins for PWMB Channel0~3  */
    SYS->P2_MFP = SYS_MFP_P20_PWM0;
    /* Lock protected registers */
    SYS_LockReg();
}



int main(void)
{
	SYS_Init();

//	DISPLAY_Init();

//	HD44780_Init();
//	HD44780_DisplayString("Hello World!");
	DISPLAY_Init();
	DISPLAY_RPM(10000);

    /*Set Pwm mode*/
    _PWM_SET_TIMER_AUTO_RELOAD_MODE(PWMA,PWM_CH0);

    /*Set PWM Timer clock prescaler*/
    _PWM_SET_TIMER_PRESCALE(PWMA,PWM_CH0, 1); // Divided by 2

    /*Set PWM Timer clock divider select*/
    _PWM_SET_TIMER_CLOCK_DIV(PWMA,PWM_CH0,PWM_CSR_DIV1);

    /*Set PWM Timer duty*/
    PWMA->CMR0 = 125;

    /*Set PWM Timer period*/
    PWMA->CNR0 = 250;

    /* Enable PWM Output pin */
    _PWM_ENABLE_PWM_OUT(PWMA, PWM_CH0);

    /* Enable Timer period Interrupt */
//    _PWM_ENABLE_TIMER_PERIOD_INT(PWMB, PWM_CH0);

    /* Enable PWMB NVIC */
//    NVIC_EnableIRQ((IRQn_Type)(PWMB_IRQn));

    /* Enable PWM Timer */
    _PWM_ENABLE_TIMER(PWMA, PWM_CH0);

    while(1)
    {
    }
}
