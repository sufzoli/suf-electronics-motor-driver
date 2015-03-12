#include "M051Series.h"
#include "sys.h"
#include "gpio.h"

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
//    SYSCLK->APBCLK = SYSCLK_APBCLK_PWM01_EN_Msk | SYSCLK_APBCLK_PWM23_EN_Msk | SYSCLK_APBCLK_TMR2_EN_Msk;
    /* IP clock source */
//    SYSCLK->CLKSEL1 = SYSCLK_CLKSEL1_PWM01_HCLK | SYSCLK_CLKSEL1_PWM23_HCLK | SYSCLK_CLKSEL1_TMR2_HCLK;
    /* IP clock source */
    // SYSCLK->CLKSEL2 = SYSCLK_CLKSEL2_PWM01_XTAL|SYSCLK_CLKSEL2_PWM23_XTAL;

    /* Reset PWMA channel0~channel3 */
//    SYS->IPRSTC2 = SYS_IPRSTC2_PWM03_RST_Msk;
//    SYS->IPRSTC2 = 0;

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
//    _GPIO_SET_PIN_MODE(P0, 2, GPIO_PMD_INPUT);
//    _GPIO_SET_PIN_MODE(P0, 3, GPIO_PMD_INPUT);
    /*
    _GPIO_ENABLE_DEBOUNCE(P0, 2);
    _GPIO_ENABLE_DEBOUNCE(P0, 3);
    _GPIO_SET_DEBOUNCE_TIME(GPIO_DBNCECON_DBCLKSRC_HCLK, GPIO_DBNCECON_DBCLKSEL_16);
    */
//    GPIO_EnableInt(P0, 2, GPIO_INT_RISING);
//    GPIO_EnableInt(P0, 3, GPIO_INT_RISING);

//    NVIC_EnableIRQ(GPIO_P0P1_IRQn);
//    NVIC_EnableIRQ(TMR2_IRQn);
//    NVIC_EnableIRQ(PWMA_IRQn);

    // Test Output
//    _GPIO_SET_PIN_MODE(P0, 4, GPIO_PMD_OUTPUT);


/*---------------------------------------------------------------------------------------------------------*/
/* Init I/O Multi-function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD and TXD  */
    // SYS->P3_MFP = SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0;
    /* Set P2 multi-function pins for PWMB Channel0~3  */
//    SYS->P2_MFP = SYS_MFP_P20_PWM0;
//    SYS->P4_MFP = SYS_MFP_P40_T2EX;
    /* Lock protected registers */
    SYS_LockReg();
}



int main(void)
{
	HD44780_Init();
	HD44780_CLS();
	SYS_SysTickDelay(2000000);
	HD44780_DisplayString("Hello World!");

    while(1)
    {
/*
    	P04 = 0;
    	SYS_SysTickDelay(500);
    	P04 = 1;
    	SYS_SysTickDelay(500);
*/
    }
}
