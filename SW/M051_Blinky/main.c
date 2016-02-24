#include "sys.h"
#include "gpio.h"

#define PLLCON_SETTING      SYSCLK_PLLCON_50MHz_XTAL
#define PLL_CLOCK           50000000

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

    // Update System Core Clock
    // User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically.
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    // Lock protected registers
    SYS_LockReg();
}




int main(void)
{
	SYS_Init();
	// Setup GPIO
    _GPIO_SET_PIN_MODE(P3, 0, GPIO_PMD_OUTPUT);
    while(1)
    {
    	P30 = 0;
    	SYS_SysTickDelay(5000);
    	P30 = 1;
    	SYS_SysTickDelay(5000);
    }
}
