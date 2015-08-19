
// System clock set to 50MHz PLL
#define PLLCON_SETTING      SYSCLK_PLLCON_50MHz_XTAL
#define PLL_CLOCK           50000000


// Global variables
extern volatile unsigned char GLOBAL_MOTOR_DUTY_CHANGED;
extern volatile unsigned char GLOBAL_MOTOR_RPMPRESET_CHANGED;
