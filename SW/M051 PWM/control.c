#include "gpio.h"
#include "control.h"
#include "display.h"
#include "motor.h"

unsigned char CONTROL_FUNCTION=CONTROL_FUNCTION_NORMAL;
unsigned char CONTROL_MODE=CONTROL_MODE_AUTO;
unsigned long CONTROL_RPM_SET=0;

void CONTROL_INIT()
{
    _GPIO_SET_PIN_MODE(P0, 4, GPIO_PMD_INPUT);
    _GPIO_SET_PIN_MODE(P0, 5, GPIO_PMD_INPUT);
    _GPIO_SET_PIN_MODE(P0, 6, GPIO_PMD_INPUT);

    _GPIO_ENABLE_DEBOUNCE(P0, 4);
    _GPIO_ENABLE_DEBOUNCE(P0, 5);
    _GPIO_ENABLE_DEBOUNCE(P0, 6);
    _GPIO_SET_DEBOUNCE_TIME(GPIO_DBNCECON_DBCLKSRC_HCLK, GPIO_DBNCECON_DBCLKSEL_32768);

}

void CONTROL_WORKER()
{
	while(1)
	{
		if(CONTROL_FUNCTION == CONTROL_FUNCTION_NORMAL)
		{
			if(P06)
			{
				CONTROL_FUNCTION = CONTROL_FUNCTION_CAL;
				CALIBRATION_START();
				CONTROL_FUNCTION = CONTROL_FUNCTION_NORMAL;
			}
			if((P05 <<1) | P04 != CONTROL_MODE)
			{
				switch((P05 <<1) | P04)
				{
					case 0:
						CONTROL_MODE = CONTROL_MODE_DUTY;
						break;
					case 1:
						CONTROL_MODE = CONTROL_MODE_AUTO;
						break;
					case 2:
						CONTROL_MODE = CONTROL_MODE_RPM;
						break;
					default:
						CONTROL_MODE = CONTROL_MODE_AUTO;
						break;
				}
				DISPLAY_CTRL_MODE(CONTROL_MODE);
				Motor_GracefullStop();
			}
		}
	}
}
