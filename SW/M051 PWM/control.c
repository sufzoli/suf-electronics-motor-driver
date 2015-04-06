#include "sys.h"
#include "gpio.h"
#include "control.h"
#include "display.h"
#include "motor.h"
#include "main.h"

unsigned char CONTROL_FUNCTION=CONTROL_FUNCTION_NORMAL;
unsigned char CONTROL_MODE=CONTROL_MODE_AUTO;
unsigned long CONTROL_RPM_SET=0;

void CONTROL_INIT()
{
    _GPIO_SET_PIN_MODE(P0, 4, GPIO_PMD_QUASI);
    _GPIO_SET_PIN_MODE(P0, 5, GPIO_PMD_QUASI);
    _GPIO_SET_PIN_MODE(P0, 6, GPIO_PMD_QUASI);

/*
    _GPIO_SET_PIN_MODE(P0, 4, GPIO_PMD_INPUT);
    _GPIO_SET_PIN_MODE(P0, 5, GPIO_PMD_INPUT);
    _GPIO_SET_PIN_MODE(P0, 6, GPIO_PMD_INPUT);
*/


    _GPIO_ENABLE_DEBOUNCE(P0, 4);
    _GPIO_ENABLE_DEBOUNCE(P0, 5);
    _GPIO_ENABLE_DEBOUNCE(P0, 6);
//    _GPIO_SET_DEBOUNCE_TIME(GPIO_DBNCECON_DBCLKSRC_HCLK, GPIO_DBNCECON_DBCLKSEL_32768);

}

void CONTROL_WORKER()
{
	char ctrl_read;
	while(1)
	{
		ctrl_read = (P05 ? 2 : 0) | (P04 ? 1 : 0);
		if(CONTROL_FUNCTION == CONTROL_FUNCTION_NORMAL)
		{
			if(P06)
			{
				CONTROL_FUNCTION = CONTROL_FUNCTION_CAL;
				CALIBRATION_START();
				CONTROL_FUNCTION = CONTROL_FUNCTION_NORMAL;
			}
			if(ctrl_read != CONTROL_MODE)
			{
				CONTROL_RPM_SET = 0;
				GLOBAL_MOTOR_RPMPRESET_CHANGED = 1;
				CONTROL_MODE = ctrl_read < 3 ? ctrl_read : CONTROL_MODE_AUTO;
/*
				switch(ctrl_read)
				{
					case CONTROL_MODE_DUTY:
						CONTROL_MODE = CONTROL_MODE_DUTY;
						break;
					case CONTROL_MODE_AUTO:
						CONTROL_MODE = CONTROL_MODE_AUTO;
						break;
					case CONTROL_MODE_RPM:
						CONTROL_MODE = CONTROL_MODE_RPM;
						break;
					default:
						CONTROL_MODE = CONTROL_MODE_AUTO;
						break;
				}
*/
				DISPLAY_CTRL_MODE(CONTROL_MODE);
//				Motor_GracefullStop();
			}
		}
		SYS_SysTickDelay(1000);
	}
}
