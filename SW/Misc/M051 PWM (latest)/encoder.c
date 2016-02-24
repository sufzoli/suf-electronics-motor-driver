#include "main.h"
#include "control.h"
#include "motor.h"
#include "sys.h"
#include "GPIO.h"
#include "encoder.h"

const signed char ENCODER_TABLE[] = {0,-1,+1,0,+1,0,0,-1,-1,0,0,+1,0,+1,-1,0};
static unsigned char ENCODER_PREV_STATE;
ENCODER_Callback_Type ENCODER_USER_Callback;


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
			GLOBAL_MOTOR_RPMPRESET_CHANGED = 1;
		}
	}
}
