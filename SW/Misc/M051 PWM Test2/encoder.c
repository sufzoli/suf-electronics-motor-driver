#include "GPIO.h"
#include "encoder.h"

const signed char ENCODER_TABLE[] = {0,-1,+1,0,+1,0,0,-1,-1,0,0,+1,0,+1,-1,0};
static unsigned char ENCODER_PREV_STATE;
ENCODER_Callback_Type ENCODER_USER_Callback;


unsigned long ENCODER_INT_Callback(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgParam, void *pvMsgData)
{
	// unsigned char encoderbits;
	xSysCtlDelay(ENCODER_DEBOUNCE_DELAY);
    xGPIOSPinIntClear(ENCODER_PIN_A);
    xGPIOSPinIntClear(ENCODER_PIN_B);

    ENCODER_PREV_STATE = (xGPIOSPinRead(ENCODER_PIN_B) << 1) | xGPIOSPinRead(ENCODER_PIN_A) | (ENCODER_PREV_STATE << 2);

    // encoder_count += ENCODER_TABLE[(ENCODER_PREV_STATE & 0x0f)];  /* Index into table */

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

    return 0;
}

void ENCODER_Init(ENCODER_Callback_Type CallBackFunc)
{

	/*
	xGPIOSPinTypeGPIOInput(ENCODER_PIN_A);
	xGPIOSPinTypeGPIOInput(ENCODER_PIN_B);
	ENCODER_USER_Callback = CallBackFunc;
 	xGPIOSPinIntCallbackInit(ENCODER_PIN_A, ENCODER_INT_Callback);
 	xGPIOSPinIntCallbackInit(ENCODER_PIN_B, ENCODER_INT_Callback);
	ENCODER_PREV_STATE = (xGPIOSPinRead(ENCODER_PIN_B) << 1) | xGPIOSPinRead(ENCODER_PIN_A);
	xGPIOSPinIntEnable(ENCODER_PIN_A,xGPIO_RISING_EDGE);
	xGPIOSPinIntEnable(ENCODER_PIN_B,xGPIO_RISING_EDGE);
	xIntEnable( xINT_GPIOA );
	*/
}
