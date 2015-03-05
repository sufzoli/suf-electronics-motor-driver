#define ENCODER_PIN_A          P02
#define ENCODER_PIN_B          P03
#define ENCODER_DEBOUNCE_DELAY	200


#define xGPIOSPinIntCallbackInit(eShortPin, xtPortCallback)	\
		GPIOSPinIntCallbackInit(eShortPin, xtPortCallback)
#define GPIOSPinIntCallbackInit(eShortPin, xtPortCallback)                                       \
        GPIOPinIntCallbackInit(G##eShortPin, xtPortCallback)

typedef void (* ENCODER_Callback_Type)(signed char cDirection);

void ENCODER_Init(ENCODER_Callback_Type CallBackFunc);
