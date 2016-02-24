#define ENCODER_PIN_A          P02
#define ENCODER_PIN_B          P03
#define ENCODER_DEBOUNCE_DELAY	200

typedef void (* ENCODER_Callback_Type)(signed char cDirection);

extern void ENCODER_Init(ENCODER_Callback_Type CallBackFunc);
extern void ENCODER_Callback(signed char cDirection);
