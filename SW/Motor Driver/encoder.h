#include "macro_helper.h"

#define ENCODER_A_PORT_NUM 0
#define ENCODER_B_PORT_NUM 0

#define ENCODER_A_PIN_NUM 3
#define ENCODER_B_PIN_NUM 2

#define ENCODER_A_PORT MACRO_CONCAT2(P,ENCODER_A_PORT_NUM)
#define ENCODER_B_PORT MACRO_CONCAT2(P,ENCODER_B_PORT_NUM)

#define ENCODER_PIN_A MACRO_CONCAT3(P,ENCODER_A_PORT_NUM, ENCODER_A_PIN_NUM)
#define ENCODER_PIN_B MACRO_CONCAT3(P,ENCODER_B_PORT_NUM, ENCODER_B_PIN_NUM)
#define ENCODER_DEBOUNCE_DELAY	200

typedef void (* ENCODER_Callback_Type)(signed char cDirection);

extern void ENCODER_Init(ENCODER_Callback_Type CallBackFunc);
extern void ENCODER_Callback(signed char cDirection);
