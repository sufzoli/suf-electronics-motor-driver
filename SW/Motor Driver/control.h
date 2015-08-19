#include "macro_helper.h"

#define CONTROL_CAL_PORT_NUM 2
#define CONTROL_CAL_PIN_NUM 4

#define CONTROL_MODE_RPM_PORT_NUM 4
#define CONTROL_MODE_RPM_PIN_NUM 2

#define CONTROL_MODE_AUTO_PORT_NUM 4
#define CONTROL_MODE_AUTO_PIN_NUM 3

#define CONTROL_CAL_PORT MACRO_CONCAT2(P,CONTROL_CAL_PORT_NUM)
#define CONTROL_MODE_RPM_PORT MACRO_CONCAT2(P,CONTROL_MODE_RPM_PORT_NUM)
#define CONTROL_MODE_AUTO_PORT MACRO_CONCAT2(P,CONTROL_MODE_AUTO_PORT_NUM)

#define CONTROL_CAL_PIN MACRO_CONCAT3(P,CONTROL_CAL_PORT_NUM,CONTROL_CAL_PIN_NUM)
#define CONTROL_MODE_RPM_PIN MACRO_CONCAT3(P,CONTROL_MODE_RPM_PORT_NUM,CONTROL_MODE_RPM_PIN_NUM)
#define CONTROL_MODE_AUTO_PIN MACRO_CONCAT3(P,CONTROL_MODE_AUTO_PORT_NUM,CONTROL_MODE_AUTO_PIN_NUM)


#define CONTROL_FUNCTION_NORMAL	0
#define CONTROL_FUNCTION_CAL	0

#define CONTROL_MODE_AUTO	2
#define CONTROL_MODE_DUTY	0
#define CONTROL_MODE_RPM	1

extern unsigned char CONTROL_FUNCTION;
extern unsigned char CONTROL_MODE;
extern unsigned long CONTROL_RPM_SET;

extern void CONTROL_INIT();
extern void CONTROL_WORKER();
