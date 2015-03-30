
#define CONTROL_FUNCTION_NORMAL	0
#define CONTROL_FUNCTION_CAL	0

#define CONTROL_MODE_AUTO	2
#define CONTROL_MODE_DUTY	0
#define CONTROL_MODE_RPM	1

extern unsigned char CONTROL_MODE;
extern unsigned long CONTROL_RPM_SET;

extern void CONTROL_INIT();
extern void CONTROL_WORKER();
