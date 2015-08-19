#ifndef __EXPWM_H__
#define __EXPWM_H__

typedef void (* EXPWM_InterruptCallback_Type)();

typedef struct
{
	unsigned char Channel;
	unsigned char Prescaler;
	unsigned char Divider;
	unsigned long Duty;
	unsigned long Period;
	unsigned char Port;
	EXPWM_InterruptCallback_Type Callback;
} expwmtype;

#define EXPWM_PORT_DISABLE	0
#define EXPWM_PORT2	1
#define EXPWM_PORT4	2

extern void EXPWM_Init(expwmtype *data);
extern void EXPWM_SetDuty(expwmtype *data, unsigned int duty);
extern unsigned long EXPWM_GetDuty(expwmtype *data);

#endif
