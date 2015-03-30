#ifndef __EXPWM_H__
#define __EXPWM_H__



typedef void (* EXPWM_InterruptCallback_Type)();

typedef struct
{
	unsigned char Chanell;
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

/*
#define SYS_MFP_P20_PWM0    0x00000100UL
#define SYS_MFP_P21_PWM1    0x00000200UL
#define SYS_MFP_P22_PWM2    0x00000400UL
#define SYS_MFP_P23_PWM3    0x00000800UL

#define SYS_MFP_P24_PWM4    0x00001000UL
#define SYS_MFP_P25_PWM5    0x00002000UL
#define SYS_MFP_P26_PWM6    0x00004000UL
#define SYS_MFP_P27_PWM7    0x00008000UL

#define SYS_MFP_P40_PWM0    0x00000001UL
#define SYS_MFP_P41_PWM1    0x00000002UL
#define SYS_MFP_P42_PWM2    0x00000004UL
#define SYS_MFP_P43_PWM3    0x00000008UL  */

extern void EXPWM_Init(expwmtype *data);
extern void EXPWM_SetDuty(expwmtype *data, unsigned int duty);

#endif
