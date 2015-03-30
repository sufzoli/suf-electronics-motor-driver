#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "expwm.h"

extern expwmtype MotorPWM;

#define Motor_SetDuty(x) EXPWM_SetDuty(&MotorPWM, x)

extern void Motor_Init();

#endif
