#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "expwm.h"

// If the motor needed to stop, start, or change the rotational speed
// gracefully. This time (in ms) is the minimum time between the change in the PWM steps
// the default 40ms with the 250 PWM steps means full stop from the maximum speed in 10s
#define MOTOR_GRACEFULL_STEPTIME	40

extern expwmtype MotorPWM;

#define Motor_SetDuty(x) EXPWM_SetDuty(&MotorPWM, x)

extern void Motor_Init();
extern void Motor_GracefullStop();

#endif
