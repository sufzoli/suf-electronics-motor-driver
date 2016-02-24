#include "expwm.h"

#define PID_KP 0.1
#define PID_KI 0
#define PID_KD 0

#define PID_OUTPUT_MIN 0
#define PID_OUTPUT_MAX 249



typedef void (* PID_SetOutputCallback_Type)(double output);
typedef double (* PID_GetInputCallback_Type)();


typedef struct
{
	double kp;
	double ki;
	double kd;
	double Input;
	double Output;
	double Setpoint;

	double IAgregate;
	double PervInput;
	double PrevOutput;
	double outMin;
	double outMax;
	unsigned char Direction;
	expwmtype* target_pwm;

} pidtype;

extern void PID_Init(expwmtype* target_pwm);
