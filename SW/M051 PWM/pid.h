#include "expwm.h"

#define PID_KP 1
#define PID_KI 0
#define PID_KD 0

#define PID_OUTPUT_MIN 0
#define PID_OUTPUT_MAX 250



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

