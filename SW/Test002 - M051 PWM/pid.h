
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
} pidtype;

