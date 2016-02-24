// The PID controller
#include "pid.h"


double pid_Limit(double value, double min, double max)
{
	if(value > max)
		return max;
	if(value < min)
		return min;
	return value;

}

void PID_Init(pidtype *PID, PID_GetInputCallback_Type input_delegate, PID_SetOutputCallback_Type output_delegate)
{
	// setup the PWM Channel
	// test if the pwm block initialized
	// get the clock of the pwm block, or initialize
	// add a callback to the pwm interrupt (create a pwm module what able to distinguish between the various callbacks

}

// PID callback for the pwm.
// call input callback
// compute the pid parameters
// call the output callback

// how travell the PID datastructure trough?
// store it in an array when the init called?
// store by the PWM only somehow? create an 8 element (one for each pwm channel) pointer array inside the
// pwm lib to be able to transfer it to the callbacks?
// handle this at the callback functions in a global variable? --- This looks good.

void PID_Compute(pidtype *PID)
{
	double error;
	double diffInput;

	// Compute all the working error variables
	error = PID->Setpoint - PID->Input;
	PID->IAgregate += PID->ki * error;
	PID->IAgregate = pid_Limit(PID->IAgregate, PID->outMin, PID->outMax);
	diffInput = PID->Input - PID->PervInput;

	//Compute PID Output
	PID->Output = PID->kp * error + PID->IAgregate + PID->kd * diffInput;
	PID->Output = pid_Limit(PID->Output, PID->outMin, PID->outMax);

	// Remember some variables for next time
	PID->PervInput = PID->Input;	// used for the next Difference calculation
	PID->PrevOutput = PID->Output;  // used for the maximum change limiting
}
