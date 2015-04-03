// The PID controller
#include "pid.h"
#include "pwm.h"
#include "expwm.h"
#include "control.h"
#include "rpm_count.h"
#include "main.h"

pidtype PID_DATA;
expwmtype PID_PWM;

double pid_Limit(double value, double min, double max)
{
	if(value > max)
		return max;
	if(value < min)
		return min;
	return value;

}

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

void PID_Worker()
{
	unsigned long duty_prev;	// previous value of the duty cycle
	unsigned long duty_current;	// current value of the duty (required to the comparison,
								// double must be converted to unsigned long)
	// Check the control mode. The PID works only in RPM and AUTO mode,
	// in the normal operation (calibration not running)
	if(CONTROL_MODE != CONTROL_MODE_DUTY && CONTROL_FUNCTION == CONTROL_FUNCTION_NORMAL)
	{
		// Set the control information (the desired rotational speed)
		PID_DATA.Setpoint = CONTROL_RPM_SET;
		// Save the current rotational speed
		PID_DATA.Input = RPM_COUNT_GetPeriodLength();
		// Compute the values
		PID_Compute(&PID_DATA);
		// Save the current duty cycle
		duty_prev = EXPWM_GetDuty(PID_DATA.target_pwm);
		duty_current = PID_DATA.Output;
		EXPWM_SetDuty(PID_DATA.target_pwm, duty_current);
		GLOBAL_MOTOR_DUTY_CHANGED = (duty_prev != duty_current);
	}
}


void PID_Init(expwmtype* target_pwm)
{
	PID_DATA.kp = PID_KP;
	PID_DATA.ki = PID_KI;
	PID_DATA.kd = PID_KD;

	PID_DATA.outMin = PID_OUTPUT_MIN;
	PID_DATA.outMax = PID_OUTPUT_MAX;

	PID_DATA.target_pwm = target_pwm;

	PID_PWM.Chanell = 4;
	PID_PWM.Prescaler = 255;
	PID_PWM.Divider = PWM_CSR_DIV16;
	PID_PWM.Duty = 1249;
	PID_PWM.Period = 2499;
	PID_PWM.Port = EXPWM_PORT_DISABLE;
	PID_PWM.Callback = PID_Worker;
	EXPWM_Init(&PID_PWM);
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

