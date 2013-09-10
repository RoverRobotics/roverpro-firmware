/*==============================================================================
File: Controller.h
 
Description: This module encapsulates a PID controller with nominal output.  
  Initialze the controller with the appropriate constants and lookup table, then
	re-compute the control output at a consistent rate.

Notes:
  - WARNING: assumes a direct-acting process -- that is, an increase in the
    output causes an increase in the input.  It is up to the user to make
    the signs of Kp, Ki and Kd negative if the process is reverse-acting.
  - If employing differential control, be wary of noise and high-frequency
    oscillations.
    
Tuning Considerations.
  - differential gain is usually high
  - integral gain is usually low
  - if you can't stabilize with P, you can't stabilize with PI
  - unless you're working on a project with very critical performance paraters,
    you can often get by with control gains that are within a factor of two of 
    the "correct" value

Sampling Rate Considerations.
  - Sample Rate Tolerance.  At WORST your sampling rate should vary by no more 
    than +/20% over any 10-sample interval.  For a PI-controller, it is 
    preferable to have EACH sample fall within +/-1% to +/-5% of the correct 
    sample time
  - rule of thumb: the sample time should be between 1/10th and 1/100th of the
    desired system settling time, where 'system setting time' is defined as the
    amount of time from the moment the drive comes out of saturation until the 
    control system has effectively settled out.

// TODO: implement an auto-tuner: Autotune(&Kp, &Ki, &Kd);
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
/*---------------------------Macros-------------------------------------------*/
#define MAX_NUM_CONTROLLERS   8 // change to support as many controllers 
                                // as needed (used to obviate the need for 
																// dynamic memory management)
/*---------------------------Public Functions---------------------------------*/
/****************************************************************************
Function: InitPIDController
Parameters:
  unsigned char controllerIndex, the index (0-based) of the controller 
                                 on which to operate
	float yMax,      the maximum value the output can produce
	float yMin,      the minimum value the output can produce
	float Kp,        proportional gain
	float Ki,        integral gain
	float Kd,        differential gain
******************************************************************************/
void InitPIDController(const unsigned char controllerIndex,
                       const float yMax, const float yMin, 
                       float Kp, float Ki, float Kd);


/****************************************************************************
Function: ComputeControlOutput
Returns:
	float,	the resulting value to command for the current iteration
Parameters:
  unsigned char controllerIndex, the index (0-based) of the controller
                                 on which to operate
	float yDesired,  	the desired output value
	float yActual,   	the current, actual output value
	float yNominal,		the nominal, steady-state output for the desired output.
	                  input 0 if nominal offset is NOT desired.
******************************************************************************/
float ComputeControlOutput(const unsigned char controllerIndex,
                           const float yDesired,
													 const float yActual,
													 const float yNominal);

#endif
