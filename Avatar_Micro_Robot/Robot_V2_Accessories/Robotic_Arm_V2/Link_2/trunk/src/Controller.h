/*==============================================================================
File: Controller.h
 
Description: This module encapsulates PID control.  Initialze the controller 
  with the appropriate constants then re-compute the control output at a 
  consistent rate.

Notes:
  - WARNING: assumes a direct-acting process -- that is, an increase in the
    output causes an increase in the input.  It is up to the user to make
    the signs of Kp, Ki and Kd all negative if the process is reverse-acting.
  - WARNING: assumes that all associated numbers are positive or zero
    (just take abs() before you give a parameter)
    
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef CONTROLLER_H
#define CONTROLLER_H
/*---------------------------Macros-------------------------------------------*/
#define MAX_NUM_CONTROLLERS   8 // change to support as many controllers 
                                // as needed (used to avoid need for dynamic
                                // memory management)

/*---------------------------Public Functions---------------------------------*/
/****************************************************************************
Function: InitController
Parameters:
  unsigned char controllerIndex, the index (0-based) of the controller 
                                 on which to operate
	float yMax,      the maximum value the output can produce
	float yMin,      the minimum value the output can produce
	float Kp,        proportional gain
	float Ki,        integral gain
	float Kd,        derivative gain
******************************************************************************/
void InitController(unsigned char controllerIndex, float yMax, float yMin, 
                    float Kp, float Ki, float Kd);


/****************************************************************************
Function: ComputeControlOutput
Returns:
	float,	the resulting value to command for the current iteration
Parameters:
  unsigned char controllerIndex, the index (0-based) of the controller
                                 on which to operate
	float yDesired,  the desired value of the output
	float yActual,   the current value of the output
******************************************************************************/
float ComputeControlOutput(unsigned char controllerIndex, 
                           float yDesired, float yActual);

// TODO: add Autotune(&Kp, &Ki, &Kd);
#endif
