/*==============================================================================
File: Controller.h
 
Description: This module encapsulates the control of a motor.  Initialze a 
  controller then re-compute the control output at a consistent rate.

Notes:
  - only handles positive inputs (if you have a desired speed take abs(yDesired)
    then negate the result)
  - PID control
  - the computation time (double-math) provides a lower limit on the loop rate
    
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef CONTROLLER_H
#define CONTROLLER_H
/*---------------------------Macros-------------------------------------------*/
#define MAX_NUM_CONTROLLERS   8 // change to support as many controllers 
                                // as needed

/*---------------------------Public Functions---------------------------------*/
/****************************************************************************
Function: InitController
Parameters:
  unsigned char controllerIndex, the index (0-based) of the controller 
                                 on which to operate
	double yMax,      the maximum value the output can produce
	double yMin,      the minimum value the output can produce
	double Kp,        proportional gain
	double Ki,        integral gain
	double Kd,        derivative gain
******************************************************************************/
void InitController(unsigned char controllerIndex, float yMax, float yMin, 
                    float Kp, float Ki, float Kd);


/****************************************************************************
Function: ComputeControlOutput
Returns:
	double,	the resulting value to command for the current iteration
Parameters:
  unsigned char controllerIndex, the index (0-based) of the controller
                                 on which to operate
	double yDesired,  the desired value of the output
	double yActual,   the current value of the output
******************************************************************************/
float ComputeControlOutput(unsigned char controllerIndex, 
                           float yDesired, float yActual);


/****************************************************************************
Function: ComputeBangBangOutput
Returns:
	double,	the resulting value to command for the current iteration
Parameters:
  unsigned char controllerIndex, the index (0-based) of the controller
                                 on which to operate
	double yDesired,  the desired value of the output
	double yActual,   the current value of the output
******************************************************************************/
float ComputeBangBangOutput(unsigned char controllerIndex, 
                            float yDesired, float yActual);

// TODO: add Autotune(&Kp, &Ki, &Kd);
#endif
