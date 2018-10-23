/*==============================================================================
File: PID.h

Description: This module encapsulates a PID controller with nominal offset.
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
#ifndef PID_H
#define PID_H
//---------------------------Dependencies---------------------------------------
#include <stdint.h> // for uintN_t data types

//---------------------------Macros---------------------------------------------
#define MAX_NUM_CONTROLLERS 8 // change to support as many controllers
// as needed (used to obviate the need for
// dynamic memory management)
//---------------------------Public Functions-----------------------------------
// Function: PID_Init
// Parameters:
//   uint8_t controller_index, the index (0-based) of the controller
//                             on which to operate
// 	float y_max,     the maximum value the output can produce
// 	float y_min,     the minimum value the output can produce
// 	float Kp,        proportional gain
// 	float Ki,        integral gain
// 	float Kd,        differential gain
void PID_Init(uint8_t controller_index, float y_max, float y_min, float Kp, float Ki, float Kd);

// Function: PID_ComputeEffort
// Returns:
// 	 float,	the resulting value to command for the current iteration
// Parameters:
//   uint8_t controller_index, the index (0-based) of the controller
//                             on which to operate
// 	float y_desired,  the desired output value
// 	float y_actual,   the current, actual output value
// 	float x_nominal,	the nominal effort to acheive the desired output
// 	                  pass zero (0) if nominal offset is NOT desired.
// 	bool should_reset whether the controller should 'forget' its history
float PID_ComputeEffort(uint8_t controller_index, float y_desired, float y_actual, float x_nominal);

// Function: PID_Reset
// Parameters:
//   uint8_t controller_index, the index (0-based) of the controller
//                             on which to operate
void PID_Reset(uint8_t controller_index);

void PID_Reset_Integral(uint8_t controller_index);

#endif
