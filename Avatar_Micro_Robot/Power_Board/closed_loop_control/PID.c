/*==============================================================================
File: PID.c
Notes:
  - You can usually just set the integrator minimum and maximum as the drive
    maximum and minimum.  If you know your disturbances are small and you
    want quicker settlines, you can limit the integrator further.

See also:
  - control system block diagram
    
Inpired By:
  - "PID without a PhD" by Tim Wescott
  - http://brettbeauregard.com/
  - http://www.cds.caltech.edu/~murray/courses/cds101/fa04/caltech/am04_ch8-3nov04.pdf
  - friends and colleagues
==============================================================================*/
//---------------------------Dependencies---------------------------------------
#include "PID.h"
#include "stdhdr.h"

//---------------------------Macros and Definitions-----------------------------
typedef struct {
    float y_max; // the maximum value the output can produce
    float y_min; // the minimum value the output can produce
    float Kp;    // proportional gain
    float Ki;    // integral gain
    float Kd;    // derivative gain
} controller_t;

//---------------------------Module Variables-----------------------------------
static controller_t controllers[MAX_NUM_CONTROLLERS];
static float integral_terms[MAX_NUM_CONTROLLERS] = {0};
static float y_actual_lasts[MAX_NUM_CONTROLLERS] = {0};

//---------------------------Public Function Definitions------------------------
void PID_Init(const uint8_t i, const float y_max, const float y_min, const float Kp, const float Ki,
              const float Kd) {
    // populate the fields that comprise a controller
    controllers[i].y_max = y_max;
    controllers[i].y_min = y_min;
    controllers[i].Kp = Kp;
    controllers[i].Ki = Ki;
    controllers[i].Kd = Kd;

    /*
    // TODO: initialize for "bumpless transfer"
    y_last = y_actual;
    integral_term = y_command;
    if (y_max < integral_term) integral_term = y_max;
    else if (integral_term < y_min) integral_term = y_min;

    return y_command;
    */
}

float PID_ComputeEffort(const uint8_t i, const float y_desired, const float y_actual,
                        const float y_nominal) {
    float error, delta_Y, y_command = 0;

    error = y_desired - y_actual_lasts[i];
    integral_terms[i] += (controllers[i].Ki * error);
    delta_Y = (y_actual - y_actual_lasts[i]);

    // limit the integral term independently (see Notes section)
    if (controllers[i].y_max < integral_terms[i])
        integral_terms[i] = controllers[i].y_max;
    else if (integral_terms[i] < controllers[i].y_min)
        integral_terms[i] = controllers[i].y_min;

    // compute the PID Output
    y_command =
        (controllers[i].Kp * error) + integral_terms[i] - (controllers[i].Kd * delta_Y) + y_nominal;

    // if we've saturated, remove the current error term from
    // the integral term to prevent integrator windup
    if (controllers[i].y_max < y_command) {
        y_command = controllers[i].y_max;
        integral_terms[i] -= (controllers[i].Ki * error);
    } else if (y_command < controllers[i].y_min) {
        y_command = controllers[i].y_min;
        integral_terms[i] -= (controllers[i].Ki * error);
    }

    // BUG ALERT: ensure the output never goes the opposite of the intended direction
    if ((0 < y_desired) && (y_command < 0))
        y_command = 0;
    else if ((y_desired < 0) && (0 < y_command))
        y_command = 0;

    y_actual_lasts[i] = y_actual;
    return y_command;
}

void PID_Reset(const uint8_t i) {
    y_actual_lasts[i] = 0;
    integral_terms[i] = 0;
}

void PID_Reset_Integral(const uint8_t i) {
    integral_terms[i] = 0;

    // if the controller is going too fast, cut the speed so that robot will stop quicker
    // May not want to put this in, as it could decrease the deceleration
    /*if(y_actual_lasts[i] > 250)
      y_actual_lasts[i] = 250
    else if(y_actual_lasts[i] < -250)
      y_actual_lasts[i] = -250*/
}
