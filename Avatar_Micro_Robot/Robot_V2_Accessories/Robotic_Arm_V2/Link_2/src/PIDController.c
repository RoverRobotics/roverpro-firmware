/*==============================================================================
File: PIDController.c
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
/*---------------------------Dependencies-------------------------------------*/
#include "./PIDController.h"

/*---------------------------Macros and Definitions---------------------------*/
typedef struct {
  float yMax;		// the maximum value the output can produce
  float yMin;   // the minimum value the output can produce
	float Kp;    	// proportional gain
	float Ki;    	// integral gain
  float Kd;    	// derivative gain
} controller_t;
/*---------------------------Module Variables---------------------------------*/
static controller_t controllers[MAX_NUM_CONTROLLERS];

/*---------------------------Public Function Definitions----------------------*/
void InitPIDController(unsigned char i, float yMax, float yMin, 
                       float Kp, float Ki, float Kd) {	
	// populate the fields that comprise a controller
  controllers[i].yMax = yMax;
  controllers[i].yMin = yMin;
  controllers[i].Kp = Kp;
  controllers[i].Ki = Ki;
  controllers[i].Kd = Kd;
  
  /*
  // TODO: initialize for "bumpless transfer" 
  yLast = yActual;
  integralTerm = yCommand;
  if (yMax < integralTerm) integralTerm = yMax;
  else if (integralTerm < yMin) integralTerm = yMin;

  return yCommand;
  */
}


float ComputeControlOutput(const unsigned char i, 
                           const float yDesired, 
                           const float yActual, 
													 const float yNominal) {
  static float integralTerm, yActualLast = 0;
  float error, deltaY, yCommand = 0;
  
  error = yDesired - yActual;
  integralTerm += (controllers[i].Ki * error);
  deltaY = (yActual - yActualLast);
  
  // limit the integral term independently (see Notes section)
  if (controllers[i].yMax < integralTerm) integralTerm = controllers[i].yMax;
  else if (integralTerm < controllers[i].yMin) integralTerm = controllers[i].yMin;
  
  // compute the PID Output
  yCommand = (controllers[i].Kp * error) + integralTerm - 
	          (controllers[i].Kd * deltaY) + yNominal;
  
  // if we've saturated, remove the current error term from 
  // the integral term to prevent integrator windup
  if (controllers[i].yMax < yCommand) {
    yCommand = controllers[i].yMax;
    integralTerm -= (controllers[i].Ki * error);
  } else if (yCommand < controllers[i].yMin) {
    yCommand = controllers[i].yMin;
    integralTerm -= (controllers[i].Ki * error);
  }
  
  // BUG ALERT: ensure the output never goes the opposite of the intended direction
  if ((0 < yDesired) && (yCommand < 0)) yCommand = 0;
  else if ((yDesired < 0) && (0 < yCommand)) yCommand = 0;
  
  yActualLast = yActual;
  return yCommand;
}
