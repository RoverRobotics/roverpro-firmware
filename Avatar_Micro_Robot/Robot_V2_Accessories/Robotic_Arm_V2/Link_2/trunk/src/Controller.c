/*==============================================================================
File: Controller.c
Notes:
  - http://en.wikipedia.org/wiki/PID_controller
  - inspired by: http://brettbeauregard.com/
==============================================================================*/
/*---------------------------Dependencies-------------------------------------*/
#include "./Controller.h"

/*---------------------------Macros and Definitions---------------------------*/
typedef struct {
  double yMax;  // the maximum value the output can produce
  double yMin;  // the minimum value the output can produce
	double Kp;    // proportional gain
	double Ki;    // integral gain
  double Kd;    // derivative gain
} controller_t;

/*---------------------------Helper Function Prototypes-----------------------*/

/*---------------------------Module Variables---------------------------------*/
static controller_t controllers[MAX_NUM_CONTROLLERS];

/*---------------------------Public Function Definitions----------------------*/
/*******************************************************************************
Note(s):
	- 
*******************************************************************************/
void InitController(unsigned char i, float yMax, float yMin, 
                    float Kp, float Ki, float Kd) {
  // populate the fields that comprise a controller
  controllers[i].yMax = yMax;
  controllers[i].yMin = yMin;
  controllers[i].Kp = Kp;
  controllers[i].Ki = Ki;
  controllers[i].Kd = Kd;
  
  /*
  // initialize for "bumpless transfer" 
  yLast = yActual;
  integralTerm = yCommand;
  if (yMax < integralTerm) integralTerm = yMax;
  else if (integralTerm < yMin) integralTerm = yMin;

  return yCommand;
  */
}


/*
Description: This control algorithm attempts to control a system below its
  output ability -- ie acheive an effective yDesired that is below its yMin 
*/
float ComputeControlOutput(unsigned char i, float yDesired, float yActual) {
  static float integralTerm, yActualLast = 0;
  float error, deltaY, yCommand = 0;
  
  error = yDesired - yActual;
  integralTerm += (controllers[i].Ki * error);
  deltaY = (yActual - yActualLast);
  
  // limit the integral term independently to prevent windup
  // TODO: integral term should have independent limits
  if (controllers[i].yMax < integralTerm) integralTerm = controllers[i].yMax;
  else if (integralTerm < controllers[i].yMin) integralTerm = controllers[i].yMin;
  
  // compute the PID Output, limiting to within its bounds
  yCommand = (controllers[i].Kp * error) + integralTerm - (controllers[i].Kd * deltaY);
  
  // limit the output to within a valid range
  if (controllers[i].yMax < yCommand) yCommand = controllers[i].yMax;
  else if (yCommand < controllers[i].yMin) yCommand = controllers[i].yMin;
  
  // BUG ALERT: ensure the output never goes the opposite of the intended direction
  if ((0 < yDesired) && (yCommand < 0)) yCommand = 0;
  else if ((yDesired < 0) && (0 < yCommand)) yCommand = 0;
  
  yActualLast = yActual;
  return yCommand;
}

float ComputeBangBangOutput(unsigned char i, float yDesired, float yActual) {
  if (yDesired < yActual) return 0;
  else return controllers[i].yMin;
}
  
/*---------------------------Private Function Definitions---------------------*/

