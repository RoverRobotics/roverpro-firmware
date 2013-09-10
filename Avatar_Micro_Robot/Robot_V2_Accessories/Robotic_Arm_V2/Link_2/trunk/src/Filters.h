/*==============================================================================
File: Filters.h
 
Description: This module encapsulates several digital signal processing
  algorithms.
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef FILTERS_H
#define FILTERS_H
/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: DeadBandFilter
Description: Applies hysteresis to a binary threshold.  This is especially
  useful for mitigating 'chatter' on a linearly rising or falling noisy sensor
	input.
Parameters:
  float x,                the value to filter
  float threshold,        the threshold above which a logical high is defined
  float hysteresis,       the width of the hysteresis band in the same units of
	                        the input to filter
Notes:
  - ensure the width of the hysteresis band is at least as great in magnitude 
    as the greatest expected noise
  - see also http://gregstanleyandassociates.com/whitepapers/FaultDiagnosis/Filtering/Hysteresis-Filters/hysteresis-filters.htm
  - see also COK chapter ???
*******************************************************************************/
unsigned char DeadBandFilter(const float x, const float threshold,
                             const float hysteresis);

														 
/*******************************************************************************
Function: ChangeBandFilter
Description: Returns the last given value until an appreciable change is seen.
  This essentially coarsens the given resolution by the minimum-delta 
  specified 
*******************************************************************************/
float ChangeBandFilter(const float x, const float minDelta);


/*******************************************************************************
Function: IIRFilter
Description: Passes the input through an Infinite-Impulse-Response filter
  characterized by the parameter alpha.
Paramters:
	float x,                  the current sample to be filtered
	float alpha,              the knob on how much to filter, [0,1)
	                          alpha = t / (t + dT)
                            where t = the low-pass filter's time-constant
                                 dT = the sample rate
*******************************************************************************/
float IIRFilter(const float x, const float alpha);


/*******************************************************************************
Function: FIRFilter
Description: Passes the input through a Finite-Impulse-Response filter
  (also known as a 'moving average filter')characterized by the given coefficients.
Paramters:
	float x,                  the current sample to be filtered
*******************************************************************************/
float FIRFilter(const float x);
 
#endif
