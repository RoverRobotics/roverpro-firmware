/*==============================================================================
File: Filters.h
 
Description: This module encapsulates several digital signal processing
  algorithms.  It is still very much a work in progress...
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef FILTERS_H
#define FILTERS_H
/*---------------------------Dependencies-------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#define MAX_N_FILTERS       16

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: DeadBandFilter
Description: Applies hysteresis to a binary threshold.  This is especially
  useful for mitigating 'chatter' on a linearly rising or falling noisy sensor
	input.
Parameters:
  const uint8_t i,        the filter index
  const float x,          the value to filter
  const float threshold,  the threshold above which a logical high is defined
  const float hysteresis, the width of the hysteresis band in the same units of
	                        the input to filter
Notes:
  - ensure the width of the hysteresis band is at least as great in magnitude 
    as the greatest expected noise
  - see also http://gregstanleyandassociates.com/whitepapers/FaultDiagnosis/Filtering/Hysteresis-Filters/hysteresis-filters.htm
  - see also COK chapter ???
*******************************************************************************/
unsigned char DeadBandFilter(const uint8_t i, const float x, 
                             const float threshold, const float hysteresis);

														 
/*******************************************************************************
Function: ChangeBandFilter
Description: Returns the last given value until an appreciable change is seen.
  This essentially coarsens the given resolution by the minimum-delta 
  specified 
Parameters:
  const uint8_t i,        the filter index
  const float x,          the current sample to be filtered
  const float min_delta   the minimum change
*******************************************************************************/
float ChangeBandFilter(const uint8_t i, const float x, const float min_delta);


/*******************************************************************************
Function: IIRFilter
Description: Passes the input through an Infinite-Impulse-Response filter
  characterized by the parameter alpha.
Paramters:
  const uint8_t i,          the filter index
	const float x,            the current sample to be filtered
	const float alpha,        the knob on how much to filter, [0,1)
	                          alpha = t / (t + dT)
                            where t = the low-pass filter's time-constant
                                 dT = the sample rate
  const bool should_reset,  whether to clear the history
*******************************************************************************/
float IIRFilter(const uint8_t i, const float x, const float alpha,
                const bool should_reset);


/*******************************************************************************
Function: FIRFilter
Description: Passes the input through a Finite-Impulse-Response filter
  (also known as a 'moving average filter')characterized by the given coefficients.
Paramters:
  const uint8_t i,          the filter index
	float x,                  the current sample to be filtered
  const float* coefficients how much to weight each of the samples
*******************************************************************************/
float FIRFilter(const uint8_t i, const float x, const float coefficients[]);
 
#endif
