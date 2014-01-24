/*==============================================================================
File: Filters.c
==============================================================================*/
//#define TEST_FILTERS
/*---------------------------Dependencies-------------------------------------*/
#include "Filters.h"
#include <stdlib.h>   // for abs() function
#include "p24FJ256GB106.h"
#include "stdhdr.h"

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_FILTERS
#include "./ConfigurationBits.h"

int main(void) {
  
	while (1) {
	}
	
	return 0;
}
#endif
/*---------------------------Public Function Definitions----------------------*/
// WARNING: assumes all parameters are positive
uint8_t DeadBandFilter(const uint8_t i, const float x,
                             const float threshold, const float hysteresis) {
  // TODO: TEST THIS!
  /*
	static uint8_t current_state = 0;
	if (current_state == 1) {
    // look to pass the low threshold
	  if (x < (threshold - hysteresis)) current_state = 0;
  } else {
    // look to pass the high threshold
    if ((threshold + hysteresis) < x) current_state = 1;
  } 	
	
	return current_state;
  */
  return 0;
}

														 
float ChangeBandFilter(const uint8_t i, const float x, const float min_delta) {
	// TODO: TEST THIS AGAIN
  /*
  // ensure the input has changed appreciably
	static float last_x = 0;
	last_x = x;
	float delta = fabs(x - last_x);
	if (min_delta < delta)  {
		last_x = x;
		return x;
	}
	
	return last_x;
  */
  return 0;
}


float IIRFilter(const uint8_t i, const float x, const float alpha,
                const bool should_reset) {
// see also: http://dsp.stackexchange.com/questions/1004/low-pass-filter-in-non-ee-software-api-contexts
// aka a 'leaky integrator'
  static float y_lasts[MAX_N_FILTERS] = {0};
  if (should_reset) y_lasts[i] = 0;
  float y = alpha * y_lasts[i] + (1.0 - alpha) * x;
  y_lasts[i] = y;
  
  return y;
}


float FIRFilter(const uint8_t i, const float x, const float coefficients[]) {
// see also: http://ptolemy.eecs.berkeley.edu/eecs20/week12/implementation.html
  /*
  TODO: TEST THIS!
  // take a moving average of the past 16 values
  #define NUM_SAMPLES   16
  static int samples[NUM_SAMPLES] = {0};
  static unsigned char currentIndex = 0;
  
  // add in the new value
  samples[currentIndex] = x;
  if (NUM_SAMPLES < ++currentIndex) currentIndex = 0;
  
  // sum the previous window of samples
  unsigned char i;
  long int sum = 0;
  for (i = 0; i < NUM_SAMPLES; i++) sum += samples[i];
  
  return (sum / NUM_SAMPLES);
  */
  return 0;
}

// TODO: ButterworthFilter(), KalmanFilter(), ExtendedKalmanFilter(),
// ChebychevFilter(), make their own modules?
