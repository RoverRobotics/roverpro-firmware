/*==============================================================================
File: Filters.c
==============================================================================*/
//#define TEST_FILTERS
/*---------------------------Dependencies-------------------------------------*/
#include "./Filters.h"
#include "./StandardHeader.h"
#include <stdlib.h>   // for abs() function

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
unsigned char DeadBandFilter(const float x, const float threshold,
                             const float hysteresis) {
	static unsigned char currentState = 0;
	if (currentState == 1) {
    // look to pass the low threshold
	  if (x < (threshold - hysteresis)) currentState = 0;
  } else {
    // look to pass the high threshold
    if ((threshold + hysteresis) < x) currentState = 1;
  } 	
	
	return currentState;
}

														 
float ChangeBandFilter(const float x, const float minDelta) {
	// ensure the input has changed appreciably
	static float lastX = 0;
	lastX = x;
	float delta = abs(x - lastX);
	if (minDelta < delta)  {
		lastX = x;
		return x;
	}
	
	return lastX;
}


// TODO: this is a general class of filters? break down
float IIRFilter(const float x, const float alpha) {
	// first-order infinite impulse response (IIR) filter
	// (aka a 'leaky integrator')
  // http://dsp.stackexchange.com/questions/1004/low-pass-filter-in-non-ee-software-api-contexts
  static float yLast = 0;
  float y = alpha * yLast + (1.0 - alpha) * x;
  yLast = y;
  
  return y;
}

/* 
TODO: this is a general class of filters?  break down
Note: we can reduce computation by only summing the front
  and subtracting the end, but it is marginal improvement for
  this situation
*/
float FIRFilter(const float x) {
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
}
  
// TODO: ButterworthFilter(), KalmanFilter(), ExtendedKalmanFilter(),
// ChebychevFilter(), make their own modules?
