/*==============================================================================
File: Filters.c
==============================================================================*/
//#define TEST_FILTERS
/*---------------------------Dependencies-------------------------------------*/
#include "./DSP.h"
#include "./StandardHeader.h"

/*---------------------------Macros-------------------------------------------*/

/*---------------------------Helper Function Prototypes-----------------------*/

/*---------------------------Module Variables---------------------------------*/

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
unsigned char DeadBandFilter(const float x, const float threshold,
                             const float hysteresis) {
	unsigned char result = 0;
	static float lastThreshold = threshold;
	if (x < (threshold - hysteresis)) {
		result = 0;
		lastThreshold = threshold + hysteresis;
	} else if ((threshold + hysteresis) < x) {
		result = 1;
		lastThreshold = threshold - hysteresis;
	}
	
	return result;
}

														 
float ChangeBandFilter(const float x, const float minDelta) {
	// ensure the input has changed appreciably
	static float lastX = x;
	float delta = abs(x - lastX);
	if (minDelta < delta)  {
		lastX = x;
		return x;
	}
	
	return lastX;
}


float IIR(const float alpha, const float x) {
	// first-order infinite impulse response (IIR) filter
  // http://dsp.stackexchange.com/questions/1004/low-pass-filter-in-non-ee-software-api-contexts
  static float yLast = 0;
  float y = alpha * yLast + (1.0 - alpha) * x;
  yLast = y;
  
  return y;
}

// TODO: implement FIR() filter
// TODO: make a KalmanFilter.c/h module
//float KalmanFilter(const unsigned char dspIndex, const float x);
//float ExtendedKalmanFilter(const unsigned char dspIndex, const float x) {}

