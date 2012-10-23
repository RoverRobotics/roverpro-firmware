/*==============================================================================
File: StandardHeader.c
==============================================================================*/
//#define TEST_STANDARD_HEADER
/*---------------------------Dependencies-------------------------------------*/
#include "./StandardHeader.h"

/*---------------------------Macros-------------------------------------------*/
#define OPS_PER_MS            3200  // operations per millisecond, for current 
                                    // oscillator choice and 
                                    // configuration-bit settings
#define OPS_PER_5US           16    // operations per 5 microseconds

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_STANDARD_HEADER
int main(void) {
  _TRISE5 = OUTPUT; _RE5 = 0;       // configure a debugging pin
  
  unsigned char i = 0;
	while (1) {
  	/*
  	// test Pause()
  	Pause(500);
  	_RE5 ^= 1;
  	*/
  	
  	// test Map() (with Pause())
		Pause(Map(i++, 20, 80, 0, 100));
		_RE5 ^= 1;                      // toggle a pin to view an output 
	}

	return 0;
}
#endif
/*---------------------------Public Function Definitions----------------------*/
void Delay(const unsigned int milliseconds) {
  unsigned int i,j;
	for (i = 0; i < OPS_PER_MS; i++) {
    for (j = 0; j < milliseconds; j++);
  }
}


void Delay5us(const uint16_t _5microseconds) {
  unsigned int i,j;
	for (i = 0; i < OPS_PER_5US; i++) {
    for (j = 0; j < _5microseconds; j++);
  }
}


unsigned int Map(const int value, const int from_low, const int from_high, 
                 const int to_low, const int to_high) {
  // compute the linear interpolation
  unsigned int result = ((double)(value - from_low) / (double)(from_high - from_low))
                        * (double)(to_high - to_low) + to_low;
  
  // constrain the result to within a valid output range
  if (to_high < result) result = to_high;
  else if (result < to_low) result = to_low;
  
  return result;
}
