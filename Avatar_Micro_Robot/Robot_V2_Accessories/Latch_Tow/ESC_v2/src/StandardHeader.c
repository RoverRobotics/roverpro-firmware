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
void Delay(unsigned int milliseconds) {
  unsigned int i,j;
	for (i = 0; i < OPS_PER_MS; i++) {
    for (j = 0; j < milliseconds; j++);
  }
}


unsigned int Map(int value, int fromLow, int fromHigh, 
                 int toLow, int toHigh) {
  // compute the linear interpolation
  unsigned int result = ((double)(value - fromLow) / (double)(fromHigh - fromLow))
                        * (double)(toHigh - toLow) + toLow;
  
  // constrain the result to within a valid output range
  if (toHigh < result) result = toHigh;
  else if (result < toLow) result = toLow;
  
  return result;
}
