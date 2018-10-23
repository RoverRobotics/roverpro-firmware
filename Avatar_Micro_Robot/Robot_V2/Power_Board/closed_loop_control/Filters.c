/*==============================================================================
File: Filters.c
==============================================================================*/
//#define TEST_FILTERS
/*---------------------------Dependencies-------------------------------------*/
#include "Filters.h"
#include <stdlib.h> // for abs() function
#include <p24Fxxxx.h>
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

float IIRFilter(const uint8_t i, const float x, const float alpha, const bool should_reset) {
    // see also:
    // http://dsp.stackexchange.com/questions/1004/low-pass-filter-in-non-ee-software-api-contexts
    // aka a 'leaky integrator'
    static float y_lasts[MAX_N_FILTERS] = {0};
    if (should_reset)
        y_lasts[i] = 0;
    float y = alpha * y_lasts[i] + (1.0 - alpha) * x;
    y_lasts[i] = y;

    return y;
}

// TODO: ButterworthFilter(), KalmanFilter(), ExtendedKalmanFilter(),
// ChebychevFilter(), make their own modules?
