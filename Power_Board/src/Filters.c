/*==============================================================================
File: Filters.c
==============================================================================*/
/*---------------------------Dependencies-------------------------------------*/
#include "Filters.h"
#include "stdhdr.h"

/*---------------------------Public Function Definitions----------------------*/

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