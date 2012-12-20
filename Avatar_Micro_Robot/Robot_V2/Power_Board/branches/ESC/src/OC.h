/*==============================================================================
File: OC.h

Description: N/A

Notes:
  - consumes timer4 as the time base for input capture
  - consumes output compare module 1, 2, 3, 4, 5, 6???

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef OC_H
#define OC_H
//---------------------------Dependencies---------------------------------------
#include <stdint.h>

//---------------------------Public Functions-----------------------------------
// Function: OC_Init
// Description: Initializes the output compare module.
void OC_Init(void);

void Energize(const uint8_t hall_state);

#endif
