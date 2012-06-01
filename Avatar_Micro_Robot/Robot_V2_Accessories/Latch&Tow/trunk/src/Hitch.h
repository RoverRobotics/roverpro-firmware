/*=============================================================================
File: Hitch.h

Description: This is the overarching file that encapsulates the 
  application-level firmware for the hitch.  It integrates any dependent 
  firmware modules and interfaces with external components.
  
Notes:
  - The brown wire for the ESC is GND
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
=============================================================================*/
#ifndef HITCH_H
#define HITCH_H

/*---------------------------Public Function Prototypes----------------------*/
/*****************************************************************************
Function: InitHitch()
Description: Initializes any I/O pins and dependent modules required by the
	 the Base PCB.
******************************************************************************/
void InitHitch(void);

/*****************************************************************************
Function: ProcessHitchIO()
Description: Runs the hitch state machine.
******************************************************************************/
void ProcessHitchIO(void);

#endif
