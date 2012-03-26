/*=============================================================================
 
File: Base.h

Description: This module encapsulates the firmware on the Base PCB of the
	Robotic Arm.  It communicates with the Avatar on which it is mounted as well
  as Link 1 of the rest of the arm.
=============================================================================*/
#ifndef ARM_BASE_H
#define ARM_BASE_H

/*---------------------------Macros and Definitions--------------------------*/

/*---------------------------Public Function Prototypes----------------------*/
/****************************************************************************
Function: InitBase()

Description: Initializes any I/O pins and dependent modules required by the
	 the Base PCB.

*****************************************************************************/
void InitBase(void);

#endif
