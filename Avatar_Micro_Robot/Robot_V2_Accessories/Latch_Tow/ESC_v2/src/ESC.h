/*=============================================================================
File: ESC.h 

Description: This file encapsulates an Electronic Speed Controller (ESC) to
  facilitate the driving of Sensorless Brushless DC (BLDC) Motor.

Notes:
  - can be used for both Wye-wound and delta-wound 3-phase motors
  - designed for circuit in Figure 1. of Microchip application note AN1175
  - designed to drive a star-configured BLDC motor
  - uses Timer4 as 16-bit timer (does not also consume Timer5)
  
  - this file depends on all the control transistors being wired contiguously
    on a single port with high-side commutation transitors being grouped
    and the low-side commutation transistors being grouped.

  //- R_coil ~= 0.6 Ohm
  //- black wire on motor is phase B

See also: 
  - expected pinout
  - current-paths schematic
  - Microchip application note AN1175 (beware of documentation errors)

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
=============================================================================*/
#ifndef ESC_H
#define ESC_H
/*---------------------------Public Function Prototypes----------------------*/
/*
Function: InitESC()
*/
void InitESC(void);

/*
Function: UpdateESCSpeed()
Parameters:
	unsigned char speed,      the percent of maximum speed (0-to-100 inclusive)
	unsigned char direction,  clockwise (CW) or counter-clockwise (CCW) as 
	                          viewed looking down the end of the shaft.  Macros
	                          for CW and CCW are included in StandardHeader.h
*/
//void UpdateESCSpeed(unsigned char speed, unsigned char direction);
void TurnMotor(unsigned char direction, unsigned char speed);

#endif
