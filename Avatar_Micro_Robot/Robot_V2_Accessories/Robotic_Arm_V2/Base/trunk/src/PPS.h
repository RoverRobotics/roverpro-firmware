/*=============================================================================
File: PWM.h 

Description: This file provides an interface to one of several Pulse-Width 
	Modulation (PWM) hardware modules.

Notes:
  - assumes a 20MHz external oscillator
  - assumes exclusive use of the Timer associated with the given module number
  - configures for Phase Correct PWM Mode
  - assumes exclusive use of the pins associated with the given module number 
  - performs no error checking on input parameters
 
TODO: generalize implementation to be able to initialize and use any 
     of the existing PWM hardware modules.  Currently only provides for PWM Timer2	   
=============================================================================*/
#ifndef PPS_H
#define PPS_H

/*---------------------------Macros------------------------------------------*/
#define OUTPUT						0		
#define INPUT							1

// Selectable Input Sources (see Table 9-1, p.124 of datasheet)
#define FN_INT1						RPINR0
#define FN_INT2						RPINR1
#define FN_INT3						RPINR1
#define FN_INT4						RPINR2
#define FN_IC1						RPINR7
#define FN_IC2						RPINR7
#define FN_IC3						RPINR8
#define FN_IC4						RPINR8
#define FN_IC5						RPINR9
#define FN_IC6						RPINR9
#define FN_IC7						RPINR10
#define FN_IC8						RPINR10
#define FN_IC9						RPINR15
#define FN_OCFA						RPINR11
#define FN_OCFB						RPINR11
#define FN_SCK1IN					RPINR20
#define FN_SDI1						RPINR20
#define FN_SS1IN					RPINR21
#define FN_SCK2IN					RPINR22
#define FN_SDI2						RPINR22
#define FN_SS2IN					RPINR23
#define FN_SCK3IN					RPINR23
#define FN_SDI3						RPINR28
#define FN_SS3IN					RPINR29
#define FN_T1CK						RPINR2
#define FN_T2CK						RPINR3
#define FN_T3CK						RPINR3
#define FN_T4CK						RPINR4
#define FN_T5CK						RPINR4
#define FN_U1CTS_BAR			RPINR18
#define FN_U1RX						RPINR18
#define FN_U2CTS_BAR			RPINR19
#define FN_U2RX						RPINR19
#define FN_U3CTS_BAR			RPINR21
#define FN_U3RX						RPINR17
#define FN_U4CTS_BAR			RPINR27
#define FN_U4RX						RPINR27

// Selectable Output Sources (see Table 9-2, p.125 of datasheet)
#define FN_NULL						0
#define FN_C1OUT					1
#define FN_C2OUT					2
#define FN_U1TX 					3
#define FN_U1RTS_BAR			4
#define FN_U2TX     			5
#define FN_U2RTS_BAR  		6
#define FN_SDO1     			7
#define FN_SCK1OUT	  		8
#define FN_SS1OUT					9
#define FN_SDO2						10
#define FN_SCK2OUT				11
#define FN_SS2OUT					12
#define FN_OC1						18
#define FN_OC2						19
#define FN_OC3						20
#define FN_OC4						21
#define FN_OC5						22
#define FN_OC6						23
#define FN_OC7						24
#define FN_OC8						25
#define FN_U3TX						28
#define FN_U3RTS_BAR			29
#define FN_U4TX						30
#define FN_U4RTS_BAR			31
#define FN_SDO3						32
#define FN_SCK3OUT				33
#define FN_SS3OUT					34
#define FN_OC9						35

/*---------------------------Public Function Prototypes----------------------*/
/*
Function: MapPeripheral()
Description: Initializes the specified PWM hardware module and begins
  outputting at 0% duty cycle on the specified pin.
Usage: 
	// Map remappable pin RP21 as an output with Output Compare 2 functionality for PWM
	MapPeripheral(21, OUTPUT, FN_OC2);
*/
void MapPeripheral(unsigned char pin, unsigned char direction,
                   unsigned char function);

#endif
