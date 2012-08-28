/*==============================================================================
File: PPS.h 

Description: This file provides an interface to the dynamic pin mapping 
  supported by the Periphal Pin Selection (PPS) facilities of the 
  PIC24FJ256GB106.

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef PPS_H
#define PPS_H

/*---------------------------Macros-------------------------------------------*/
// Selectable Output Sources (see Table 10-3, p.138 of datasheet)
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

// Selectable Input Sources (see Table 10-2, p.137 of datasheet)
// Note: Don't know that there is a way to pass bit-fields :(
// I'm arbitrariliy assigning numbers for a switch() statement within PPS.c
// because I don't think it's worth it to make it's own type and can't think
// of a better option right now
#define FN_INT1						0
#define FN_INT2						1
#define FN_INT3						2
#define FN_INT4						3
#define FN_IC1						4
#define FN_IC2						5
#define FN_IC3						6
#define FN_IC4						7
#define FN_IC5						8
#define FN_IC6						9
#define FN_IC7						10
#define FN_IC8						11
#define FN_IC9						12
#define FN_OCFA						13
#define FN_OCFB						14
#define FN_SCK1IN					15
#define FN_SDI1						16
#define FN_SS1IN					17
#define FN_SCK2IN					18
#define FN_SDI2						19
#define FN_SS2IN					20
#define FN_SCK3IN					21
#define FN_SDI3						22
#define FN_SS3IN					23
#define FN_T1CK						24
#define FN_T2CK						25
#define FN_T3CK						26
#define FN_T4CK						27
#define FN_T5CK						28
#define FN_U1CTS_BAR			29
#define FN_U1RX						30
#define FN_U2CTS_BAR			31
#define FN_U2RX						32
#define FN_U3CTS_BAR			33
#define FN_U3RX						34
#define FN_U4CTS_BAR			35
#define FN_U4RX						36

/*---------------------------Public Function Prototypes-----------------------*/
/*
Function: PPS_MapPeripheral
Description: Initializes the specified PWM hardware module and begins
  outputting at 0% duty cycle on the specified pin.
Usage: 
	PPS_MapPeripheral(21, OUTPUT, FN_OC2);  // map remappable pin RP21 as an 
	                                        // output with Output Compare 2 
	                                        // functionality for PWM
*/
void inline PPS_MapPeripheral(unsigned char pin, unsigned char direction,
                              unsigned char function);
#endif
