/*=============================================================================
File: PPS.c

Notes:
	- see also Section 12 of PIC24F Family Reference Manual
=============================================================================*/
/*---------------------------Dependencies------------------------------------*/
#include "./PPS.h"
#include <p24FJ256GB106.h>

/*---------------------------Helper Function Prototypes----------------------*/
static void UnlockControlRegisters(void);
static void LockControlRegisters(void);

/*---------------------------Public Function Definitions---------------------*/
void MapPeripheral(unsigned char pin, unsigned char direction, 
                   unsigned char function) {
	UnlockControlRegisters();
	
	// Passing a bit-field by reference doesn't work.  Have to hard-code for now.
	switch (direction) {
		case OUTPUT:
			switch (pin) {
				case 0: _RP0R = function; break;
				case 1: _RP1R = function; break;
				case 2: _RP2R = function; break;
				case 3: _RP3R = function; break;
				case 4: _RP4R = function; break;
				case 11: _RP11R = function; break;
				case 12: _RP12R = function; break;
				case 13: _RP13R = function; break;
				case 18: _RP18R = function; break;
				case 19: _RP19R = function; break;
				case 21: _RP21R = function; break;
				case 26: _RP26R = function; break;
				case 27: _RP27R = function; break;
				case 28: _RP28R = function; break;
			}
			break;
		case INPUT:
			// TODO: implmement this
			break;
	}

	LockControlRegisters();
}

/*---------------------------Helper Function Definitions---------------------*/
/*
Notes:
	- see p.127 of datasheet describing these special sequences
	- Because the unlock sequence is timing-critical, it must
	  be executed as an assembly language routine
*/
static void UnlockControlRegisters(void) {
	asm volatile ( "MOV #OSCCON, w1 \n"
								 "MOV #0x46, w2 \n"
								 "MOV #0x57, w3 \n"
								 "MOV.b w2, [w1] \n"
								 "MOV.b w3, [w1] \n"
								 "BCLR OSCCON,#6");
}

static void LockControlRegisters(void) {
	asm volatile ( "MOV #OSCCON, w1 \n"
								 "MOV #0x46, w2 \n"
								 "MOV #0x57, w3 \n"
							 	 "MOV.b w2, [w1] \n"
								 "MOV.b w3, [w1] \n"
								 "BSET OSCCON, #6" );
}
