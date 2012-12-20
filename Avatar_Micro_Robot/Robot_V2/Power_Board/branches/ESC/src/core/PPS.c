/*==============================================================================
File: PPS.c

Notes:
	- see also Section 12 of PIC24F Family Reference Manual
==============================================================================*/
/*---------------------------Dependencies-------------------------------------*/
#include "./PPS.h"
#include "./StandardHeader.h"	// for INPUT/OUTPUT macros

/*---------------------------Helper Function Prototypes-----------------------*/
static inline void UnlockControlRegisters(void);
static inline void LockControlRegisters(void);

/*---------------------------Public Function Definitions----------------------*/
void inline PPS_MapPeripheral(unsigned char pin, unsigned char direction,
                              unsigned char function) {
	//UnlockControlRegisters();
	
	// Compiler compains when attempting to pass a bit-field by reference.  
	// Must hard-code for now :(
	switch (direction) {
  	case OUTPUT:
      switch (pin) {  	
      	case 0: _RP0R = function; break;
    		case 1: _RP1R = function; break;
    		case 2: _RP2R = function; break;
    		case 3: _RP3R = function; break;
    		case 4: _RP4R = function; break;
    		case 6: _RP6R = function; break;
    		case 7: _RP7R = function; break;
    		case 8: _RP8R = function; break;
    		case 9: _RP9R = function; break;
    		case 10: _RP10R = function; break;
    		case 11: _RP11R = function; break;
    		case 12: _RP12R = function; break;
    		case 13: _RP13R = function; break;
    		case 14: _RP14R = function; break;
    		case 17: _RP17R = function; break;
    		case 18: _RP18R = function; break;
    		case 19: _RP19R = function; break;
    		case 20: _RP20R = function; break;
    		case 21: _RP21R = function; break;
    		case 22: _RP22R = function; break;
    		case 23: _RP23R = function; break;
    		case 24: _RP24R = function; break;
    		case 25: _RP25R = function; break;
    		case 26: _RP26R = function; break;
    		case 27: _RP27R = function; break;
    		case 28: _RP28R = function; break;
    		case 29: _RP29R = function; break;
		  }
  	  break;
  	case INPUT:
  	  switch (function) {
  	    case FN_INT1: _U1RXR = pin; break;
        case FN_INT2: _INT2R = pin; break;
        case FN_INT3: _INT3R = pin; break;
        case FN_INT4: _INT4R = pin; break;
        case FN_IC1: _IC1R = pin; break;
        case FN_IC2: _IC2R = pin; break;
        case FN_IC3: _IC3R = pin; break;
        case FN_IC4: 	_IC4R = pin; break;
        case FN_IC5: 	_IC5R = pin; break;
        case FN_IC6: 	_IC6R = pin; break;
        case FN_IC7: _IC7R = pin; break;
        case FN_IC8: _IC8R = pin;	break;
        case FN_IC9: 	_IC9R = pin; break;
        case FN_OCFA: _OCFAR = pin; break;
        case FN_OCFB: _OCFBR = pin; break;
        case FN_SCK1IN: _SCK1R = pin; break;
        case FN_SDI1: _SDI1R = pin; break;
        case FN_SS1IN: _SS1R = pin; break;
        case FN_SCK2IN: _SCK2R = pin; break;
        case FN_SDI2: _SDI2R = pin; break;
        case FN_SS2IN: _SS2R = pin; break;
        case FN_SCK3IN: _SCK3R = pin; break;
        case FN_SDI3: _SDI3R = pin; break;
        case FN_SS3IN: _SS3R = pin; break;
        case FN_T1CK: _T1CKR = pin; break;
        case FN_T2CK: _T2CKR = pin; break;
        case FN_T3CK: _T3CKR = pin; break;
        case FN_T4CK: _T4CKR = pin; break;
        case FN_T5CK: _T5CKR = pin; break;
        case FN_U1CTS_BAR: _U1CTSR = pin; break;
        case FN_U1RX: _U1RXR = pin; break;
        case FN_U2CTS_BAR: _U2CTSR = pin; break;
        case FN_U2RX: _U2RXR = pin; break;
        case FN_U3CTS_BAR: _U3CTSR = pin; break;
        case FN_U3RX: _U3RXR = pin; break;
        case FN_U4CTS_BAR: _U4CTSR = pin; break;
        case FN_U4RX: _U4RXR = pin; break;
  	  }
  	  break;
	}

	//LockControlRegisters();
}

/*---------------------------Helper Function Definitions----------------------*/
/*
Notes:
	- see p.140 of datasheet describing these special sequences
	- Because the unlock sequence is timing-critical, it must
	  be executed as an assembly language routine.  The C30 compiler now 
	  provides a convenient macro for this sequence
*/
static inline void UnlockControlRegisters(void) {
  __builtin_write_OSCCONL(OSCCON & 0xBF);
  /*
	asm volatile ( "MOV #OSCCON, w1 \n"
								 "MOV #0x46, w2 \n"
								 "MOV #0x57, w3 \n"
								 "MOV.b w2, [w1] \n"
								 "MOV.b w3, [w1] \n"
								 "BCLR OSCCON,#6");
  */
}

static inline void LockControlRegisters(void) {
	__builtin_write_OSCCONL(OSCCON | 0x40);
	/*
	asm volatile ( "MOV #OSCCON, w1 \n"
								 "MOV #0x46, w2 \n"
								 "MOV #0x57, w3 \n"
							 	 "MOV.b w2, [w1] \n"
								 "MOV.b w3, [w1] \n"
								 "BSET OSCCON, #6" );
  */
}
