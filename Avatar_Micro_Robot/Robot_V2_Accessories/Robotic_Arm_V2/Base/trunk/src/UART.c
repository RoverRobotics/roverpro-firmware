/*=============================================================================

File: UART.c
Description: Provides an interface to the UART1 hardware module.

internal FRC = 8MHz
F_OSC = 20MHz (given our crystal)
F_CY = F_OSC / 2

F_OSC = 4MHz (after divide by 5 from PLL)

=============================================================================*/
#define TEST_UART1

/*---------------------------Dependencies------------------------------------*/
#include "UART.h"
#include <p24FJ256GB106.h>
//#include "ConfigurationBits.h"

/*---------------------------Macros and Definitions--------------------------*/


/*---------------------------Helper Function Prototypes----------------------*/

/*---------------------------Module Variables--------------------------------*/

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_UART1

#define THRESHOLD			150		// threshold for testing Rx
#define OPS_PER_MS		3200	// roughly, for Pause()

static inline void Pause(unsigned int milliseconds) {
	unsigned int i,j;
	for (i = 0; i < OPS_PER_MS; ++i) {
		for (j = 0; j < milliseconds; ++j) {}
	}
}

int main(void) {
  unsigned char dummy_message = 0;
  
  InitUART(1);
  
  while (1) {
    if (IsTxClear()) {
      TransmitByte(dummy_message);
      dummy_message++;
      
			Pause(25);
    }

		// uncomment to test reception independently
    /*
    if (IsRxAvailable()) {
      x = GetRxByte();
    	if (x < THRESHOLD) {
			}
		}
    */
  }
  return 0;
}

#endif
/*---------------------------End Test Harness--------------------------------*/

/*---------------------------Public Function Definitions---------------------*/
void InitUART(unsigned char uart_number) {
  // configure Tx pin as digital
  
	//MapPeripherals(unsigned char uart_number);
	// map inputs (see table 9-1, p.126 of datasheet)
	RPINR18bits.U1RXR = ?; 	// assign U1RX to pin RP17
	RPINR18bits.U1CTSR = ?;	// assign U1CTS to pin ?
	
	// map outputs (see table 9-2, p.127 of datasheet)
	// (see p.127 of datasheet)
	RPOR1bits.RP2R = 3;			// assign U1TX To Pin RP10
	RPOR1bits.RP3R = 4;			// assign U1RTS To Pin ?
	
	// TODO: test to ensure analog doesn't explicitly need to be DISABLED for any of these pins

  // configure for high precision baud rate (U1MODE, BRGH	1)
  U1MODEbits.BRGH = 1;
  // configure baud rate for 115200[pulses/s => 16bits/s?] (see p.190 of datasheet)
	// UxBRG = F_CY / (16 * desired_baud_rate) - 1
  U1BRG = 34;         		// 115200 ~= 20e6Hz/(4*(34+1))
  
  // configure for 8-bit, no parity transmission
  U1MODEbits.PDSEL0 = 0;
  U1MODEbits.PDSEL1 = 0;

  U1MODEbits.UARTEN = 1; 	// enable the UART module
  U1MODEbits.STSEL = 0; 	// configure to 1 stop bit
  U1STAbits.UTXEN = 1; 		// enable transmission
}

char inline IsTxClear(void) {
  return U1STAbits.TRMT;
}

void inline TransmitByte(unsigned char message) {
  U1TXREG = message;
}

char inline IsRxAvailable(void) {
  return (char)U1STAbits.URXDA;
}

char inline GetRxByte(void) {
  return (char)U1RXREG;
}


/*---------------------------Private Function Definitions--------------------*/

/*---------------------------End of File-------------------------------------*/
