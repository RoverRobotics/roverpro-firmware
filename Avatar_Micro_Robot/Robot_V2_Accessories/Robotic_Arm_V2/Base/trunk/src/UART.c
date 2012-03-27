/*=============================================================================

File: UART.c
Description: Provides an interface to the UART hardware modules.


Notes:
F_OSC = 32MHz (given our 20MHz crystal, dividy-by-5 from PLL, and 
               multiply-by-8 from somewhere)
F_CY = F_OSC / 2 = 16MHz
=============================================================================*/
#define TEST_UART

/*---------------------------Dependencies------------------------------------*/
#include "UART.h"
#include <p24FJ256GB106.h>
#include "ConfigurationBits.h"

/*---------------------------Macros and Definitions--------------------------*/


/*---------------------------Helper Function Prototypes----------------------*/
static inline MapPeripherals(unsigned char uart_number);
static inline ConfigureBaudRate(unsigned char uart_number, 
                                unsigned int baud_rate);

/*---------------------------Module Variables--------------------------------*/

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_UART

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
/*
Notes:
	1) by default, configures to 1 stop bit, 8-bit transmission, no parity
*/
void InitUART(unsigned char uart_number, unsigned int baud_rate) {
	// (see also: register definitions for default configurations)
	MapPeripherals(uart_number);
	ConfigureBaudRate(uart_number, baud_rate);

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
static inline MapPeripherals(unsigned char uart_number) {
	
	U1MODEbits.UARTEN = 0;	// control UARTx pins through corresponding PORT, LAT and TRIS bits

// TODO: test to ensure analog doesn't explicitly need to be DISABLED for
  // any of these pins
	LATF5 = UART1RX;
	LATF4 = UART1TX;

	// map inputs (see table 9-1, p.126 of datasheet)
	RPINR18bits.U1RXR = ?; 	// assign U1RX to pin RP17
	RPINR18bits.U1CTSR = ?;	// assign U1CTS to pin ?
	
	// map outputs (see table 9-2, p.127 of datasheet)
	RPOR1bits.RP2R = 3;			// assign U1TX To Pin RP10
	RPOR1bits.RP3R = 4;			// assign U1RTS To Pin ?
}

static inline ConfigureBaudRate(unsigned char uart_number, 
                                unsigned int baud_rate) {
  U1MODEbits.BRGH = 1;		// configure for high precision baud rate
  // configure baud rate for 9600[pulses/s]
	// UxBRG = F_CY / (4 * desired_baud_rate) - 1 (see p.190 of datasheet)
	//       = 32MHz / (4 * 9600) - 1
  //       ~= 832
  U1BRG = 832;
}

/*---------------------------End of File-------------------------------------*/
