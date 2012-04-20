/*=============================================================================

File: UART.c
Description: Provides an interface to the UART hardware modules.
Notes:
F_OSC = 32MHz (given our 20MHz crystal, dividy-by-5 from PLL, and 
               multiply-by-8 from somewhere)
F_CY = F_OSC / 2 = 16MHz
=============================================================================*/
//#define TEST_UART

/*---------------------------Dependencies------------------------------------*/
#include "./UART.h"
#include "./PPS.h"
#include <p24FJ256GB106.h>

// make ISR's an empty function by default
void DummyISR(void);
void (*U1TX_UserISR)(void) = DummyISR;
void (*U1RX_UserISR)(void) = DummyISR;

/*---------------------------Helper Function Prototypes----------------------*/
static void ConfigureBaudRate(unsigned long int baud_rate);

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_UART
#include "./Pause.h"
#include "./Protocol.h"
#include "./ConfigurationBits.h"

#define RS485_OUTEN_EN(a)     (_TRISD6 = !(a))
#define RS485_OUTEN_ON(a)     (_LATD6 = (a))

int main(void) {
  /*
  // test BuildPacket()
  unsigned char data_out[BASE_DATA_LENGTH] = {0};
  unsigned char packet[MAX_PACKET_LENGTH] = {0};
  unsigned char packet_length = 0;
  data_out[0] = 0xBE; data_out[1] = 0xEF; data_out[2] = 0xEE; data_out[3] = 0xFF;
  BuildPacket(BASE, data_out, packet, &packet_length);
  */
  
  /*
  // test GetData()
  unsigned char data_in[BASE_DATA_LENGTH] = {0};
  unsigned char data_length = 0;
  GetData(packet, data_in, &data_length);
  */
  
  InitUART(20, 25, 9600);
  RS485_OUTEN_EN(1);
  RS485_OUTEN_ON(0);  // 0 for Rx
  
  while (1) {
    /*
    // test slow transmission, inspecting result in debugger
    unsigned char i = 0;
    for (i = 0; i < packet_length; i++) {
      TransmitByte(packet[i]);
      Pause(1000);
    }
    */
    
    // if the time between reception is excessively long
    // clear the error bits, reset anything
    // ensure framing error and parity error bits are cleared
 	  // in case we get an error that can disable the interrupt
 	  // THESE ARE READ ONLY :( U1STAbits.PERR = 0; U1STAbits.FERR = 0; 
 	
    // test reception from python script--can we get in here?
    if (received_something) {
      received_something = 0;
      unsigned char dummy = 1;
      unsigned char dummy2 = 2;
    }
    
  }
  
  return 0;
}
#endif
/*---------------------------End Test Harness--------------------------------*/

/*---------------------------Public Function Definitions---------------------*/
/*
Notes:
	- by default, configures to 1 stop bit, 8-bit transmission, no parity
*/
void InitUART(unsigned char Tx_pin, unsigned char Rx_pin, 
              unsigned long int baud_rate) {
	MapPeripheral(Tx_pin, OUTPUT, FN_U1TX);
	MapPeripheral(Rx_pin, INPUT, FN_U1RX);
	ConfigureBaudRate(baud_rate);
	
	_U1RXIP = 6;            // configure interrupt priority
	_U1TXIP = 5;            // Note: 7 is highest priority interrupt
	
	_U1TXIF = 0;            // begin with any interrupt flags cleared
	_U1RXIF = 0;

	U1MODEbits.UARTEN = 1;  // enable UART1
	Nop();
	U1STAbits.UTXEN = 1;    // enable transmission
	// BUG ALERT: the UxTXIF bit is set when the module is first enabled
	
	IEC0bits.U1TXIE = 1;    // enable UART1 Tx interrupt
	IEC0bits.U1RXIE = 1;    // enable UART1 Rx interrupt
}

void inline TransmitByte(unsigned char message) {
  U1TXREG = message;
}

char inline GetRxByte(void) {
  return (char)U1RXREG;
}

void DummyISR(void) {
}
  
void  __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt(void) {
 	U1TX_UserISR();
}

void  __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void) {
 	U1RX_UserISR();
}


/*---------------------------Private Function Definitions--------------------*/
/*
Notes:
  - see also Table 21-2 of PIC24F family reference manual
  - UxBRG = F_CY / (16 * desired_baud_rate) - 1 (see p.200 of datasheet)
          = 16MHz / (4 * 9600) - 1
         ~= 416
*/
static void ConfigureBaudRate(unsigned long int baud_rate) {
  U1MODEbits.BRGH = 1;		// configure for high precision baud rate
  switch (baud_rate) {
    case 9600: U1BRG = 416; break;  // 0.07% error
    case 115200: U1BRG = 34; break; // 0.62% error
  }
  
}

/*---------------------------End of File-------------------------------------*/
