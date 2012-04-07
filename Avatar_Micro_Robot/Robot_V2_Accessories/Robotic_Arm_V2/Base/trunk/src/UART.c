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
#include "./UART.h"
#include "./PPS.h"
#include "./ConfigurationBits.h"
#include <p24FJ256GB106.h>

/*---------------------------Macros and Type Definitions---------------------*/
// TODO: move these out of this module
#define RS485_OUTEN_EN(a)     (_TRISD6 = !(a))
#define RS485_OUTEN_ON(a)     (_LATD6 = a)

/*---------------------------Helper Function Prototypes----------------------*/
static void ConfigureBaudRate(unsigned long int baud_rate);
static void U1TX_ISR(void);
static void U1RX_ISR(void);

/*---------------------------Module Variables--------------------------------*/

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_UART
#include "./Pause.h"

int main(void) {
  unsigned char dummy_message = 0;
  
  InitUART(20, 25, 9600);
  
  while (1) {
    /*
    TransmitByte(dummy_message++);
    Pause(1000);
    */
    
		// Testing Reception AND Transmission:
		// Setup: python messaging script => USB-to-UART-to-RS485 adapter =>
		//    RS485 IC => UART within PIC24FJ 
    
    if (IsRxAvailable()) {
      TransmitByte(GetRxByte());
      // probe reception via to verify incoming data
      // probe transmission via to verify full loop
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
	
	IEC0bits.U1TXIE = 1;    // enable UART1 Tx interrupt
	IEC0bits.U1RXIE = 1;    // enable UART1 Rx interrupt
	U1MODEbits.UARTEN = 1;  // enable UART1
	U1STAbits.UTXEN = 1;    // enable transmission
	
  // TODO: remove this after confirming test
  RS485_OUTEN_EN(1); 
  RS485_OUTEN_ON(1);
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

void  __attribute__((__interrupt__, auto_psv)) _U1TXInterrupt(void) {
 	U1TX_ISR();
}


void  __attribute__((__interrupt__, auto_psv)) _U1RXInterrupt(void) {
 	U1RX_ISR();
}

/*---------------------------Private Function Definitions--------------------*/
/*
Notes:
  - UxBRG = F_CY / (4 * desired_baud_rate) - 1 (see p.188 of datasheet)
          = 16MHz / (4 * 9600) - 1
         ~= 415
*/
static void ConfigureBaudRate(unsigned long int baud_rate) {
  U1MODEbits.BRGH = 1;		// configure for high precision baud rate
  switch (baud_rate) {
    case 9600: U1BRG = 415; break;
    case 115200: U1BRG = 34; break;
  }
}

static void U1TX_ISR(void) {
  
  _U1TXIF = 0;  // clear the source of the interrupt
}

static void U1RX_ISR(void) {
  _U1RXIF = 0;  // clear the source of the interrupt
}















/*
void RS485_TX_ISR(void) {
  static unsigned char message_index = 0;
  _U1TXIF = 0;
 
  //start at 1, since we already sent the first byte
  message_index++;

  if (message_index >= RS485_TX_LENGTH) {
    message_index = 0;
    _U1TXIE = 0;
    rs485_transmitting = 0;
    return;
  }

  U1TXREG = rs485_tx_buffer[message_index];
}


void RX_ISR(void) {
  static unsigned char message_index = 0;
  static unsigned char current_length = 100;
  static unsigned char current_device = 0x00;
  unsigned char i;
  _U1RXIF = 0;

  //if we've gotten to the end of a message
  if (message_index >= current_length) {
    if (current_device == DEVICE_ARM_BASE) {
      for (i = 0; i < RS485_BASE_LENGTH; i++) {
        rs485_base_message[i] = rs485_rx_buffer[i];
      }
      rs485_base_message_ready = 1;
    }
    else if (current_device == DEVICE_ARM_LINK1) {
      for (i = 0; i < RS485_LINK1_LENGTH; i++) {
        rs485_link1_message[i] = rs485_rx_buffer[i];
      }
      rs485_link1_message_ready = 1;
    }
    current_device = 0x00;
    message_index = 0;
    current_length = 100;
    return;
  }

  rs485_rx_buffer[message_index] = U1RXREG;

  switch (message_index) {
    case 0x00:
      if (rs485_rx_buffer[0] != 0xff) {
        message_index = 0;
        return;
      }
      break;
    case 0x01:
      if (rs485_rx_buffer[1] != 0xcc) {
        message_index = 0;
        return;
      }
      break;
    case 0x02:
      //if we're dealing with a message from the base
      if (rs485_rx_buffer[2] == 0x0b) {
        current_length = RS485_BASE_LENGTH;
        current_device = DEVICE_ARM_BASE;
      } else if (rs485_rx_buffer[2] == 0x0c) { // if message from link 1
        current_length = RS485_LINK1_LENGTH;
        current_device = DEVICE_ARM_LINK1;
      } else { // otherwise it's an invalid device
        message_index = 0;
        return;
      }
    break;
  }
  message_index++;
}
*/

/*---------------------------End of File-------------------------------------*/
