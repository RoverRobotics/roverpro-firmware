/*****************************************************************************
File: UART.h

Description: This module encapsulates the interface to the UART1 hardware
  module.
  
Notes:
  1) assumes a 32MHz external oscillator
  2) only UART1 is used, with the following pinout:
      Peripheral	PIC Pin
      U1TX				  53
      U1RX				  52
  3) see test harness for example usage
 *****************************************************************************/
#ifndef UART_H
#define UART_H

/*---------------------------Public Functions--------------------------------*/
/*****************************************************************************
  Function: InitUART()
	Parameters:
	  unsigned char Tx_pin, remappable pin number to assign to tranmission
	  unsigned char Rx_pin, remappable pin number to assign to reception 
		unsigned long int baud_rate, baud rate at which to communicate [pulse/s] 
		                        (9600 or 115200)
 *****************************************************************************/
void InitUART(unsigned char Tx_pin, unsigned char Rx_pin, 
              unsigned long int baud_rate);

/*****************************************************************************
	Function: IsTxClear()
  Returns
    char, boolean whether the transmission register is free
  Description: Returns whether U1TX can accept another byte
 *****************************************************************************/
char inline IsTxClear(void);

/*****************************************************************************
  Function: TransmitByte()
  Parameters:
    unsigned char message, the byte to send
  Description: Writes the given byte to the transmission register, U1TX
 *****************************************************************************/
void inline TransmitByte(unsigned char message);

/*****************************************************************************
  Function: IsRxAvailable()
  Returns:
    char, boolean whether data is available in U1RX
  Description: Returns whether new data is available in U1RX
 *****************************************************************************/
char inline IsRxAvailable(void);

/*****************************************************************************
  Function: GetRxByte()
  Returns:
    char, byte received from U1RX
  Description: Returns the last byte from U1RX
 *****************************************************************************/
char inline GetRxByte(void);

#endif
/*--------------------------------End of file--------------------------------*/
