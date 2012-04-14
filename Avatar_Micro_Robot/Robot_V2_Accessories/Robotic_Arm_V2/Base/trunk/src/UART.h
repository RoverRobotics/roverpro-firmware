/******************************************************************************
File: UART.h

Description: This module encapsulates the interface to the UART1 hardware
  module.
  
Notes:
  - assumes an instruction cycle clock of F_CY = 16MHz
 *****************************************************************************/
#ifndef UART_H
#define UART_H

/*---------------------------Public Functions---------------------------------*/
/******************************************************************************
Function: InitUART()
Parameters:
  unsigned char Tx_pin, remappable pin number to assign to tranmission
  unsigned char Rx_pin, remappable pin number to assign to reception 
	unsigned long int baud_rate, baud rate at which to communicate [pulse/s] 
		                        (9600 or 115200)
******************************************************************************/
void InitUART(unsigned char Tx_pin, unsigned char Rx_pin, 
              unsigned long int baud_rate);

/******************************************************************************
Function: TransmitByte()
Parameters:
  unsigned char message, the byte to send
Description: Writes the given byte to the transmission register, U1TX
*******************************************************************************/
void inline TransmitByte(unsigned char message);

/******************************************************************************
Function: GetRxByte()
Returns:
  char, byte received from U1RX
Description: Returns the last byte from U1RX
******************************************************************************/
char inline GetRxByte(void);

/*****************************************************************************
Usage: U1Tx_UserISR = MyU1TxISRFunctionName;
Usage: U1Rx_UserISR = MyU1RxISRFunctionName;
******************************************************************************/
void DummyISR(void);
extern void (*U1TX_UserISR)(void);
extern void (*U1RX_UserISR)(void);

#endif
/*--------------------------------End of file--------------------------------*/
