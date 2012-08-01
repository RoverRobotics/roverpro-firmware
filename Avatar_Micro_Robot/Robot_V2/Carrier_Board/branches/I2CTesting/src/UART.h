/*******************************************************************************
File: UART.h

Description: This module encapsulates the interface to the UART1 hardware
  module.  Since both transmission and reception are interrupt-based, this
  module additionally provides the ability to define external interrupt 
  service routines (see U1Tx_UserISR usage below). 
  
	TODO: TEST THIS AGAIN, MADE CHANGES!!!
	
Notes:
  - assumes an instruction cycle clock of F_CY = 16MHz
*******************************************************************************/
#ifndef UART_H
#define UART_H

// UART baud rate options, used during initialization
typedef enum {
	kUARTBaudRate9600,
	kUARTBaudRate57600,
	kUARTBaudRate115200
} UARTBaudRate;

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: InitUART
Parameters:
  unsigned char Tx_pin, remappable pin number to assign to tranmission
  unsigned char Rx_pin, remappable pin number to assign to reception 
	UARTBaudRate baudRate, baud rate at which to communicate [pulse/s]
*******************************************************************************/
void UART_Init(unsigned char Tx_pin, unsigned char Rx_pin, 
               UARTBaudRate baudRate);


/*******************************************************************************
Function: UART_TransmitByte
Parameters:
  unsigned char byte, the byte to send
Description: Writes the given byte to the transmission register, U1TX
*******************************************************************************/
void inline UART_TransmitByte(unsigned char byte);


/*******************************************************************************
Function: UART_GetRxByte
Returns:
  char, byte received from U1RX
Description: Returns the last byte from U1RX
*******************************************************************************/
char inline UART_GetRxByte(void);


/*******************************************************************************
Usage: U1Tx_UserISR = MyU1TxISRFunctionName;
Usage: U1Rx_UserISR = MyU1RxISRFunctionName;
*******************************************************************************/
extern void (*U1TX_UserISR)(void);
extern void (*U1RX_UserISR)(void);

#endif
