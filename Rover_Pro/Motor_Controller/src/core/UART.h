/*******************************************************************************
File: UART.h

Description: This module encapsulates the interface to the UART1 hardware
  module.  Since both transmission and reception are interrupt-based, this
  module additionally provides the ability to define external interrupt 
  service routines (see U1Tx_UserISR usage below). 
	
Notes:
	- BUG ALERT: when writing custom ISR's, know that the UxTXIF bit is set 
	  when the module is first enabled
  - assumes an instruction cycle clock of F_CY = 16MHz
*******************************************************************************/
#ifndef UART_H
#define UART_H

//---------------------------Dependencies---------------------------------------
#include <stdint.h>

// UART baud rate options, used during initialization
typedef enum {
	kUARTBaudRate9600,
	kUARTBaudRate57600,
	kUARTBaudRate115200
} UARTBaudRate;

//---------------------------Public Functions-----------------------------------
// Function: InitUART
// Parameters:
//   uint8_t Tx_pin,         remappable pin number to assign to tranmission
//   uint8_t Rx_pin,         remappable pin number to assign to reception 
// 	UARTBaudRate baud_rate, baud rate at which to communicate [pulse/s]
void UART_Init(uint8_t Tx_pin, uint8_t Rx_pin, 
               UARTBaudRate baud_rate);


// Function: UART_TransmitByte
// Parameters:
//   uint8_t byte, the byte to send
// Description: Writes the given byte to the transmission register, U1TX
void inline UART_TransmitByte(uint8_t byte);


// Function: UART_GetRxByte
// Returns:
//   uint8_t, byte received from U1RX
// Description: Returns the last byte from U1RX
uint8_t inline UART_GetRxByte(void);


// Function: UART_Deinit
// Description: Deinitializes this module, restoring any resources and/or pins 
//  that were allocated during initialization.
void UART_Deinit(void);


// Usage: U1Tx_UserISR = MyU1TxISRFunctionName;
// Usage: U1Rx_UserISR = MyU1RxISRFunctionName;
extern void (*U1TX_UserISR)(void);
extern void (*U1RX_UserISR)(void);

#endif
