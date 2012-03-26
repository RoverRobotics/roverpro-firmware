/****************************************************************************
  File: UART.h

  Description: This module encapsulates the interface to the UART hardware
    modules.
    
  Notes:
    1) only UART1 is used, with the following pinout:
        Peripheral	Pin
        U1TX				31
        U1RX				32
		2) TODO: generalize for use with any UART hardware module (1, 2,...4)
 
 ****************************************************************************/
#ifndef UART_H
#define UART_H

/*---------------------------Public Functions--------------------------------*/
/****************************************************************************
  Function: InitUART()
	Parameters:
		unsigned char uart_number, which hardware module to initialize
  Description: Initializes the given UART communication module.
 ****************************************************************************/
void InitUART(unsigned char uart_number);

/****************************************************************************
	Function: IsTxClear()
  Returns
    char, boolean whether the transmission register is free
  Description: Returns whether U1TX can accept another byte.
 ****************************************************************************/
char IsTxClear(void);

/****************************************************************************
  Function: TransmitByte()
  Parameters:
    unsigned char message, the byte to send
  Description: Writes the given byte to the transmission register, U1TX.
 ****************************************************************************/
void TransmitByte(unsigned char message);

/****************************************************************************
  Function: IsRxAvailable()
  Returns:
    char, boolean whether data is available in U1RX.
  Description: Returns whether new data is available in U1RX.
 ****************************************************************************/
char IsRxAvailable(void);

/****************************************************************************
  Function: GetRxByte()
  Returns:
    char, byte received from U1RX.
  Description: Returns the last byte from U1RX
 ****************************************************************************/
char GetRxByte(void);

#endif
/*--------------------------------End of file--------------------------------*/
