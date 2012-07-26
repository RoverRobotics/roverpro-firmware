/*******************************************************************************
File: I2C.h

Description: This module provides an interface to the I2C1 hardware module on
  the PIC24FJ256GB106 as a master in single-master mode.
  Common peripheral devices on an I2C bus include serial EEPROMs, 
  display drivers and A/D Converters.
  
  As currently configured, this module supports:
    - single-master operation
    - 7-bit addressing (can support 128 devices on the I2C bus)
    - 1 byte transmission (non-sequential write/read)
    - can NOT communicate with devices on same bus with same address
    
Notes:
	- N/A
*******************************************************************************/
#ifndef I2C_H
#define I2C_H
/*---------------------------Type Definitions---------------------------------*/
// baud rate options for I2C
typedef enum {
	kBaudRate100kHz = 0,
	kBaudRate400kHz,
	kBaudRate1MHz,
} BaudRate;

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: I2C_Init
Parameters:
  BaudRate baudRate,   the baud rate at which to communicate
*******************************************************************************/
void I2C_Init(const BaudRate baudRate);


/*******************************************************************************
Function: I2C_RequestData
Parameters:
	char deviceAddress,   the slave address from whom to request the data
// WARNING: DOESN'T PUT IN REQUEST IF THE BUS IS NOT IDLE
// TODO: add a FIFO queue for different device addresses
*******************************************************************************/
void I2C_RequestData(const char deviceAddress);


/*******************************************************************************
Function: I2C_IsBusIdle
*******************************************************************************/
inline unsigned char I2C_IsBusIdle(void);

/*******************************************************************************
Function: I2C_IsDataAvailable
*******************************************************************************/
inline unsigned char I2C_IsNewDataAvailable(const char deviceAddress);


/*******************************************************************************
Function: I2C_GetData
*******************************************************************************/
inline int I2C_GetData(const char deviceAddress);


/*******************************************************************************
Function: I2C_IsDeviceOnBus
Parameters:
	char deviceAddress,   the slave address from whom to request the data
*******************************************************************************/
//unsigned char I2C_IsDeviceOnBus(const char deviceAddress);

#endif
