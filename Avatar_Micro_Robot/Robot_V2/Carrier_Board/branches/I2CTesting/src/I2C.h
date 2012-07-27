/*******************************************************************************
File: I2C.h

Description: This module provides an interface to the I2C1 hardware module on
  the PIC24FJ256GB106 as a master in single-master mode.
  Common peripheral devices on an I2C bus include serial EEPROMs, 
  display drivers and A/D Converters.
  
  As currently configured, this module supports:
    - single-master operation
    - 7-bit addressing (can support 128 devices on the I2C bus)
    - only can read (no writing yet)
    - can NOT communicate with devices on same bus with same address
    
Notes:
	- N/A
*******************************************************************************/
#ifndef I2C_H
#define I2C_H
/*---------------------------Type Definitions---------------------------------*/
#define I2C_MAX_DATA_LENGTH   5   // the maximum number of bytes a device on 
                                  // the I2C bus would ever transmit (used to 
                                  // obviate the need to dynamically manage
                                  // memory internally)  DO NOT make larger
                                  // than 127
typedef struct {
  char address;
  char subaddress;            // the aspect of the sensor that contains the data 
                              // Other subaddresses may point to a register
                              // where the sensor can be configured or where
                              // other outputs the sensor is capable of
                              // producing  are stored (e.g. multiple
                              // acceleration directions)
  char numDataBytes;          // number of data bytes the sensor returns
  char data[];
} I2CDevice;

// baud rate options for I2C
typedef enum {
	kI2CBaudRate100kHz = 0,
	kI2CBaudRate400kHz,
	kI2CBaudRate1MHz,
} I2CBaudRate;

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: I2C_Init
Parameters:
  I2C_BaudRate baudRate,  the baud rate at which to communicate
*******************************************************************************/
void I2C_Init(const I2CBaudRate baudRate);


/*******************************************************************************
Function: I2C_RequestData
Parameters:
	I2CDevice myDevice,     the I2C-interfaced device on which to operate
*******************************************************************************/
void I2C_RequestData(const I2CDevice myDevice);


/*******************************************************************************
Function: I2C_IsBusIdle
*******************************************************************************/
inline unsigned char I2C_IsBusIdle(void);


/*******************************************************************************
Function: I2C_IsNewDataAvailable
*******************************************************************************/
inline unsigned char I2C_IsNewDataAvailable(void);


/*******************************************************************************
Function: I2C_GetData
Parameters:
	I2CDevice *device,      pointer to the device on which to operate
*******************************************************************************/
void I2C_GetData(I2CDevice *pDevice);


/*******************************************************************************
Function: I2C_WriteData
Parameters:
	I2CDevice device,       the I2C-interfaced device on which to operate
	char data[],            pointer to a statically-declared array of the bytes
	                        to write
*******************************************************************************/
void I2C_WriteData(const I2CDevice device, const char data[]);


/*******************************************************************************
Function: I2C_ErrorHasOcurred
*******************************************************************************/
unsigned char I2C_ErrorHasOccurred(void);


/*******************************************************************************
Function: I2C_RefreshModule
*******************************************************************************/
void I2C_RefreshModule(void);


/*******************************************************************************
Function: I2C_IsDeviceOnBus
Parameters:
	I2CDevice myDevice,     the I2C-interfaced device on which to operate
*******************************************************************************/
//unsigned char I2C_IsDeviceOnBus(const I2CDevice myDevice);

#endif
