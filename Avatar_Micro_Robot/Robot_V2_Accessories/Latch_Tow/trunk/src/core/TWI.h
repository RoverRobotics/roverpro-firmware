/*******************************************************************************
File: I2C.h

Description: This module provides an interface to the I2C1 hardware module on
  the PIC24FJ256GB106 as a master in single-master mode.  It can be initialized
	to conform to I2C or SMBus standards.
  
  As currently configured, this module supports:
    - single-master operation
    - 7-bit addressing (can support 128 devices on the I2C bus)
*******************************************************************************/
#ifndef TWI_H
#define TWI_H
/*---------------------------Type Definitions---------------------------------*/
#define TWI_MAX_DATA_LENGTH   5   // the maximum number of bytes a device on 
                                  // the bus would ever transmit (used to 
                                  // obviate the need to dynamically manage
                                  // memory internally).  DO NOT make larger
                                  // than 127
typedef struct {
  unsigned char address;
  unsigned char subaddress;   // register in the sensor that contains the data 
                              // Other subaddresses may point to a register
                              // where the sensor can be configured or where
                              // other outputs the sensor is capable of
                              // producing  are stored (e.g. multiple
                              // acceleration directions)
  unsigned char numDataBytes; // number of data bytes the sensor returns
  unsigned char *data;
} TWIDevice;

// baud rate options
typedef enum {
	kBaudRate100kHz = 0,
	kBaudRate400kHz,
	kBaudRate1MHz,
} BaudRate;

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: TWI_Init
Parameters:
  BaudRate baudRate,  the baud rate at which to communicate
	unsigned char isSMBus, whether SMBus thresholds should be used instead
*******************************************************************************/
void TWI_Init(const BaudRate baudRate, const unsigned char isSMBus);


/*******************************************************************************
Function: TWI_RequestData
Parameters:
	TWIDevice *device,     the device on the bus on which to operate
*******************************************************************************/
void TWI_RequestData(const TWIDevice *device);


/*******************************************************************************
Function: TWI_IsBusIdle
*******************************************************************************/
inline unsigned char TWI_IsBusIdle(void);


/*******************************************************************************
Function: TWI_IsNewDataAvailable
*******************************************************************************/
inline unsigned char TWI_IsNewDataAvailable(void);


/*******************************************************************************
Function: TWI_GetData
Parameters:
	TWIDevice *device,     the device on the bus on which to operate
*******************************************************************************/
void TWI_GetData(TWIDevice *device);


/*******************************************************************************
Function: TWI_WriteData
Parameters:
	TWIDevice *device,     the device on the bus on which to operate
	unsigned char data[],  pointer to a statically-declared array of the bytes
	                       to write
*******************************************************************************/
void TWI_WriteData(const TWIDevice *device, const unsigned char data[]);


/*******************************************************************************
Function: TWI_ErrorHasOcurred
*******************************************************************************/
unsigned char TWI_ErrorHasOccurred(void);


/*******************************************************************************
Function: TWI_RefreshModule
*******************************************************************************/
void TWI_RefreshModule(void);


/*******************************************************************************
Function: TWI_Deinit
Description: Deinitializes this module, restoring any resources and/or pins 
  that were allocated during initialization.
*******************************************************************************/
void TWI_Deinit(void);

#endif
