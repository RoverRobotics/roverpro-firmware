/*******************************************************************************
File: I2C.h

Description: This module provides an interface to the I2C hardware modules on
  the PIC24FJ256GB106 as a master in single-master mode.  Each module can be 
  initialized to conform to I2C or SMBus standards.
  
  As currently configured, this module supports:
    - single-master operation
    - 7-bit addressing (can support 128 devices on the I2C bus)
    
Responsible Engineer(s): Stellios Leventis (sleventis@robotex.com)
*******************************************************************************/
#ifndef TWI_H
#define TWI_H

/*---------------------------Dependencies-------------------------------------*/
#include "./StandardHeader.h"

/*---------------------------Macros-------------------------------------------*/
#define TWI_MAX_DATA_LENGTH   5   // the maximum number of bytes a device on 
                                  // the bus would ever transmit (used to 
                                  // obviate the need to dynamically manage
                                  // memory internally).  DO NOT make larger
                                  // than 127
#define SMBUS                 1
#define I2C                   0

/*---------------------------Type Definitions---------------------------------*/
typedef struct {
  uint8_t address;
  uint8_t subaddress;   // register in the sensor that contains the data 
                        // Other subaddresses may point to a register
                        // where the sensor can be configured or where
                        // other outputs the sensor is capable of
                        // producing  are stored (e.g. multiple
                        // acceleration directions)
  uint8_t n_data_bytes; // number of data bytes the sensor returns
  uint8_t *data;
} TWIDevice;

// baud rate options
typedef enum {
	kTWIBaudRate100kHz = 0,
	kTWIBaudRate400kHz,
	kTWIBaudRate1MHz
} kTWIBaudRate;

// I2C hardware module options
typedef enum {
	kTWI01 = 0,
	kTWI02 = 1,
	kTWI03 = 2
} kTWIModule;

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: TWI_Init
Parameters:
  kTWIModule module,     which of the available hardware modules  
  kTWIBaudRate baudRate, the baud rate at which to communicate
	bool isSMBus,          whether SMBus thresholds should be used
*******************************************************************************/
void TWI_Init(const kTWIModule module,
              const kTWIBaudRate baudRate, 
              const bool isSMBus);


/*******************************************************************************
Function: TWI_RequestData
Parameters:
  kTWIModule module,     which of the available hardware modules
	TWIDevice* device,     the device on the bus on which to operate
*******************************************************************************/
void TWI_RequestData(const kTWIModule module, const TWIDevice* device);


/*******************************************************************************
Function: TWI_IsBusIdle
Parameters:
  kTWIModule module,     which of the available hardware modules
*******************************************************************************/
inline bool TWI_IsBusIdle(const kTWIModule module);


/*******************************************************************************
Function: TWI_IsNewDataAvailable
Parameters:
  kTWIModule module,     which of the available hardware modules
*******************************************************************************/
inline bool TWI_IsNewDataAvailable(const kTWIModule module);


/*******************************************************************************
Function: TWI_GetData
Parameters:
  kTWIModule module,     which of the available hardware modules
	TWIDevice* device,     the device on the bus on which to operate
*******************************************************************************/
void TWI_GetData(const kTWIModule module, const TWIDevice* device);


/*******************************************************************************
Function: TWI_WriteData
Parameters:
  kTWIModule module,     which of the available hardware modules
	TWIDevice* device,     the device on the bus on which to operate
	uint8_t data[],        pointer to a statically-declared array of the bytes
	                       to write
*******************************************************************************/
void TWI_WriteData(const kTWIModule module, 
                   const TWIDevice* device, 
                   const uint8_t data[]);


/*******************************************************************************
Function: TWI_ErrorHasOcurred
Parameters:
  kTWIModule module,     which of the available hardware modules
*******************************************************************************/
bool TWI_ErrorHasOccurred(const kTWIModule module);


/*******************************************************************************
Function: TWI_RefreshModule
Parameters:
  kTWIModule module,     which of the available hardware modules
*******************************************************************************/
void TWI_RefreshModule(const kTWIModule module);


/*******************************************************************************
Function: TWI_Deinit
Parameters:
  kTWIModule module,     which of the available hardware modules
Description: Deinitializes this module, restoring any resources and/or pins 
  that were allocated during initialization.
*******************************************************************************/
void TWI_Deinit(const kTWIModule module);

#endif
