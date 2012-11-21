/*******************************************************************************
File: TWISlave.h

Description: This module provides an interface to the I2C hardware modules on
  the PIC24FJ256GB106 as a slave.  Each module can be initialized to conform 
  to I2C or SMBus standards.  The responses are defined by assigning a 
  user-defined function to TWISlave_GetResponse() (see example usage below). 
  
  As currently configured, this module supports:
    - 7-bit addressing (can support 128 devices on the I2C bus)
    - write-word/read-word communication
    
Responsible Engineer(s): Stellios Leventis (sleventis@robotex.com)
*******************************************************************************/
#ifndef TWI_SLAVE_H
#define TWI_SLAVE_H

/*---------------------------Dependencies-------------------------------------*/
#include "./StandardHeader.h"

/*---------------------------Macros-------------------------------------------*/
//#define SMBUS                 1
//#define I2C                   0

/*---------------------------Type Definitions---------------------------------*/
// I2C hardware module options
typedef enum {
	kTWISlave01 = 0,
	kTWISlave02,
	kTWISlave03,
} kTWISlaveModule;

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: TWISlave_Init
Parameters:
  kTWISlaveModule module, which of the available hardware modules  
  uint8_t slave_address,  the slave address to give this module
	bool is_SMBus,          whether SMBus thresholds should be used
*******************************************************************************/
void TWISlave_Init(const kTWISlaveModule module,
                   const uint8_t slave_address, 
                   const bool is_SMBus);

/*******************************************************************************
Usage: I2C1_UserProtocol = MyI2C1FunctionName;  // assign the function

// implement your desired transmission response
static uint8_t MyI2C1FunctionName(const uint8_t command, 
                                  const uint8_t response_index) {
  switch (command) {
    case 0xAC:
      if (response_index == 0) return 0x03;
      else if (response_index == 1) return 0x06;
    default: return 0xFF;
  }
}
*******************************************************************************/
extern uint8_t (*I2C1_UserProtocol)(const uint8_t command, 
                                    const uint8_t response_index);
extern uint8_t (*I2C2_UserProtocol)(const uint8_t command, 
                                    const uint8_t response_index);
extern uint8_t (*I2C3_UserProtocol)(const uint8_t command, 
                                    const uint8_t response_index);

#endif
