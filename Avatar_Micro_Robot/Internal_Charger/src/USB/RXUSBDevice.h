/*==============================================================================
File: RXUSBDevice.h
 
Description: This module provides a USB device interface built on microchip's
  USB stack.

Notes:
  - see also: usb_config.h, registers.h
  - see also: http://www.microchip.com/stellent/idcplg?IdcService=SS_GET_PAGE&nodeId=2680&dDocName=en537044

Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#ifndef RXUSBDevice_H
#define RXUSBDevice_H

/*---------------------------Dependencies-------------------------------------*/
#include <stdint.h>            // for uint16_t definition
#include "./USB/usb_config.h"  // to give scope to firmware/software-shared 
                               // registers which act as our interface to
                               // USB communication (TODO: clean this up!)
                               // BUG ALERT: this must be BEFORE usb.h to 
                               // configure through #define's

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: RXUSBDevice_Init
Description: Initializes the microcontroller as a USB, full-speed device.
Parameters:
  uint16_t productID, the product identification number (e.g. 0x0012)
*******************************************************************************/
void RXUSBDevice_Init(uint16_t productID);


/*******************************************************************************
Function: RXUSBDevice_ProcessMessage
Description: Intended to be placed in the main loop, this function processes
  incoming and outgoing messages.  The results of incoming messages are placed
  in the relevant registers defined in registers.h.  Outgoing messages are
  constructed from data placed in the relevant output registers.
*******************************************************************************/
void RXUSBDevice_ProcessMessage(void);

#endif
