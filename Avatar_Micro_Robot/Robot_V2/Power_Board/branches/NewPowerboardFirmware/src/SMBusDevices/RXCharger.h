/*******************************************************************************
File: RXCharger.h

Description: This module provides macros to facilitate access to configuration
  bits and registers within the I2C-interfaced RoboteX internal charger.
*******************************************************************************/
#ifndef RX_CHARGER_H
#define RX_CHARGER_H

/*---------------------------Macros-------------------------------------------*/
#define RXCHARGER_ADDRESS         0x0C

// sub-addresses from which to request data (read-only)
#define RXCHARGER_STATUS          0xCA  // whether the Charger is Alive

// pre-defined responses
#define RXCHARGER_ALIVE           0xDA  // the charger has made stable contact
                                        // with the Dock and is Alive
#endif



