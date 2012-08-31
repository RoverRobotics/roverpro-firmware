/*******************************************************************************
File: NM33.h

Description: Provides an interface to the NM33 fish-eye Lens Camera,
  encapsulating the UART-based protocol.

Notes:
  - consumes UART1 hardware module
  - it is the responsibility of the user to not exceed the pan/tilt/zoom limits
*******************************************************************************/
#ifndef NM33_H
#define NM33_H

/*---------------------------Dependencies-------------------------------------*/
#include <stdint.h>

/*---------------------------Type Definitions---------------------------------*/
typedef enum {
	kNM33LimitsMinPan = 0,
	kNM33LimitsMaxPan = 200,//359, // TODO: how can we tranmist 359 with a uchar?
	kNM33LimitsMinTilt = 0,
	kNM33LimitsMaxTilt = 90,
	kNM33LimitsMinZoom = 10,
	kNM33LimitsMaxZoom = 130
} kNM33Limits;

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: NM33_Init
Parameters:
  uint8_t txPin,  transmission re-programmable pin number (n of RPn)
  uint8_t rxPin,  reception re-programmable pin number (n of RPn)
Description: Initializes the communication interface and camera.
*******************************************************************************/
void NM33_Init(uint8_t txPin, uint8_t rxPin);


/*******************************************************************************
Function: NM33_SetLocation
Parameters:
  uint8_t pan,
  uint8_t tilt,
  uint8_t zoom,
*******************************************************************************/
void NM33_SetLocation(uint8_t pan, uint8_t tilt, uint8_t zoom);


/*******************************************************************************
Function: NM33_Deinit
Description: Deinitializes this module, restoring any resources and/or pins 
  that were allocated during initialization.
*******************************************************************************/
//void NM33_Deinit(void);

#endif
