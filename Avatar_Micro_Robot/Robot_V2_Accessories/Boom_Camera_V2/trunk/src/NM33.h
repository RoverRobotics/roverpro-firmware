/*******************************************************************************
File: NM33.h

Description: Provides an interface to the NM33 fish-eye Lens Camera,
  encapsulating the UART-based protocol.

Notes:
  - consumes UART1 hardware module
  - do NOT exceed the pan/tilt/zoom limits
*******************************************************************************/
#ifndef NM33_H
#define NM33_H

/*---------------------------Dependencies-------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/*---------------------------Type Definitions---------------------------------*/
typedef enum {
	kNM33LimitMinPan = 0,
	kNM33LimitMaxPan = 359,
	kNM33LimitMinTilt = 0,
	kNM33LimitMaxTilt = 90,
	kNM33LimitMinZoom = 10,
	kNM33LimitMaxZoom = 130
} kNM33Limit;


#define DEFAULT_PAN               90
#define DEFAULT_TILT              80
#define DEFAULT_ZOOM              100

/*---------------------------Public Functions---------------------------------*/
/*******************************************************************************
Function: NM33_Init
Parameters:
  uint8_t txPin,  transmission re-programmable pin number (n of RPn)
  uint8_t rxPin,  reception re-programmable pin number (n of RPn)
Description: Initializes the communication interface and camera.
Notes:
  - BLOCKING! - it delays to guarantee the camera has had sufficient time to
    boot (about 1.5s)
*******************************************************************************/
void NM33_Init(uint8_t txPin, uint8_t rxPin);


/*******************************************************************************
Function: NM33_IsReceptive
Returns:
  bool,           whether the camera is available to receive another command
*******************************************************************************/
bool NM33_IsReceptive(void);


/*******************************************************************************
Function: NM33_set_location
Parameters:
  uint16_t pan,   NOTE: THE PAN IS OF TYPE UINT16!
  uint8_t tilt,
  uint8_t zoom
*******************************************************************************/
void NM33_set_location(uint16_t pan, uint8_t tilt, uint8_t zoom);


/*******************************************************************************
Function: NM33_Deinit
Description: Deinitializes this module, restoring any resources and/or pins 
  that were allocated during initialization.
*******************************************************************************/
void NM33_Deinit(void);

#endif
