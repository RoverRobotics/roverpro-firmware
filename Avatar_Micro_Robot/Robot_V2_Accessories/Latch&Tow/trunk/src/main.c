/*=============================================================================
File: Main.c

Description: Overarching file for a generic project.
  
Notes:
  - adapted from a general-purpose file originally assembled by J. Brinton
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
=============================================================================*/
#define TEST_MAIN
/*---------------------------Dependencies------------------------------------*/
#include "./stdhdr.h"
#include "../../../../../Common/REV_A/USB/usb_communication.h"
#include "./Hitch.h"

// from usb_device.c
extern volatile BDT_ENTRY *pBDTEntryOut[USB_MAX_EP_NUMBER+1];
#define USB_NEXT_PING_PONG 0x0004

/*---------------------------Helper Function Prototypes----------------------*/
static void InitializeSystem(void);
static void ProcessIO(void);

/*---------------------------Global Variables--------------------------------*/
int gpio_id = 0;
extern USB_DEVICE_DESCRIPTOR device_dsc;

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_MAIN
#include "./ConfigurationBits.h"
int main(void) {
  InitializeSystem();
  
  _SWDTEN = 1; // enable the watchdog timer
  while(1) {
    ProcessIO();
    ClrWdt();  // clear the watchdog timer
  }
}
#endif
/*---------------------------End Test Harness--------------------------------*/
/*---------------------------Private Function Definitions--------------------*/
static void InitializeSystem(void) {
	gpio_id = PORTE & 0x001F; ///< 5-bit board ID (RE0 to RE4)
  Initialize_USB_Message();

  // perform any PCBA-specific initializations
	device_dsc.idProduct = gpio_id;
	switch (gpio_id) {
    case DEVICE_HITCH:
      InitHitch();
      break;
		default:
			break;
	}

	// perform any generic initializations
  USBDeviceInit();
	USBDeviceAttach();
}

/*
Function: ProcessIO()
Description: Processes whatever I/O is required for the given board
*/
static void ProcessIO(void) {
  // perform any PCBA-specific routines
	switch (gpio_id) {
		case DEVICE_HITCH:
			ProcessHitchIO();
      break;
		default:
      break;
	}

	// perform any generic routines
	Handle_USB_Message();
}
