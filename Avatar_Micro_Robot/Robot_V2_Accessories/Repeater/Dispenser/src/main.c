/**
 * @file main.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex one-size-fits-all firmware. Designed for the PIC24FJ256GB106 only.
 * Device running at 32MHz:
 *     Input Oscillator 20MHz
 *     PLLDIV = 5 (4MHz)
 *     PLL generates 48MHz for USB and 32MHZ for system clock
 *     CPDIV = 0 (CPU clock 32MHz)
 *     instruction clock 16MHz
 *
 */

#include "stdhdr.h"
#include "device_repeater.h"
#include "usb_communication.h"

// -------------------------------------------------------------------------
// PIC24FJ256GB106 FLASH CONFIGURATION
// -------------------------------------------------------------------------

_CONFIG1( JTAGEN_OFF & GCP_ON & GWRP_OFF & COE_OFF & FWDTEN_ON & ICS_PGx2 & WDTPS_PS256) 
_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS &

//WDTPS_PS1024 = ~5s
//WDTPS_PS2048 = ~10s

          FNOSC_PRIPLL & PLLDIV_DIV5 & IOL1WAY_ON)



// -------------------------------------------------------------------------
// GLOBAL VARIABLES
// -------------------------------------------------------------------------

#pragma udata

//int gNewData;
int gpio_id = 0;

// from usb_device.c
extern volatile BDT_ENTRY *pBDTEntryOut[USB_MAX_EP_NUMBER+1];
#define USB_NEXT_PING_PONG 0x0004

// -------------------------------------------------------------------------
// PROTOTYPES
// -------------------------------------------------------------------------

static void InitializeSystem(void);
void ProcessIO(void);
extern USB_DEVICE_DESCRIPTOR device_dsc;

// -------------------------------------------------------------------------
// CODE
// -------------------------------------------------------------------------

#pragma code

int main(void)
{
    InitializeSystem();

    while(1) { ProcessIO(); }

}

static void InitializeSystem(void)
{
	gpio_id = PORTE & 0x001F; ///< 5-bit board ID (RE0 to RE4)

  Initialize_USB_Message();

	// ---------------------------------------------------------------------
	// DEVICE SPECIFIC INITIALIZATION HERE
	// ---------------------------------------------------------------------

	device_dsc.idProduct = gpio_id;

	switch (gpio_id)
	{

		case DEVICE_REPEATER:
			Repeater_Init();
			break;

		case DEVICE_GENERIC:
		default:
			break;
	}

	// ---------------------------------------------------------------------
	// GENERIC INITIALIZATION HERE
	// ---------------------------------------------------------------------

    USBDeviceInit();
	USBDeviceAttach();
}

void ProcessIO(void)
{


	// ---------------------------------------------------------------------
	// DEVICE SPECIFIC I/O PROCESS HERE
	// ---------------------------------------------------------------------

	switch (gpio_id)
	{


		case DEVICE_REPEATER:
      ClrWdt();
			Repeater_Process_IO();
		break;

		case DEVICE_GENERIC:
		default:
			break;
	}


	// ---------------------------------------------------------------------
	// GENERIC I/O PROCESS HERE
	// ---------------------------------------------------------------------


  Handle_USB_Message();
}  
