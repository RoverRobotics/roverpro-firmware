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
//#include "device_generic.h"
//#include "device_robot_motor.h"
//#include "device_ocu.h"
#include "device_detector.h"
//#include "device_arm_base.h"
//#include "device_arm_sholder.h"
//#include "device_arm_hand.h"

/*#include "../../../../Common/V2.X/USB/usb_config.c"
#include "../../../../Common/V2.X/USB/usb_descriptors.c"
#include "../../../../Common/V2.X/USB/usb_device.c"
#include "../../../../Common/V2.X/USB/usb_communication.c"*/

#include USB_CONFIG_INCLUDE 
#include USB_DESCRIPTORS_INDCLUE
#include USB_DEVICE_INCLUDE
#include USB_COMMUNICATION_INCLUDE

//#include "SA1xLibrary/SA_API.h"


// -------------------------------------------------------------------------
// PIC24FJ256GB106 FLASH CONFIGURATION
// -------------------------------------------------------------------------

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2 & WDTPS_PS256) 
_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS &

//WDTPS_PS1024 = ~5s
//WDTPS_PS2048 = ~10s

          FNOSC_PRIPLL & PLLDIV_DIV5 & IOL1WAY_ON)
/*_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2) 
_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS &
          FNOSC_FRCPLL & PLLDIV_DIV2 & IOL1WAY_ON)*/


// -------------------------------------------------------------------------
// GLOBAL VARIABLES
// -------------------------------------------------------------------------

#pragma udata




// -------------------------------------------------------------------------
// BOOTLOADER
// 
// All bootloader code must be self contained with interrupts disabled
// and no function calls. When the bootloader function is called, it will
// autonomously receive the next set of USB packets and write them to flash.
// In a way it's not a complete bootloader because it doesn't start until
// called, so it relies on firmware to be in place to reflash the firmware

// 0x000 to 0x1FF  (system interrupt vectors)
// 0x200 to 0x3FF  (bootloader function)
// 0x400 to ---    (firmware - flash from here forward)
//
// _FLASH_PAGE = 0x200 (on pic24F)
// _FLASH_ROW  = 0x040 (on pic24F)
//
// receive one _FLASH_PAGE of bytes at a time 

// -------------------------------------------------------------------------

#pragma code

#define BL_PRI   __attribute__(( section("bootloader"),space(prog),address(0x200) ))
#define BL       __attribute__(( section("bootloader"),space(prog) ))
#define FIRMWARE __attribute__(( section("firmware"),space(prog),address(0x400) ))

// from usb_device.c
extern volatile BDT_ENTRY *pBDTEntryOut[USB_MAX_EP_NUMBER+1];
#define USB_NEXT_PING_PONG 0x0004

// adjust this to make sure bootloader is exactly 0x200 in length
/*const char __bootloader_buffer[20] BL = { 0 };
const char __firmware_buffer[] FIRMWARE = "Robotex Firmware begins here\n\r";

// rearm USB endpoint after processing of packet, must be 512byte packets
USB_HANDLE BL arm_bdt_for_bootloader()
{
	volatile BDT_ENTRY* handle;

	handle = pBDTEntryOut[USBGEN_EP_NUM];
	handle->ADR = ConvertToPhysicalAddress((BYTE*)&OutPacket);
	handle->CNT = (unsigned)192; // one flash row (24-bits wide X 64 long)
	handle->STAT.Val &= _DTSMASK;
	handle->STAT.Val |= _USIE | _DTSEN;
 	((BYTE_VAL*)&pBDTEntryOut[USBGEN_EP_NUM])->Val ^= USB_NEXT_PING_PONG;
	return (USB_HANDLE)handle;
}

void BL flash_firmware(unsigned row)
{
	_prog_addressT p;
	long base;

	_init_prog_address(p, __firmware_buffer);
	base = p + (row * _FLASH_ROW);

	// on page boundaries erase flash
	if( (base % _FLASH_PAGE) == 0) _erase_flash(base);

	unsigned n;

	// casting only works little-endian
	// 0  1  2  3  4  5  6  7  8
	// [     ]   <-24-bits cast to 32-bits
	//       [     ]
    //             [     ]
    //                   [     ]
	//
	for (n = 0; n < _FLASH_ROW; n++)
		_write_flash_word24(base + n, *((long*)(*OutPacket+(3*n))));
}

void BL_PRI bootloader(void)
{
	// disable all interrupts
	_IPL = 0x03;

	unsigned row = 0;

	USBGenericOutHandle = arm_bdt_for_bootloader();

	while(1)
	{
		if(USBTransactionCompleteIF)
		{
			// macro (no external function call)
		    USBClearInterruptFlag(USBTransactionCompleteIFReg,USBTransactionCompleteIFBitNum);

			// macro (no external function call)
	    	if(!USBHandleBusy(USBGenericOutHandle))
			{
				// parse USB message
				flash_firmware(row++);

				// reset once written all byte
				if ((row * _FLASH_ROW) >= CMD_UPDATE_FIRMWARE.length) Reset();

				// rearm BDT
				USBGenericOutHandle = arm_bdt_for_bootloader();
			}
		}
		// macro (no external function call)
		USBClearUSBInterrupt();
	}
}*/



// -------------------------------------------------------------------------
// PROTOTYPES
// -------------------------------------------------------------------------

static void InitializeSystem(void);
//void USBDeviceTasks(void);
//void USBSendPacket(uint8_t command);
void ProcessIO(void);


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

		case DEVICE_CARRIER:
			//DeviceCarrierInit();
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
		case DEVICE_OCU:
			break;

		case DEVICE_CARRIER:
			//DeviceCarrierProcessIO();
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
