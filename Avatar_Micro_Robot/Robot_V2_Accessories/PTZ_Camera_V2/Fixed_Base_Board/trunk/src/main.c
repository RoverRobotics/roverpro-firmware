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
#include "device_generic.h"
#include "device_ocu.h"
#include "device_carrier.h"
#include "device_arm_base.h"
#include "device_arm_sholder.h"
#include "device_arm_hand.h"
#include "device_ptz_base.h"
#include "device_ptz_rotation.h"

#include "SA1xLibrary/SA_API.h"


// -------------------------------------------------------------------------
// PIC24FJ256GB106 FLASH CONFIGURATION
// -------------------------------------------------------------------------

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2) 
_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS &
          FNOSC_PRIPLL & PLLDIV_DIV5 & IOL1WAY_ON)
/*_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2) 
_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS &
          FNOSC_FRCPLL & PLLDIV_DIV2 & IOL1WAY_ON)*/


// -------------------------------------------------------------------------
// GLOBAL VARIABLES
// -------------------------------------------------------------------------

#pragma udata

int gNewData;
int gpio_id = 0;
int gRegisterCount = 0;
uint8_t OutPacket[OUT_PACKET_LENGTH];
uint8_t InPacket[IN_PACKET_LENGTH];
USB_HANDLE USBGenericOutHandle = 0;
USB_HANDLE USBGenericInHandle = 0;


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
const char __bootloader_buffer[20] BL = { 0 };
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

void BL flash_firmware(uint32_t address)
{
	unsigned n;

	// on page boundaries erase flash
	//if( (address % _FLASH_PAGE) == 0) _erase_flash(address);

	// casting only works little-endian
	// 0  1  2  3  4  5  6  7  8
	// [     ]   <-24-bits cast to 32-bits
	//       [     ]
    //             [     ]
    //                   [     ]
	//
	for (n = 0; n < FIRMWARE_BOUNDARIES; n += 3) {
		//_write_flash_word24(address >> 1, *((long*)(OutPacket+(4*n))));
	}
}

void BL_PRI bootloader(void)
{
	// disable all interrupts
	_IPL = 0x03;

	uint32_t address = 0;

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
				// burn image except for boot loader
				if( address < 0x200 || address >= 0x400 ) flash_firmware(address);

				address += FIRMWARE_BOUNDARIES;

				// reset once written all byte
				if (address >= FIRMWARE_SIZE) Reset();

				// rearm BDT
				USBGenericOutHandle = arm_bdt_for_bootloader();
			}
		}
		// macro (no external function call)
		USBClearUSBInterrupt();
	}
}

void CalcChecksum()
{
	_prog_addressT p = 0;
	uint8_t buf[4] = { 0 };

	//REG_FIRMWARE_CHECKSUM = 0;

    for( p = 0; p < (FIRMWARE_SIZE/2); p += 2) {
		_memcpy_p2d24( buf, p, 1 );
        //REG_FIRMWARE_CHECKSUM = REG_FIRMWARE_CHECKSUM + buf[0] + buf[1] + buf[2];
	}
}


// -------------------------------------------------------------------------
// PROTOTYPES
// -------------------------------------------------------------------------

static void InitializeSystem(void);
void USBDeviceTasks(void);
void USBSendPacket(uint8_t command);
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

	/*
	// override clock settings
	CLKDIVbits.RCDIV = 0;
	CLKDIVbits.CPDIV = 0;
	CLKDIVbits.DOZEN = 0;
	CLKDIVbits.DOZE = 0;*/


	// get number of registers
    while( registers[gRegisterCount].ptr != 0 )
	{
		gRegisterCount++;
	}

	// ---------------------------------------------------------------------
    // CALCULATE HARDWARE CHECKSUM
	// ---------------------------------------------------------------------
    //CalcChecksum();

	// ---------------------------------------------------------------------
	// DEVICE SPECIFIC INITIALIZATION HERE
	// ---------------------------------------------------------------------

	device_dsc.idProduct = gpio_id;

	switch (gpio_id)
	{
		case DEVICE_OCU:
			DeviceOcuInit();
			break;

		case DEVICE_CARRIER:
			DeviceCarrierInit();
			break;

		case DEVICE_MOTOR:
			DeviceGenericInit();
			break;

		case DEVICE_ARM_BASE:
			DeviceArmBaseInit();
			break;

		case DEVICE_ARM_SHOLDER:
			DeviceArmSholderInit();
			break;

		case DEVICE_ARM_HAND:
			DeviceArmHandInit();
			break;
	
		case DEVICE_PTZ_BASE:
			DevicePTZBaseInit();
			break;

		case DEVICE_PTZ_ROTATION:
			DevicePTZRotationInit();
			break;

		case DEVICE_GENERIC:
		default:
			DeviceGenericInit();
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
	uint16_t n = 0;
	uint16_t cur_word, reg_index, checksum;
	uint16_t reg_size;
	uint16_t i = 0;
	uint16_t j;

	// ---------------------------------------------------------------------
	// DEVICE SPECIFIC I/O PROCESS HERE
	// ---------------------------------------------------------------------

	switch (gpio_id)
	{
		case DEVICE_OCU:
			DeviceOcuProcessIO();
			break;

		case DEVICE_CARRIER:
			DeviceCarrierProcessIO();
			break;

		case DEVICE_MOTOR:
			DeviceGenericProcessIO();
			break;

		case DEVICE_ARM_BASE:
			DeviceArmBaseProcessIO();
			break;

		case DEVICE_ARM_SHOLDER:
			DeviceArmSholderProcessIO();
			break;

		case DEVICE_ARM_HAND:
			DeviceArmHandProcessIO();
			break;

		case DEVICE_PTZ_BASE:
			DevicePTZBaseProcessIO();
			break;

		case DEVICE_PTZ_ROTATION:
			DevicePTZRotationProcessIO();
			break;

		case DEVICE_GENERIC:
		default:
			DeviceGenericProcessIO();
			break;
	}


	// ---------------------------------------------------------------------
	// GENERIC I/O PROCESS HERE
	// ---------------------------------------------------------------------


	if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;
    
    if(!USBHandleBusy(USBGenericOutHandle))
    {
		i = 0;                // reset IN packet pointer
		n = 0;                // reset OUT packet pointer
		checksum = 0;         // reset OUT packet checksum
        // CHECK FOR VALID PACKET ---------------------------------------------
		while(1)
		{
			if( (n + 2) > OUT_PACKET_LENGTH ) goto crapout5; // overflow

			checksum += OutPacket[n];
			checksum += OutPacket[n+1];
			cur_word = OutPacket[n] + (OutPacket[n+1] << 8); // get register value
			n += 2; // move OUT packet pointer

			if (cur_word == PACKET_TERMINATOR)
			{
				if ((n + 2) > OUT_PACKET_LENGTH) goto crapout5;
				if( checksum != (OutPacket[n] + (OutPacket[n+1] << 8) ) ) goto crapout5;
				break;
			}

            if (n > OUT_PACKET_LENGTH) goto crapout5;        // end of list

			reg_index = cur_word & ~DEVICE_READ;

            if( reg_index >= gRegisterCount ) goto crapout5;   // bad packet

			reg_size = registers[reg_index].size;

            if( (cur_word & DEVICE_READ) != DEVICE_READ )
			{
			    if( (n + reg_size) > OUT_PACKET_LENGTH ) goto crapout5; // overflow
				for( j = 0; j < reg_size; j++ )
				{
					checksum += OutPacket[n + j];
				}
				n += reg_size; // move OUT packet pointer
			}
		}


		i = 0;                // reset IN packet pointer
		n = 0;                // reset OUT packet pointer
        // PARSE INCOMING PACKET ----------------------------------------------
		while(1)
		{
			if( (n + 2) > OUT_PACKET_LENGTH ) break; // overflow

			cur_word = OutPacket[n] + (OutPacket[n+1] << 8); // get register value
			n += 2; // move OUT packet pointer

			if ((cur_word == PACKET_TERMINATOR) ||
                 n > OUT_PACKET_LENGTH) break;        // end of list

			reg_index = cur_word & ~DEVICE_READ;

            if( reg_index >= gRegisterCount ) break;   // bad packet

			reg_size = registers[reg_index].size;

            if( (cur_word & DEVICE_READ) == DEVICE_READ )
			{
				if( (i + 2) > IN_PACKET_LENGTH ) break;         // overflow
				InPacket[i + 1]     = reg_index >> 8;
				InPacket[i]         = reg_index & 0xff;
				i = i + 2;        // move IN packet pointer
				if( (i + reg_size) > IN_PACKET_LENGTH ) break;  // overflow
				memcpy(InPacket + i, registers[reg_index].ptr, reg_size); 
				i = i + reg_size; // move IN packet pointer
			}
			else
			{
			    if( (n + reg_size) > OUT_PACKET_LENGTH ) break; // overflow
				memcpy(registers[reg_index].ptr, OutPacket + n, reg_size);
				n += reg_size; // move OUT packet pointer
			}
		}
	//	if (CMD_UPDATE_FIRMWARE == 0x2345BCDE)
	//	{
			//bootloader();
	//	}

		if( (i + 2) > IN_PACKET_LENGTH ) goto crapout5;

		InPacket[i + 1] = PACKET_TERMINATOR >> 8;
		InPacket[i]     = PACKET_TERMINATOR & 0xff;
		i += 2;

		if(!USBHandleBusy(USBGenericInHandle) && (i > 0))		
		{
			USBGenericInHandle = USBTxOnePacket((BYTE)USBGEN_EP_NUM,(BYTE*)&InPacket,(WORD)i);
		}
		gNewData = !gNewData; // toggle new data flag for those watching

crapout5:
		// Arm USB hardware to receive next packet.
        USBGenericOutHandle = USBRxOnePacket((BYTE)USBGEN_EP_NUM,
                                  (BYTE*)&OutPacket,(WORD)(OUT_PACKET_LENGTH));
    }
}


void USBCBInitEP(void)
{
    USBEnableEndpoint(USBGEN_EP_NUM,USB_OUT_ENABLED|USB_IN_ENABLED|USB_DISALLOW_SETUP);
    USBGenericOutHandle = USBRxOnePacket((BYTE)USBGEN_EP_NUM,(BYTE*)&OutPacket,(WORD)(OUT_PACKET_LENGTH));
}


BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            break;
        case EVENT_EP0_REQUEST:
            break;
        case EVENT_SOF:
            break;
        case EVENT_SUSPEND:
            break;
        case EVENT_RESUME:
            break;
        case EVENT_BUS_ERROR:
            break;
        case EVENT_TRANSFER:
            break;
        default:
            break;
    }      
    return TRUE; 
}
