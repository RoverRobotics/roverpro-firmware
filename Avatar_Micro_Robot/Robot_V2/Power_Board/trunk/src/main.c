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

#include "device_robot_motor.h"


#include "SA1xLibrary/SA_API.h"


// -------------------------------------------------------------------------
// PIC24FJ256GB106 FLASH CONFIGURATION
// -------------------------------------------------------------------------

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2 & WDTPS_PS2048) 
_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS &
          FNOSC_PRIPLL & PLLDIV_DIV5 & IOL1WAY_ON)
/*_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2) 
_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS &
          FNOSC_FRCPLL & PLLDIV_DIV2 & IOL1WAY_ON)*/

//WDTPS_PS2048 = ~10s

// -------------------------------------------------------------------------
// BOOTLOADER
// -------------------------------------------------------------------------

#define PF __attribute__((section("programmable"))) // programmable function
#define FIRST_PROGRAMMABLE_FUNC __attribute__((address(0xF00)))

typedef enum COMMAND_T
{
   DEVICE_OCU_INIT,
   DEVICE_CARRIER_INIT,

   DEVICE_OCU_PROCESS_IO,
   DEVICE_CARRIER_PROCESS_IO
   // etc.....
} COMMAND;

void PF FIRST_PROGRAMMABLE_FUNC callFunc(COMMAND command, void *params)
{
   return;
}



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

int PF main(void)
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
	// DEVICE SPECIFIC INITIALIZATION HERE
	// ---------------------------------------------------------------------

	device_dsc.idProduct = DEVICE_MOTOR;


	//we got rid of the ID pins, so force robot motor to init
	DeviceRobotMotorInit();

/*	switch (gpio_id)
	{
		case DEVICE_OCU:
			DeviceOcuInit();
			break;

		case DEVICE_CARRIER:
			DeviceCarrierInit();
			break;

		case DEVICE_MOTOR:
			DeviceRobotMotorInit();
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

		case DEVICE_GENERIC:
		default:
			DeviceGenericInit();
			break;
	}*/

	// ---------------------------------------------------------------------
	// GENERIC INITIALIZATION HERE
	// ---------------------------------------------------------------------

    USBDeviceInit();
	USBDeviceAttach();
}

void ProcessIO(void)
{
	uint16_t n = 0;
	uint16_t cur_word, reg_index;
	uint16_t reg_size;
	uint16_t i = 0;

	ClrWdt();

	// ---------------------------------------------------------------------
	// DEVICE SPECIFIC I/O PROCESS HERE
	// ---------------------------------------------------------------------

	//we got rid of id pins, so force motor controller to run
	Device_MotorController_Process();

/*	switch (gpio_id)
	{
		case DEVICE_OCU:
			DeviceOcuProcessIO();
			break;

		case DEVICE_CARRIER:
			DeviceCarrierProcessIO();
			break;

		case DEVICE_MOTOR:
			Device_MotorController_Process();
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

		case DEVICE_GENERIC:
		default:
			DeviceGenericProcessIO();
			break;
	}*/


	// ---------------------------------------------------------------------
	// GENERIC I/O PROCESS HERE
	// ---------------------------------------------------------------------


	if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;
    
    if(!USBHandleBusy(USBGenericOutHandle))
    {
		gNewData = !gNewData; // toggle new data flag for those watching
		i = 0;                // reset IN packet pointer

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

		if( (i + 2) > IN_PACKET_LENGTH ) goto crapout5;

		InPacket[i + 1] = PACKET_TERMINATOR >> 8;
		InPacket[i]     = PACKET_TERMINATOR & 0xff;
		i += 2;

		if(!USBHandleBusy(USBGenericInHandle) && (i > 0))		
		{
			USBGenericInHandle = USBTxOnePacket((BYTE)USBGEN_EP_NUM,(BYTE*)&InPacket,(WORD)i);
		}

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
