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
#include "device_robot_motor.h"
#include "device_ocu.h"
#include "device_carrier.h"
#include "device_battery.h"
#include "device_arm_base.h"
#include "device_arm_sholder.h"
#include "device_arm_hand.h"

#include "SA1xLibrary/SA_API.h"

// -------------------------------------------------------------------------
// PIC24FJ256GB106 FLASH CONFIGURATION
// -------------------------------------------------------------------------

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2) 
_CONFIG2( IESO_OFF & FCKSM_CSDCMD & OSCIOFNC_ON & POSCMOD_HS &
          FNOSC_PRIPLL & PLLDIV_DIV5 & IOL1WAY_ON)

// -------------------------------------------------------------------------
// GLOBAL VARIABLES
// -------------------------------------------------------------------------

#pragma udata

int gNewData;
int gpio_id = 0;
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

int main(void)
{   
    InitializeSystem();

    while(1) { ProcessIO(); }

}


static void InitializeSystem(void)
{
	gpio_id = PORTE & 0x001F; ///< 5-bit board ID (RE0 to RE4)

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

		case DEVICE_BATTERY:
			DeviceBatteryInit();
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
	uint16_t reg;
	uint16_t reg_size;
	uint16_t i = 0;

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
 			//PORTFbits.RF5=0;
			Device_MotorController_Process();
 			//PORTFbits.RF5=1;
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

		case DEVICE_BATTERY:
			DeviceBatteryProcessIO();
			break;

		case DEVICE_GENERIC:
		default:
			DeviceGenericProcessIO();
			break;
	}

	// ---------------------------------------------------------------------
	// GENERIC I/O PROCESS HERE
	// ---------------------------------------------------------------------

    // USBDeviceTasks();
	if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;
    
    if(!USBHandleBusy(USBGenericOutHandle)) ///< if packet has been received
    {
		//gREG_MOTOR1_VELOCITY = gREG_MOTOR1_VELOCITY?0:1;
		gNewData = !gNewData;
		i = 0;
		while(1)
		{
			// get register value
			reg = *(uint16_t*)(OutPacket + n);
			n += 2;

			// crap out if we're at the end of the list
			if ((reg == PACKET_TERMINATOR) || n > OUT_PACKET_LENGTH) break;

			reg_size = registers[reg & ~DEVICE_READ].size;
            if (reg & DEVICE_READ) {
				*(uint16_t*)(InPacket + i) = reg & ~DEVICE_READ;
				i += 2;
				memcpy(InPacket + i, registers[reg & ~DEVICE_READ].ptr, reg_size);
				i += reg_size;
			}
			else {
				memcpy(registers[reg].ptr, OutPacket + n, reg_size);
				n += reg_size;
			}
		}
		*(uint16_t*)(InPacket + i) = PACKET_TERMINATOR;
		i += 2;
		if(!USBHandleBusy(USBGenericInHandle) && (i > 0))		
		{
			USBGenericInHandle = USBTxOnePacket(USBGEN_EP_NUM,(BYTE*)&InPacket,(unsigned)i);
		}
		/// Arm USB hardware to receive next packet.
        USBGenericOutHandle = USBRxOnePacket(USBGEN_EP_NUM,(BYTE*)&OutPacket,(unsigned)OUT_PACKET_LENGTH);
    }
}


void USBCBInitEP(void)
{
    USBEnableEndpoint(USBGEN_EP_NUM,USB_OUT_ENABLED|USB_IN_ENABLED|USB_DISALLOW_SETUP);
    USBGenericOutHandle = USBRxOnePacket(USBGEN_EP_NUM,(BYTE*)&OutPacket,(unsigned)OUT_PACKET_LENGTH);
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
            Nop();
            break;
        default:
            break;
    }      
    return TRUE; 
}
