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
#include "device_initiator.h"


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

	// get number of registers
    while( registers[gRegisterCount].ptr != 0 )
	{
		gRegisterCount++;
	}


	// ---------------------------------------------------------------------
	// DEVICE SPECIFIC INITIALIZATION HERE
	// ---------------------------------------------------------------------

	device_dsc.idProduct = gpio_id;

	switch (gpio_id)
	{

		case DEVICE_INITIATOR:
			Initiator_Init();
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


		case DEVICE_INITIATOR:
      ClrWdt();
			Initiator_Process_IO();
		break;

		case DEVICE_GENERIC:
		default:
			break;
	}


	// ---------------------------------------------------------------------
	// GENERIC I/O PROCESS HERE
	// ---------------------------------------------------------------------










/*OutPacket[0] = 0x11;
OutPacket[1] = 0x80;
OutPacket[2] = 0x12;
OutPacket[3] = 0x80;
OutPacket[4] = 0x13;
OutPacket[5] = 0x80;
OutPacket[6] = 0x14;
OutPacket[7] = 0x80;
OutPacket[8] = 0x15;
OutPacket[9] = 0x80;
OutPacket[10] = 0x16;
OutPacket[11] = 0x80;
OutPacket[12] = 0x17;
OutPacket[13] = 0x80;
OutPacket[14] = 0x18;
OutPacket[15] = 0x80;
OutPacket[16] = 0x19;
OutPacket[17] = 0x80;
OutPacket[18] = 0x1a;
OutPacket[19] = 0x80;
OutPacket[20] = 0x1b;
OutPacket[21] = 0x80;
OutPacket[22] = 0x1c;
OutPacket[23] = 0x80;
OutPacket[24] = 0x1d;
OutPacket[25] = 0x80;
OutPacket[26] = 0x1e;
OutPacket[27] = 0x80;
OutPacket[28] = 0x34;
OutPacket[29] = 0x80;
OutPacket[30] = 0x35;
OutPacket[31] = 0x80;
OutPacket[32] = 0x36;
OutPacket[33] = 0x80;
OutPacket[34] = 0x37;
OutPacket[35] = 0x80;
OutPacket[36] = 0x38;
OutPacket[37] = 0x80;
OutPacket[38] = 0x39;
OutPacket[39] = 0x80;
OutPacket[40] = 0x3a;
OutPacket[41] = 0x80;
OutPacket[42] = 0x3b;
OutPacket[43] = 0x80;
OutPacket[44] = 0x3c;
OutPacket[45] = 0x80;
OutPacket[46] = 0x3d;
OutPacket[47] = 0x80;
OutPacket[48] = 0x3e;
OutPacket[49] = 0x80;
OutPacket[50] = 0x3f;
OutPacket[51] = 0x80;
OutPacket[52] = 0x43;
OutPacket[53] = 0x80;
OutPacket[54] = 0x44;
OutPacket[55] = 0x80;
OutPacket[56] = 0x45;
OutPacket[57] = 0x80;
OutPacket[58] = 0x46;
OutPacket[59] = 0x80;
OutPacket[60] = 0x47;
OutPacket[61] = 0x80;
OutPacket[62] = 0x48;
OutPacket[63] = 0x80;
OutPacket[64] = 0x49;
OutPacket[65] = 0x80;
OutPacket[66] = 0x4a;
OutPacket[67] = 0x80;
OutPacket[68] = 0xff;
OutPacket[69] = 0xff;
OutPacket[70] = 0x2d;
OutPacket[71] = 0x19;*/






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
				//if( checksum != (OutPacket[n] + (OutPacket[n+1] << 8) ) ) goto crapout5;
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
        USB_timeout_counter = 0;
			    if( (n + reg_size) > OUT_PACKET_LENGTH ) break; // overflow
				memcpy(registers[reg_index].ptr, OutPacket + n, reg_size);
				n += reg_size; // move OUT packet pointer
			}
		}
//		if (CMD_UPDATE_FIRMWARE.magic == 0x2345BCDE) bootloader();

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
