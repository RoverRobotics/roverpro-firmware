#include "usb_communication.h"
#include "../microchip/USB/usb.h"

unsigned int i, j, n = 0;
unsigned int  cur_word, reg_index, checksum;
unsigned int  reg_size;


unsigned char USB_timeout_counter = 0;
int gNewData;
int gRegisterCount = 0;
uint8_t OutPacket[OUT_PACKET_LENGTH];
uint8_t InPacket[IN_PACKET_LENGTH];
USB_HANDLE USBGenericOutHandle = 0;
USB_HANDLE USBGenericInHandle = 0;

void Initialize_USB_Message(void)
{
	// get number of registers
    while( registers[gRegisterCount].ptr != 0 )
	{
		gRegisterCount++;
	}

}

void Handle_USB_Message(void)
{

	static unsigned char usb_rx_failed = 0;
	if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

	//if we failed to arm the USB module for reception the last time, do so now, and return from the function (since we won't have a new packet waiting

	if(!USBHandleBusy(USBGenericInHandle) && (usb_rx_failed == 1) )
	{
	        USBGenericOutHandle = USBRxOnePacket((BYTE)USBGEN_EP_NUM,
	                                  (BYTE*)&OutPacket,(WORD)(OUT_PACKET_LENGTH));
			usb_rx_failed = 0;
			return;
	}

    
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

		if(!USBHandleBusy(USBGenericInHandle))
		{
			// Arm USB hardware to receive next packet.
	        USBGenericOutHandle = USBRxOnePacket((BYTE)USBGEN_EP_NUM,
	                                  (BYTE*)&OutPacket,(WORD)(OUT_PACKET_LENGTH));
			usb_rx_failed = 0;
		}
		else
		{
			usb_rx_failed = 1;
		}
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
	        //if there is an error, trigger a bus reset
	        USBClearInterruptRegister(U1EIR);               // This clears UERRIF
			USBResetIF = 1;
			USBResetIE = 1;
			Nop();
            break;
        case EVENT_TRANSFER:
            break;
        default:
            break;
    }      
    return TRUE; 
}
