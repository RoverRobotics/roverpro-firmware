/*==============================================================================
File: RXUSBDevice.c
===============================================================================*/
//#define TEST_RXUSBDEVICE
//---------------------------Dependencies---------------------------------------
#include "./RXUSBDevice.h"
#include "./core/StandardHeader.h"
#include "./USB/HardwareProfile.h"      // used by USB somehow?
#include "./microchip/USB/usb.h"        // interface to microchip's USB stack
                                        // BUG ALERT: usb_config.h MUST be 
                                        // defined before this?
#include "./microchip/USB/usb_device.h" // for USBDeviceAttach(), etc

//---------------------------Macros---------------------------------------------
#define USB_NEXT_PING_PONG  0x0004      // ?

//---------------------------Helper Function Prototypes-------------------------

//---------------------------Module Variables-----------------------------------
extern volatile BDT_ENTRY *pBDTEntryOut[USB_MAX_EP_NUMBER + 1];
extern USB_DEVICE_DESCRIPTOR device_dsc;	// warning must be this name, defined in usb_descriptors.c

unsigned int i, j, n = 0;
unsigned int cur_word, reg_index, checksum;
unsigned int reg_size;
int gNewData;
int numRegisters = 0;
uint8_t OutPacket[OUT_PACKET_LENGTH];
uint8_t InPacket[IN_PACKET_LENGTH];
USB_HANDLE USBGenericOutHandle = 0;
USB_HANDLE USBGenericInHandle = 0;

//---------------------------Test Harness---------------------------------------
#ifdef TEST_RXUSBDEVICE
#include "./core/ConfigurationBits.h"

int main(void) {
  uint16_t productID = 0x0012;
  RXUSBDevice_Init(productID);
  
  while (1) {
    RXUSBDevice_ProcessMessage();
  }
  
  return 0;
}
#endif

//---------------------------Public Function Definitions------------------------
void RXUSBDevice_Init(uint16_t productID) {
  // initialize any pins?
  
  // determine the number of registers ?
  while (registers[numRegisters].ptr != 0) numRegisters++;
  
  device_dsc.idProduct = productID;
  USBDeviceInit();
	USBDeviceAttach();
}

void RXUSBDevice_ProcessMessage(void) {
  //	static unsigned char usb_rx_failed = 0;
	if ((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;
  
	/*
	TODO: understand why this was creating lag in USB?
	if (!USBHandleBusy(USBGenericInHandle) && (usb_rx_failed == 1) ) {
	  USBGenericOutHandle = USBRxOnePacket((BYTE)USBGEN_EP_NUM, (BYTE*)&OutPacket,(WORD)(OUT_PACKET_LENGTH));
		usb_rx_failed = 0;
		return;
	}
	*/
  
  if (!USBHandleBusy(USBGenericOutHandle)) {
    // CHECK FOR VALID PACKET
    i = 0;                // reset IN packet pointer
  	n = 0;                // reset OUT packet pointer
  	checksum = 0;         // reset OUT packet checksum
  	while (1) {
  		if ((n + 2) > OUT_PACKET_LENGTH ) goto crapout5; // overflow
  		
  		checksum += OutPacket[n];
  		checksum += OutPacket[n+1];
  		cur_word = OutPacket[n] + (OutPacket[n+1] << 8); // get register value
  		n += 2; // move OUT packet pointer
  
  		if (cur_word == PACKET_TERMINATOR) {
  			if ((n + 2) > OUT_PACKET_LENGTH) goto crapout5;
  			//if( checksum != (OutPacket[n] + (OutPacket[n+1] << 8) ) ) goto crapout5;
  			break;
  		}
  
      if (n > OUT_PACKET_LENGTH) goto crapout5;        // end of list
  
  		reg_index = cur_word & ~DEVICE_READ;
  
      if (reg_index >= numRegisters ) goto crapout5;   // bad packet
  
  		reg_size = registers[reg_index].size;
  
      if ((cur_word & DEVICE_READ) != DEVICE_READ) {
  		  if ((n + reg_size) > OUT_PACKET_LENGTH) goto crapout5; // overflow
  			for(j = 0; j < reg_size; j++) checksum += OutPacket[n + j];
  			n += reg_size; // move OUT packet pointer
  		}
  	}
  
  	// PARSE INCOMING PACKET
  	i = 0;                // reset IN packet pointer
  	n = 0;                // reset OUT packet pointer
    while (1) {
  		if ((n + 2) > OUT_PACKET_LENGTH) break; // overflow
  		cur_word = OutPacket[n] + (OutPacket[n+1] << 8); // get register value
  		n += 2; // move OUT packet pointer
  
  		if ((cur_word == PACKET_TERMINATOR) || n > OUT_PACKET_LENGTH) break;        // end of list
  
  		reg_index = cur_word & ~DEVICE_READ;
      if( reg_index >= numRegisters ) break;   // bad packet
  		
  		reg_size = registers[reg_index].size;
      if ((cur_word & DEVICE_READ) == DEVICE_READ) {
  			if ((i + 2) > IN_PACKET_LENGTH) break;         // overflow
  			InPacket[i + 1] = reg_index >> 8;
  			InPacket[i] = reg_index & 0xff;
  			i = i + 2;        // move IN packet pointer
  			if ((i + reg_size) > IN_PACKET_LENGTH) break;  // overflow
  			memcpy(InPacket + i, registers[reg_index].ptr, reg_size); 
  			i = i + reg_size; // move IN packet pointer
  		} else {
  		  if( (n + reg_size) > OUT_PACKET_LENGTH ) break; // overflow
  			memcpy(registers[reg_index].ptr, OutPacket + n, reg_size);
  			n += reg_size; // move OUT packet pointer
  		}
  	}
  	
  	if ((i + 2) > IN_PACKET_LENGTH) goto crapout5;
  
  	InPacket[i + 1] = PACKET_TERMINATOR >> 8;
  	InPacket[i] = PACKET_TERMINATOR & 0xff;
  	i += 2;
  
  	if (!USBHandleBusy(USBGenericInHandle) && (i > 0)) {
  		USBGenericInHandle = USBTxOnePacket((BYTE)USBGEN_EP_NUM,(BYTE*)&InPacket,(WORD)i);
  	}
  	gNewData = !gNewData; // toggle new data flag for those watching
  
crapout5:
		USBGenericOutHandle = USBRxOnePacket((BYTE)USBGEN_EP_NUM, (BYTE*)&OutPacket, (WORD)(OUT_PACKET_LENGTH));
  	/*
  	// TODO: UNDERSTAND WHY THIS CREATED LAG?
    if (!USBHandleBusy(USBGenericInHandle)) {
  		// prepare USB hardware to receive next packet.
      USBGenericOutHandle = USBRxOnePacket((BYTE)USBGEN_EP_NUM, (BYTE*)&OutPacket,(WORD)(OUT_PACKET_LENGTH));
  		usb_rx_failed = 0;
  	} else {
  		usb_rx_failed = 1;
  	}
  	*/
  }
}

/****************************NOT USED RIGHT NOW********************************/
void USBCBInitEP(void) {
  USBEnableEndpoint(USBGEN_EP_NUM,USB_OUT_ENABLED|USB_IN_ENABLED|USB_DISALLOW_SETUP);
  USBGenericOutHandle = USBRxOnePacket((BYTE)USBGEN_EP_NUM,(BYTE*)&OutPacket,(WORD)(OUT_PACKET_LENGTH));
}

bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size) {
  switch (event) {
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
  
  return 1;
}
