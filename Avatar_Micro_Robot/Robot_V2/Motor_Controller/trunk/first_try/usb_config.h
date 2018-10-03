/**
 * @file usb_config.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * USB packet definitions for firmware and software
 *
 */

#ifndef USB_CONFIG_H
#define USB_CONFIG_H

#include "microchip/stdint.h"

#ifdef __cplusplus
extern "C" {
   namespace rbx {
      namespace telemetry {
#endif

#ifdef __C30
	#define VAR_ATTRIBS far
#else
	#define VAR_ATTRIBS
#endif

// -------- MICROCONTROLLER PRODUCT IDS --------

typedef int16_t DEVICE_NUMBER;
#define DEVICE_NONE        ~0
#define DEVICE_GENERIC      0x00
#define DEVICE_OCU          0x01
#define DEVICE_CARRIER      0x02
#define DEVICE_MOTOR        0x03
#define DEVICE_ARM          0x04
#define DEVICE_PTZ_BASE		  0x05
#define FRONT_CAMERA_EMPIA	0x06
#define PTZ_EMPIA           0x07
#define ARM_EMPIA			      0x08
#define ARM_PAYLOAD_EMPIA   0x09
#define DEVICE_PTZ_ROTATION 0x0a
#define DEVICE_ARM_BASE		  0x0b
#define DEVICE_ARM_LINK1	  0x0c
#define DEVICE_ARM_LINK2	  0x0d
#define DEVICE_DETECTOR     0x0e
#define DEVICE_HITCH        0x0f


// -------- PROGRAM DEFINES --------

#define DEVICE_READ  0x8000
#define DEVICE_WRITE 0x0000

#define SYNC    1
#define NO_SYNC 0

#define OUT_PACKET_LENGTH 255
#define IN_PACKET_LENGTH  255

#define PACKET_TERMINATOR 0xFFFF

// -------- TELEMETRY VARIABLE DEFINITIONS --------

#define REGISTER_START()
#define REGISTER( a, b, c, d, e)       extern e a __attribute__((VAR_ATTRIBS));
#define REGISTER_END()
#define MESSAGE_START( a )
#define MEMBER( a )
#define MESSAGE_END()
#include "registers.h"
#undef  REGISTER_START
#undef  REGISTER
#undef  REGISTER_END
#undef  MESSAGE_START
#undef  MEMBER
#undef  MESSAGE_END


// -------- TELEMETRY SERIALIZATION DATA DEFINITIONS --------

typedef uint16_t DEVICE;
typedef uint16_t SIZE;
typedef uint16_t SYNC_BIT;
typedef DEVICE   RW;
typedef uint16_t INDEX;
#ifndef __C30__
	typedef uint8_t  BYTE;
#endif
typedef void*    DATA_PTR;

struct REGISTER
{
   SIZE     size;
   SYNC_BIT sync;
   DEVICE   device;
   RW       rw;
   DATA_PTR ptr;
};

extern struct REGISTER registers[];


// -------- THINGS WE NEED FOR MICROCHIP FIRMWARE INTERFACE --------
#define USB_SUPPORT_DEVICE
#define USB_ENABLE_ALL_HANDLERS
//#define USB_POLL
#define USB_INTERRUPT
#define USB_EP0_BUFF_SIZE                    8
#define USB_MAX_NUM_INT                      1
#define USB_MAX_EP_NUMBER                    1
#define USB_NUM_STRING_DESCRIPTORS           3
#define USBGEN_EP_SIZE                       64
#define USBGEN_EP_NUM                        1
#define USB_USER_CONFIG_DESCRIPTOR           USB_CD_Ptr
#define USB_USER_CONFIG_DESCRIPTOR_INCLUDE   extern ROM BYTE *ROM USB_CD_Ptr[]
#define USB_PING_PONG_MODE                   USB_PING_PONG__FULL_PING_PONG
#define USB_PULLUP_OPTION                    USB_PULLUP_ENABLE
#define USB_TRANSCEIVER_OPTION               USB_INTERNAL_TRANSCEIVER
#define USB_SPEED_OPTION                     USB_FULL_SPEED
#define USB_USER_DEVICE_DESCRIPTOR           &device_dsc
#define USB_USER_DEVICE_DESCRIPTOR_INCLUDE \
           extern USB_DEVICE_DESCRIPTOR device_dsc


#ifdef __cplusplus
      } // namespace telemetry
   } // namespace rbx
} // extern "C"
#endif

#endif // USB_CONFIG_H

