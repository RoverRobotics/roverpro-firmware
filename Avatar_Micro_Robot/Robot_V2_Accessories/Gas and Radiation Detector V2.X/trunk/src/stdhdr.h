/**
 * @file stdhdr.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex one-size-fits-all firmware. Designed for the PIC24FJ256GB106 only.
 *
 */

#ifndef STDHDR_H
#define STDHDR_H


#define FCY 16000000UL        // instruction clock
#define I2C_RATE_SETTING 0Xff // 200 KHz for 32 MHz clock frequency

#define STDINT_INCLUDE            "../../../../../Common/REV_A/USB/microchip/stdint.h"
#define GENERIC_TYPEDEFS_INCLUDE  "../../../../../Common/REV_A/USB/microchip/GenericTypeDefs.h"
#define USB_INCLUDE               "../../../../../Common/REV_A/USB/microchip/USB/usb.h"
#define COMPILER_INCLUDE          "../../../../../Common/REV_A/USB/microchip/Compiler.h"

#define USB_CONFIG_INCLUDE        "../../../../../Common/REV_A/USB/usb_config.c"
#define USB_DESCRIPTORS_INDCLUE   "../../../../../Common/REV_A/USB/usb_descriptors.c"
#define USB_DEVICE_INCLUDE        "../../../../../Common/REV_A/USB/usb_device.c"
#define USB_COMMUNICATION_INCLUDE "../../../../../Common/REV_A/USB/usb_communication.c"

//#include COMMON_FILE_LOCATION##"/USB/microchip/stdint.h"
#include STDINT_INCLUDE
#include <libpic30.h>

#include GENERIC_TYPEDEFS_INCLUDE
#include COMPILER_INCLUDE
#include USB_INCLUDE
#include "i2c.h"
#include "PwrMgnt.h"
#include "interrupt_switch.h"

// PROTOTYPES FOR PROJECT stdfunction.h

extern void block_ms(unsigned int ms);
extern void writeI2C( unsigned char add, unsigned char v );
extern int readI2C( unsigned char add );
extern void writeI2CReg( unsigned char add, unsigned char v, unsigned char w);
unsigned char readI2C_Reg(unsigned char add, unsigned char reg);

#include "../HardwareProfile.h"

#endif
