/**
 * @file HardwareProfile.h
 * @author Joel Brinton
 * @author Robotex, Inc.
 *
 * Microchip firmware settings
 *
 */


#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

//This section is the set of definitions required by the MCHPFSUSB
//  framework.  These definitions tell the firmware what mode it is
//  running in, and where it can find the results to some information
//  that the stack needs.

//These definitions are required by every application developed with
//  this revision of the MCHPFSUSB framework.  Please review each
//  option carefully and determine which options are desired/required
//  for your application.

//#define USE_SELF_POWER_SENSE_IO
#define tris_self_power     TRISAbits.TRISA2    // Input
#define self_power          1

//#define USE_USB_BUS_SENSE_IO
#define tris_usb_bus_sense  U1OTGSTATbits.SESVD  //TRISBbits.TRISB5    // Input
#define USB_BUS_SENSE       U1OTGSTATbits.SESVD


#define CLOCK_FREQ 32000000
#define GetInstructionClock() 16000000

/** I/O pin definitions ********************************************/
#define INPUT_PIN 1
#define OUTPUT_PIN 0

#endif  //HARDWARE_PROFILE_H
