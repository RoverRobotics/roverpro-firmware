/**
 * @file device_generic.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex generic PIC firmware.
 *
 */

void DeviceGenericInit();

void DeviceGenericProcessIO();

extern uint16_t gPBusCurrent;
extern uint16_t gPBus;


// I'm assuming that if you're using a generic device the
// PIC is on the PIC24F Starter Kit 1 Board and the following applies...

#define mInitAllLEDs()      LATG &= 0xFE1F; TRISG &= 0xFE1F; LATF &= 0xFFCF; TRISF &= 0xFFCF; //G6,7,8,9 and F4,5

#define mGetLED_1()         (TRISG & ~0x0180?1:0)
#define mGetLED_2()         (TRISG & ~0x0060?1:0)
#define mGetLED_3()         (TRISF & ~0x0030?1:0)
#define mGetLED_4()

#define mLED_1_On()         TRISG |= 0x0180; // makes port pin an input
#define mLED_2_On()         TRISG |= 0x0060;
#define mLED_3_On()         TRISF |= 0x0030;
#define mLED_4_On()

#define mLED_1_Off()        TRISG &= ~0x0180; // makes port pin an output
#define mLED_2_Off()        TRISG &= ~0x0060;
#define mLED_3_Off()        TRISF &= ~0x0030;
#define mLED_4_Off()

#define mLED_1_Toggle()     TRISG ^= 0x0180;
#define mLED_2_Toggle()     TRISG ^= 0x0060;
#define mLED_3_Toggle()     TRISF ^= 0x0030;
#define mLED_4_Toggle()
