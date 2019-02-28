/// @file
/// Hardware details, generally derived from the electrical diagram of the rover

#ifndef HARDWARE_DEFINITIONS_H
#define HARDWARE_DEFINITIONS_H

#include <xc.h>

//*-----------------------------General Purpose------------------------------*/
#define CLEAR 0
#define SET 1
#define HI 1
#define LO 0
#define Set_ActiveLO 0
#define Clear_ActiveLO 1

// constant for pins

// output pin mapping
#define M1_PWM _RP24R
#define M2_PWM _RP2R
#define M3_PWM _RP25R

// UART pins
#define U1RX_RPn 6
#define U1TX_RPn _RP7R

// A pin is configured as an analog input when
// the corresponding PCFGx bit (AD1PCFG<x>) is cleared
#define M1_TEMP_EN(a) _PCFG2 = !(a)
#define M1_CURR_EN(a) _PCFG3 = !(a)
#define M2_TEMP_EN(a) _PCFG0 = !(a)
#define M2_CURR_EN(a) _PCFG1 = !(a)
#define M3_TEMP_EN(a) _PCFG14 = !(a)
#define M3_CURR_EN(a) _PCFG15 = !(a)

#define VCELL_A_EN(a) _PCFG10 = !(a)
#define VCELL_B_EN(a) _PCFG11 = !(a)
#define CELL_A_CURR_EN(a) _PCFG12 = !(a)
#define CELL_B_CURR_EN(a) _PCFG13 = !(a)

#define M3_POS_FB_1_EN(a) _PCFG8 = !(a)
#define M3_POS_FB_2_EN(a) _PCFG9 = !(a)

// Configure outputs
// Data Direction Register TRISx controls whether a pin is input (1) or output (0)
#define M1_DIR_EN(a) _TRISD6 = !(a)
#define M1_BRAKE_EN(a) _TRISD7 = !(a)
#define M1_MODE_EN(a) _TRISD9 = !(a)
#define M1_COAST_EN(a) _TRISD10 = !(a)

#define M2_DIR_EN(a) _TRISB4 = !(a)
#define M2_BRAKE_EN(a) _TRISB5 = !(a)
#define M2_MODE_EN(a) _TRISG9 = !(a)
#define M2_COAST_EN(a) _TRISG8 = !(a)

#define M3_DIR_EN(a) _TRISE3 = !(a)
#define M3_BRAKE_EN(a) _TRISF0 = !(a)
#define M3_MODE_EN(a) _TRISE4 = !(a)
#define M3_COAST_EN(a) _TRISF1 = !(a)

#define M1_DIRO _RD0
#define M1_DIR _LATD6
#define M1_BRAKE _LATD7
#define M1_MODE _LATD9
#define M1_COAST _LATD10
#define M1_TACHO_RPN 12 // Motor 1 TACHO is pin RP12
#define M1_FF1 _RC14
#define M1_FF2 _RC13

#define M2_DIRO _RE5
#define M2_DIR _LATB4
#define M2_BRAKE _LATB5
#define M2_MODE _LATG9
#define M2_COAST _LATG8
#define M2_TACHO_RPN 16 // Motor 2 TACHO is pin RP16
#define M2_FF1 _RG6
#define M2_FF2 _RG7

#define M3_DIRO _RE0
#define M3_DIR _LATE3
#define M3_BRAKE _LATF0
#define M3_MODE _LATE4
#define M3_COAST _LATF1
#define M3_TACHO_RPN 20 // Motor 3 TACHO is pin RP20
#define M3_FF1 _RE2
#define M3_FF2 _RE1

/// Physical addresses of I2C devices
enum I2CDeviceAddress {
    /// I2C Bus 2 address of fan controller Maxim MAX6615AEE+. Datasheet with messaging info:
    /// https://www.mouser.com/datasheet/2/256/MAX6615-MAX6616-370370.pdf
    FAN_CONTROLLER_ADDRESS = 0b0011000,
    /// I2C Bus 2+3 batteries (one on I2C bus 2, one on bus 3). These are logically separate devices
    /// but they live in the same plastic housing.
    /// Batteries may be one of several models. Communicate using the SmartBattery Data
    /// specification http://sbs-forum.org/specs/sbdat110.pdf
    BATTERY_ADDRESS = 0b0001011,
    /// I2C Bus 2 EEPROM. Not in use.
    EEPROM_ADDRESS = 0b1010000,
    /// I2C Bus 2 Temperature Sensor. Not in use.
    TEMPERATURE_SENSOR_ADDRESS = 0b1001001,
    /// I2C Bus 3 Battery Charger. See Internal_Charger firmware for slave device logic. Should
    /// respond to I2C ReadWord 0xca with value 0xdada to indicate that charging is active.
    BATTERY_CHARGER_ADDRESS = 0b0001100,
};

#endif
