/*******************************************************************************
File: SBS.h

Description: This module provides macros to facilitate access to configuration
  bits and registers within a battery compliant with Smart Battery System v1.1

Notes:
    - see also: http://sbs-forum.org/specs/sbdat110.pdf
    - BUG ALERT: if polling values, poll at a period: 5s < period < 60s 
*******************************************************************************/
#ifndef SBS_H
#define SBS_H
/*---------------------------Macros-------------------------------------------*/

// NOTE: THESE DEFINITIONS ARE INCOMPLETE.  ONLY WHAT HAS BEEN USED HAS BEEN
// ADDED.  IMPROVE AS NEEDED

#define SBS_SLAVE_ADDRESS       0x0B  // 7-bit address


// sub-addresses from which to request data (read-only)
#define SBS_VOLTAGE             0x09

// Returns the predicted remaining battery capacity expressed as a percentage 
// of FullChargeCapacity() (%)
#define SBS_RELATIVE_SOC        0x0D

// Returns the predicted remaining battery capacity expressed as a percentage 
// of DesignCapacity() (%). Note that AbsoluteStateOfCharge() can return values 
// greater than 100%.     
#define SBS_ABSOLUTE_SOC        0x0E


// This function selects the various battery operational modes and reports the 
// battery’s capabilities, modes, and flags minor conditions requiring 
// attention. (see p.19)
#define SBS_BATTERY_MODE        0x03

// Returns the Smart Battery's status word which contains Alarm and Status bit 
// flags. Some of the BatteryStatus() flags (REMAINING_CAPACITY_ALARM and 
// REMAINING_TIME_ALARM) are calculated based on either current or power 
// depending on the setting of the BatteryMode()'s CAPACITY_MODE bit. This is 
// important because use of the wrong calculation mode may result in an 
// inaccurate alarm.
#define SBS_BATTERY_STATUS      0x16  // (see p.29)

// Returns the version number of the Smart Battery specification the battery 
// pack supports, as well as voltage and current and capacity scaling 
// information in a packed unsigned integer.
#define SBS_SPECIFICATION_INFO  0x1A


#define SBS_MANUFACTURER_NAME   0x20

// Returns a character string that contains the battery's name. For example,
// a DeviceName() of "MBC101" would indicate that the battery is a model
// MBC101.
#define SBS_DEVICE_NAME         0x21

#endif
