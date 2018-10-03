/**
 * @file device_carrier.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex carrier PIC firmware.
 *
 */

// FUNCTION PROTOTYPES

void DeviceCarrierInit();

void DeviceCarrierProcessIO();

void handle_watchdogs(void);

extern unsigned int usb_timeout_counter;
extern unsigned char first_usb_message_received;
