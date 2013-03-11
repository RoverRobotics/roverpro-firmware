/**
 * @file device_ocu.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex OCU PIC firmware.
 *
 */

void DeviceOcuInit();

void DeviceOcuProcessIO();
void test_sleep_current(void);

#define U1TX_RPn  	_RP7R
