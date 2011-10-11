/**
 * @file device_ocu.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex OCU PIC firmware.
 *
 */

#include "stdhdr.h"
#include "device_ocu.h"

#pragma code

void DeviceOcuInit()
{
	_ADON = 0;
	AD1PCFGL = 0xffff;
	AD1PCFGH = 0xffff;

	// turn everything off
	OCU_V3V3_ON(0);
	OCU_V5_ON(0);
	OCU_V12_ON(0);
	OCU_V3V3_CHRG_ON(0);
	OCU_JSTICK_ON(0);
	OCU_FAN_SET_ON(0);
	OCU_CHRGR_EN_ON(0);

	// enable all outputs
	OCU_V3V3_EN(1);
	OCU_V5_EN(1);
	OCU_V12_EN(1);
	OCU_V3V3_CHRG_EN(1);
	OCU_JSTICK_EN(1);
	OCU_FAN_SET_EN(1);
	OCU_CHRGR_EN_EN(1);

	// start turning things on

	__delay_ms(10);
	OCU_V3V3_ON(1);
	OCU_V5_ON(1);
	OCU_V12_ON(1);
	OCU_JSTICK_ON(1);

	__delay_ms(30);

	
}

void DeviceOcuProcessIO()
{
}
