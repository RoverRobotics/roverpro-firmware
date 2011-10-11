/**
 * @file device_battery.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex battery PIC firmware.
 *
 */

#include "stdhdr.h"
#include "device_battery.h"

#pragma code

void DeviceBatteryInit()
{
	_ADON = 0;
	AD1PCFGL = 0xffff;
	AD1PCFGH = 0xffff;

	// turn everything off
	SMBUS_ON(0);
	CODEC_PD_ON(0);
	RADIO_ON(0);
	COM_EXPRESS_PG_ON(0);
	COM_EXPRESS_ON(0);
	CAMERA1_PWR_ON(0);
	CAMERA2_PWR_ON(0);

	CLR_MAIN_SW_ON(0);

	ROTATE_UD_ON(0);
	ROTATE_RL_ON(0);
	LCD_POCB_ON(0);

	LCD_PWM1_ON(0);
	LCD_PWM2_ON(0);
	LCD_PWR_ON(0);

	LVDS_PDWN_ON(0);

	GPS_ON(0);

	// enable all outputs
	SMBUS_EN(1);
	CODEC_PD_EN(1);
	RADIO_EN(1);
	COM_EXPRESS_PG_EN(1);
	COM_EXPRESS_EN(1);
	CAMERA1_PWR_EN(1);
	CAMERA2_PWR_EN(1);

	CLR_MAIN_SW_EN(1);

	ROTATE_UD_EN(1);
	ROTATE_RL_EN(1);
	LCD_POCB_EN(1);

	LCD_PWM1_EN(1);
	LCD_PWM2_EN(1);
	LCD_PWR_EN(1);

	LVDS_PDWN_EN(1);

	GPS_EN(1);
	
	// start turning things on

	__delay_ms(40);
	SMBUS_ON(1);
	CODEC_PD_ON(1);
	RADIO_ON(1);
	GPS_ON(1);
	COM_EXPRESS_ON(1);
	LCD_PWR_EN(1); // we may want to delay this until the OS starts to boot
	// start LCD PWM 

	// tell the COMExpress it's good to go!
	__delay_ms(30);
	COM_EXPRESS_PG_ON(1);
}

void DeviceBatteryProcessIO()
{
    
}
