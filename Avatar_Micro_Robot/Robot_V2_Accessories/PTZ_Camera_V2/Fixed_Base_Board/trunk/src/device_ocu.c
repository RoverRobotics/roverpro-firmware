/**
 * @file device_battery.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex battery PIC firmware.
 *
 */

#include "stdhdr.h"
#include "device_ocu.h"

//button inputs
#define VOLUME_DOWN()	(_RG6)
#define VOLUME_UP()		(_RG7)
#define TOGGLE1_UP() 	(_RB15)
#define TOGGLE1_DOWN()	(_RB8)
#define TOGGLE2_DOWN()	(_RB14)
#define TOGGLE2_UP()	(_RB9)
#define TALK_BUTTON()	(!_RB3)
#define MENU_BUTTON()	(!_RB2)

#define LIGHT_BUTTON()	(!_RB4)
#define POWER_BUTTON()	(!_RD0)

//joystick inputs
#define JOY1_X_EN(a)	_PCFG11 = !a
#define JOY1_Y_EN(a)	_PCFG12 = !a
#define JOY2_X_EN(a)	_PCFG10 = !a
#define JOY2_Y_EN(a)	_PCFG13 = !a

//joystick AN pin numbers
#define JOY1_X_CH		11
#define JOY1_Y_CH		12
#define JOY2_X_CH		10
#define JOY2_Y_CH		13

//indicators of COM Express power state
#define SUS_S5			_RC14
#define SUS_S3			_RD6

//indicates that COM Express has overheated
#define NC_THERM_TRIP	_RD2

//outputs
#define CHARGER_EN(a)	_TRISD11 = !a
#define CHARGER_ON(a)	_LATD11 = a

//power button on COM Express
#define COMPUTER_PWR_EN(a)	_TRISB1 = !a
#define COMPUTER_PWR_ON(a)	_LATB1 = a

#define V3V3_EN(a)		_TRISE7 = !a
#define V3V3_ON(a)		_LATE7 = a

#define V5V_EN(a)		_TRISE5 = !a
#define V5V_ON(a)		_LATE5 = a

#define V12V_EN(a)		_TRISE6	= !a
#define V12V_ON(a)		_LATE6 = !a

#define COMPUTER_PWR_OK_EN(a)	_TRISD1 = !a
#define COMPUTER_PWR_OK(a)		_LATD1 = a

#define GREEN_LED_EN(a)		_TRISD8 = !a
#define GREEN_LED_ON(a)		_LATD8 = a

#define RED_LED_EN(a)		_TRISD3 = !a
#define RED_LED_ON(a)		_LATD3 = a

#define GPS_TX_OR			_RP27R
#define GPS_RX_PIN			19

#define CAMERA_PWR_EN(a)	_TRISB5 = !a
#define CAMERA_PWR_ON(a)	_LATB5 = a		

#define LCD_PWM_OR			_RP25R

#define LCD_BACK_EN(a)	TRISDbits.TRISD4 = !a
#define LCD_BACK_ON(a)	LATDbits.LATD4 = a

#pragma code

void DeviceOcuInit()
{


	V3V3_EN(1);
	V5V_EN(1);
	V3V3_ON(1);
	V5V_ON(1);
	GREEN_LED_EN(1);
	block_ms(1000);
	COMPUTER_PWR_OK(1);


	LCD_BACK_EN(1);
	LCD_BACK_ON(1);

	
	while(!POWER_BUTTON());
	GREEN_LED_ON(1);


}



void DeviceOcuProcessIO()
{
    
}
