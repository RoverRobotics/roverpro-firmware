/**
 * @file device_battery.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex battery PIC firmware.
 *
 */

/*#define SMBUS_ON(a)              _RG6     = a
#define SMBUS_EN(a)              _TRISG6  = !a

#define CODEC_PD_ON(a)           _RG7     = a
#define CODEC_PD_EN(a)           _TRISG7  = !a

#define RADIO_ON(a)              _RB8     = a
#define RADIO_EN(a)              _TRISB8  = !a

#define COM_EXPRESS_PG_ON(a)     _RB2     = a
#define COM_EXPRESS_PG_EN(a)     _TRISB2  = !a

#define COM_EXPRESS_ON(a)        _RF1     = a
#define COM_EXPRESS_EN(a)        _TRISF1  = !a

//#define CAMERA1_PWR_ON(a)        _RB5     = a
//#define CAMERA1_PWR_EN(a)        _TRISB5  = !a

#define CAMERA2_PWR_ON(a)        _RF3     = a
#define CAMERA2_PWR_EN(a)        _TRISF3  = !a

#define CLR_MAIN_SW_ON(a)        _RD11    = !a // inverse logic
#define CLR_MAIN_SW_EN(a)        _TRISD11 = !a

#define ROTATE_UD_ON(a)          _RC13    = a
#define ROTATE_UD_EN(a)          _TRISC13 = !a

#define ROTATE_RL_ON(a)          _RC14    = a
#define ROTATE_RL_EN(a)          _TRISC14 = !a

//define GPS_ON(a)                _RF5     = a // inverse logic
//#define GPS_EN(a)                _TRISF5  = !a

/*#define LCD_PWM1_ON(a)           _RD4     = a
#define LCD_PWM1_EN(a)           _TRISD4  = !a

#define LCD_PWM2_ON(a)           _RD3     = a
#define LCD_PWM2_EN(a)           _TRISD3  = !a

#define LVDS_PDWN_ON(a)          _RE5     = a
#define LVDS_PDWN_EN(a)          _TRISE5  = !a

#define LCD_POCB_ON(a)           _RE6     = a
#define LCD_POCB_EN(a)           _TRISE6  = !a

#define LCD_PWR_ON(a)            _RE7     = a
#define LCD_PWR_EN(a)            _TRISE7  = !a


#define SMB_SDA // SMBus data i/o
#define SMB_SCL // SMBus clk

#define SUS_S5 // inverse logic
#define NC_THRMTRIP // inverse logic
#define SUS_S3 // inverse logic
#define SMBUS_ALERT // inverse logic
#define PWRBTN // inverse logic (output)
#define SECURE_PIC


#define NCOM_WAKE0  // output
#define NCOM_WAKE1  // output
#define NCOM_SYSRST // output

#define GPS_RXD
#define GPS_TXD

#define GPS_1PPS // input
#define GPS_LCKFIX // input
#define GPS_RFPWRUP // input*/


#define OCU_V3V3_ON(a)               _RC13    = a
#define OCU_V3V3_EN(a)               _TRISC13 = !a*/

#define OCU_V5_ON(a)                 _RD11    = a
#define OCU_V5_EN(a)                 _TRISD11 = !a

#define OCU_V12_ON(a)                _RD0     = !a // inverted logic
#define OCU_V12_EN(a)                _TRISD0  = !a	//pin 46

/*#define OCU_V3V3_CHRG_ON(a)          _RC14    = a
#define OCU_V3V3_CHRG_EN(a)          _TRISC14 = !a

#define OCU_JSTICK_ON(a)             _RG6     = a
#define OCU_JSTICK_EN(a)             _TRISG6  = !a

#define OCU_FAN_SET_ON(a)            _RD3     = a
#define OCU_FAN_SET_EN(a)            _TRISD3  = !a*/

#define OCU_CHRGR_EN_ON(a)           _RE5     = a	//pin 1
#define OCU_CHRGR_EN_EN(a)           _TRISE5  = !a

/*#define OCU_V3V3_ALWS_GOOD()         (_RF5)
#define OCU_V3V3_CHRG_GOOD()         (_RF4)*/

/*#define OCU_COMEXPRESS_EN(a)		_TRISF1		= !a
#define OCU_COMEXPRESS_ON(a)		_RF1		= a

#define OCU_COMEXPRESS_PGOOD_EN(a)	_TRISB2		= !a
#define OCU_COMEXPRESS_PGOOD_ON(a)	_RB2		= a*/

/*#define OCU_SMBBATT_SDA // bidirectional SMBus
#define OCU_SMBATT_SCL  // SMBus clk*/


/*#define OCU_JSTICK1_X_POS // analog input (AN3)
#define OCU_JSTICK1_Y_POS // analog input (AN2)
#define OCU_JSTICK1_BUTTON()         (_RB8)

#define OCU_EXT_TEMP_SENSOR // analog input (AN0)

#define OCU_TOGGLE2_DWN()            (_RB9)
#define OCU_TOGGLE2_UP()             (_RB14)
#define OCU_TOGGLE1_DWN()            (_RB15)
#define OCU_TOGGLE1_UP()             (_RG9)

#define OCU_VOLUME_DWN()             (_RG8)
#define OCU_VOLUME_UP()              (_RG7)

#define OCU_TEMP_FAN_ALERT           (!_RD5)
#define OCU_FAN_OVER_INT             (!_RD4)

#define OCU_BUTTON1                  (_RB13)
#define OCU_BUTTON2                  (_RB12)
#define OCU_BUTTON3                  (_RD7) // (not actually correct on REV 1)

#define OCU_JSTICK2_X_POS // analog input (AN10) (not actually correct on REV. 1)
#define OCU_JSTICK2_Y_POS // analog input (AN11) (not actually correct on REV. 1)
#define OCU_JSTICK2_BUTTON           (_RD8)

#define OCU_LED_EN(a)		_TRISE7	= !a
#define OCU_LED_ON(a)		_LATE7 = a

#define OCU_REAR_LED_EN(a)		_TRISE6	= !a
#define OCU_REAR_LED_ON(a)		_LATE6 = a*/

/*#define OCU_CHARGE_EN(a)		_TRISE5	= !a
#define OCU_CHARGE_ON(a)		_LATE5 = a*/

//#define OCU_ACOK_EN(a)			_TRISF3 = a
#define OCU_ACOK				_LATF3

/*#define OCU_12V_EN(a)		_TRISD0	= !a
#define OCU_12V_ON(a)		_LATD0 = !a*/

void DeviceBatteryInit();

void DeviceBatteryProcessIO();
