/**
 * @file device_battery.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex battery PIC firmware.
 *
 */

#define SMBUS_ON(a)              _RG6     = a
#define SMBUS_EN(a)              _TRISG6  = !a

#define CODEC_PD_ON(a)           _RG7     = a
#define CODEC_PD_EN(a)           _TRISG7  = !a

#define RADIO_ON(a)              _RB8     = a
#define RADIO_EN(a)              _TRISB8  = !a

#define COM_EXPRESS_PG_ON(a)     _RB2     = a
#define COM_EXPRESS_PG_EN(a)     _TRISB2  = !a

#define COM_EXPRESS_ON(a)        _RF1     = a
#define COM_EXPRESS_EN(a)        _TRISF1  = !a

#define CAMERA1_PWR_ON(a)        _RB5     = a
#define CAMERA1_PWR_EN(a)        _TRISB5  = !a

#define CAMERA2_PWR_ON(a)        _RF3     = a
#define CAMERA2_PWR_EN(a)        _TRISF3  = !a

#define CLR_MAIN_SW_ON(a)        _RD11    = !a // inverse logic
#define CLR_MAIN_SW_EN(a)        _TRISD11 = !a

#define ROTATE_UD_ON(a)          _RC13    = a
#define ROTATE_UD_EN(a)          _TRISC13 = !a

#define ROTATE_RL_ON(a)          _RC14    = a
#define ROTATE_RL_EN(a)          _TRISC14 = !a

#define GPS_ON(a)                _RF5     = a // inverse logic
#define GPS_EN(a)                _TRISF5  = !a

#define LCD_PWM1_ON(a)           _RD4     = a
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
#define GPS_RFPWRUP // input

void DeviceBatteryInit();

void DeviceBatteryProcessIO();
