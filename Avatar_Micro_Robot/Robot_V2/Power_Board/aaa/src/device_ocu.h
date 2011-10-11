/**
 * @file device_ocu.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex OCU PIC firmware.
 *
 */

#define OCU_V3V3_ON(a)               _RC13    = a
#define OCU_V3V3_EN(a)               _TRISC13 = !a

#define OCU_V5_ON(a)                 _RD11    = a
#define OCU_V5_EN(a)                 _TRISD11 = !a

#define OCU_V12_ON(a)                _RD0     = !a // inverted logic
#define OCU_V12_EN(a)                _TRISD0  = !a

#define OCU_V3V3_CHRG_ON(a)          _RC14    = a
#define OCU_V3V3_CHRG_EN(a)          _TRISC14 = !a

#define OCU_JSTICK_ON(a)             _RG6     = a
#define OCU_JSTICK_EN(a)             _TRISG6  = !a

#define OCU_FAN_SET_ON(a)            _RD3     = a
#define OCU_FAN_SET_EN(a)            _TRISD3  = !a

#define OCU_CHRGR_EN_ON(a)           _RE5     = a
#define OCU_CHRGR_EN_EN(a)           _TRISE5  = !a

#define OCU_V3V3_ALWS_GOOD()         (_RF5)
#define OCU_V3V3_CHRG_GOOD()         (_RF4)

#define OCU_SMBBATT_SDA // bidirectional SMBus
#define OCU_SMBATT_SCL  // SMBus clk


#define OCU_JSTICK1_X_POS // analog input (AN3)
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

void DeviceOcuInit();

void DeviceOcuProcessIO();
