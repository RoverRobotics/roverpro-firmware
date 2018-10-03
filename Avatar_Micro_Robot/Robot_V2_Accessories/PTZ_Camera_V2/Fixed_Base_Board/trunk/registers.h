/*=============================================================================
File: Registers.h 

Description: This is where the global hardware registers are defined between 
  software and hardware. Device variables should only be declared here 
  NOT locally).  Data types should be multiples of whole bytes (NO bitfields). 
  New data types should be added only to the end of the list to maintain 
  backwards compatibility.

Notes:
  - adapted from a file originally written by J. Brinton
  - all naming is from the perspective of software (ie whether software
    reads-from or write-to the register
  - REGISTER(MY_REGISTER_NAME, SOFTWARE_R/W_DIRECTION, 
             SYNC_OR_NO_SYNC, my_data_type)
  
Responsible Engineer(s):
=============================================================================*/
#ifndef REGISTERS_H
#define REGISTERS_H

/*---------------------------Macros------------------------------------------*/
#define BUTTON_DEPRESSED 1
#define REGISTER_START()
#define REGISTER_END()

/*---------------------------Type Definitions--------------------------------*/
typedef struct { uint8_t data[8]; } CRYPTO_DATA;
typedef struct { int16_t left, right, flipper; } MOTOR_DATA_3EL_16BI;

typedef struct { int16_t left, right, board; } TMP_3EL_16BI;
typedef struct { int16_t left, right; } MOTOR_DATA_2EL_32BI;
typedef struct { int16_t fan1, fan2; } FAN_DATA_2EL_16BI;
typedef struct { int8_t  left, right; } MOTOR_DATA_2EL_8BI;
typedef struct { int16_t pot1, pot2; } FLIPPER_DATA_2EL_16BI; 
typedef struct { int32_t left, right, flipper; } MOTOR_DATA_3EL_32BI;
typedef struct { int8_t  left, right, flipper; } MOTOR_DATA_3EL_8BI;
typedef struct { float   data[4][3]; } MOTOR_DATA_CTRL;
typedef struct { int16_t a,b; } BATTERY_DATA_2EL_16BI;
typedef struct { uint16_t deg, min, sec; } GPS_VECT;
typedef struct { GPS_VECT lat, lon; } GPS_DATA;
typedef struct {uint8_t data[100]; } GPS_MESSAGE;
typedef struct {uint8_t data[24]; } FIRMWARE_BUILD_STRING;
typedef struct {uint8_t data[79]; } BOARD_DATA;
typedef struct { uint32_t length, magic; } UPDATE_FIRMWARE; // magic = 0x2345BCDE

typedef struct { int16_t tilt, zoom; } REG_CAMERA_VEL_ROT_2EL_16BI;
// PTZ data type for representing tilt, zoom, and digital zoom data
typedef struct { int16_t tilt, zoom, digitalZoom; } REG_CAMERA_POS_ROT_3EL_16BI;
typedef struct { int16_t turret, shoulder, elbow, wrist, gripper; } ARM_DATA_5EL_16BI;
typedef struct { uint16_t turret, shoulder, elbow, wrist, gripper_actuator, gripper; } ARM_DATA_6EL_16BI;

// Hitch data type for representing linear position of the latch
typedef unsigned char hitch_t;

#endif


/*---------------------------Hardware Register Definitions-------------------*/
REGISTER_START()

//REGISTER INDEX: 0x0000 (0)
REGISTER( REG_TEST_VAR,            DEVICE_READ,  DEVICE_GENERIC, SYNC,    float )
REGISTER( REG_TEST_VAR2,           DEVICE_WRITE, DEVICE_GENERIC, SYNC,    int16_t )
REGISTER( REG_CRYPTO,              DEVICE_READ,  DEVICE_CARRIER, NO_SYNC, CRYPTO_DATA )

REGISTER( REG_TELEMETRY_COUNT,     DEVICE_READ,  DEVICE_CARRIER, SYNC,    uint16_t )
REGISTER( REG_PAYLOAD_1_PRESENT,   DEVICE_READ,  DEVICE_CARRIER, SYNC,    uint8_t )
REGISTER( REG_PAYLOAD_2_PRESENT,   DEVICE_READ,  DEVICE_CARRIER, SYNC,    uint8_t )
REGISTER( REG_CAMERA_PRESENT,      DEVICE_READ,  DEVICE_CARRIER, SYNC,    uint8_t )
REGISTER( REG_RAW_POWER_GOOD,      DEVICE_READ,  DEVICE_CARRIER, SYNC,    uint8_t )
REGISTER( REG_V12_POWER_GOOD,      DEVICE_READ,  DEVICE_CARRIER, SYNC,    uint8_t )
REGISTER( REG_V5_POWER_GOOD,       DEVICE_READ,  DEVICE_CARRIER, SYNC,    uint8_t )
REGISTER( REG_V14_VALUE,           DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )
REGISTER( REG_ACCEL_X,             DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )
REGISTER( REG_ACCEL_Y,             DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )
REGISTER( REG_ACCEL_Z,             DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )

REGISTER( REG_TEMP_INT0,           DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )
REGISTER( REG_TEMP_INT1,           DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )

//REGISTER INDEX: 0x0010 (16)
REGISTER( REG_TEMP_EXT0,           DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )

REGISTER( REG_JOYSTICK1_X,         DEVICE_READ,  DEVICE_OCU,     SYNC,    uint16_t )
REGISTER( REG_JOYSTICK1_Y,         DEVICE_READ,  DEVICE_OCU,     SYNC,    uint16_t )
REGISTER( REG_JOYSTICK2_X,         DEVICE_READ,  DEVICE_OCU,     SYNC,    uint16_t )
REGISTER( REG_JOYSTICK2_Y,         DEVICE_READ,  DEVICE_OCU,     SYNC,    uint16_t )
REGISTER( REG_SWITCH1_UP,          DEVICE_READ,  DEVICE_OCU,     SYNC,    uint8_t )
REGISTER( REG_SWITCH1_DOWN,        DEVICE_READ,  DEVICE_OCU,     SYNC,    uint8_t )
REGISTER( REG_SWITCH2_UP,          DEVICE_READ,  DEVICE_OCU,     SYNC,    uint8_t )
REGISTER( REG_SWITCH2_DOWN,        DEVICE_READ,  DEVICE_OCU,     SYNC,    uint8_t )
REGISTER( REG_VOLUME_UP,           DEVICE_READ,  DEVICE_OCU,     SYNC,    uint8_t )
REGISTER( REG_VOLUME_DOWN,         DEVICE_READ,  DEVICE_OCU,     SYNC,    uint8_t )
REGISTER( REG_POWER_DOWN,          DEVICE_READ,  DEVICE_OCU,     SYNC,    uint8_t )
REGISTER( REG_HOME_BUTTON,         DEVICE_READ,  DEVICE_OCU,     SYNC,    uint8_t )
REGISTER( REG_AUX_BUTTON1,         DEVICE_READ,  DEVICE_OCU,     SYNC,    uint8_t )
REGISTER( REG_AUX_BUTTON2,         DEVICE_READ,  DEVICE_OCU,     SYNC,    uint8_t )

REGISTER( REG_MOTOR_VELOCITY,      DEVICE_WRITE, DEVICE_MOTOR,   SYNC,    MOTOR_DATA_3EL_16BI)

//REGISTER INDEX: 0x0020 (32)
REGISTER( REG_FAN_VELOCITY,        DEVICE_WRITE, DEVICE_MOTOR,   SYNC,    FAN_DATA_2EL_16BI)
REGISTER( REG_MOTOR_FB_RPM,        DEVICE_READ,  DEVICE_MOTOR,   SYNC,    MOTOR_DATA_2EL_32BI)
REGISTER( REG_FLIPPER_FB_POSITION, DEVICE_READ,  DEVICE_MOTOR,   SYNC,    FLIPPER_DATA_2EL_16BI)
REGISTER( REG_MOTOR_FB_CURRENT,    DEVICE_READ,  DEVICE_MOTOR,   SYNC,    MOTOR_DATA_3EL_16BI)
REGISTER( REG_MOTOR_ENCODER_COUNT, DEVICE_READ,  DEVICE_MOTOR,   SYNC,    MOTOR_DATA_2EL_32BI)
REGISTER( REG_MOTOR_KP,            DEVICE_WRITE, DEVICE_MOTOR,   SYNC,    MOTOR_DATA_CTRL)
REGISTER( REG_MOTOR_KI,            DEVICE_WRITE, DEVICE_MOTOR,   SYNC,    MOTOR_DATA_CTRL)
REGISTER( REG_MOTOR_KD,            DEVICE_WRITE, DEVICE_MOTOR,   SYNC,    MOTOR_DATA_CTRL)
REGISTER( REG_MOTOR_CTRL_MODE,     DEVICE_WRITE, DEVICE_MOTOR,   SYNC,    MOTOR_DATA_3EL_16BI)
REGISTER( REG_MOTOR_FAULT_FLAG,    DEVICE_READ,  DEVICE_MOTOR,   SYNC,    MOTOR_DATA_2EL_8BI)
REGISTER( REG_MOTOR_TEMP,          DEVICE_READ,  DEVICE_MOTOR,   SYNC,    TMP_3EL_16BI)

REGISTER( REG_MOTOR_TEMP_STATUS,   DEVICE_READ,  DEVICE_MOTOR,   SYNC,    TMP_3EL_16BI)//0-data is bad, 1-data is good
REGISTER( REG_PWR_BAT_VOLTAGE,     DEVICE_READ,  DEVICE_MOTOR,   SYNC,    BATTERY_DATA_2EL_16BI)
REGISTER( REG_PWR_TOTAL_CURRENT,   DEVICE_READ,  DEVICE_MOTOR,   SYNC,    uint16_t)



REGISTER( REG_CARRIER_STATE,       DEVICE_WRITE, DEVICE_CARRIER, SYNC,    uint8_t )
REGISTER( REG_MAGNETIC_X,          DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )

//REGISTER INDEX: 0x0030 (48)
REGISTER( REG_MAGNETIC_Y,          DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )
REGISTER( REG_MAGNETIC_Z,          DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )
REGISTER( REG_ROBOT_HUMIDITY,      DEVICE_READ,  DEVICE_CARRIER, SYNC,    float )
REGISTER( REG_WHITE_LED,           DEVICE_WRITE, DEVICE_CARRIER, SYNC,    uint8_t )
   
REGISTER( REG_OCU_TEMP_1,          DEVICE_READ,  DEVICE_OCU, SYNC,        float )
REGISTER( REG_OCU_TEMP_2,          DEVICE_READ,  DEVICE_OCU, SYNC,        float )
REGISTER( REG_OCU_ACCEL_X,         DEVICE_READ,  DEVICE_OCU, SYNC,        float )
REGISTER( REG_OCU_ACCEL_Y,         DEVICE_READ,  DEVICE_OCU, SYNC,        float )
REGISTER( REG_OCU_ACCEL_Z,         DEVICE_READ,  DEVICE_OCU, SYNC,        float)
REGISTER( REG_OCU_MAGNETIC_X,      DEVICE_READ,  DEVICE_OCU, SYNC,        float )
REGISTER( REG_OCU_MAGNETIC_Y,      DEVICE_READ,  DEVICE_OCU, SYNC,        float )
REGISTER( REG_OCU_MAGNETIC_Z,      DEVICE_READ,  DEVICE_OCU, SYNC,        float )
REGISTER( REG_OCU_HUMIDITY,        DEVICE_READ,  DEVICE_OCU, SYNC,        float )

REGISTER( REG_OCU_TOUCH_PEN,       DEVICE_READ,  DEVICE_OCU, SYNC,        uint8_t )
REGISTER( REG_OCU_TOUCH_X,         DEVICE_READ,  DEVICE_OCU, SYNC,        int16_t )
REGISTER( REG_OCU_TOUCH_Y,         DEVICE_READ,  DEVICE_OCU, SYNC,        int16_t )

//REGISTER INDEX: 0x0040 (64)
REGISTER( REG_ROBOT_GPS,           DEVICE_READ,  DEVICE_CARRIER, SYNC,    GPS_DATA )
REGISTER( REG_IR_LED,              DEVICE_WRITE, DEVICE_CARRIER, SYNC,    uint8_t )

REGISTER( CMD_UPDATE_FIRMWARE,     DEVICE_WRITE, DEVICE_CARRIER, NO_SYNC, UPDATE_FIRMWARE)

REGISTER( REG_OCU_GPS_MESSAGE,        DEVICE_READ,  DEVICE_OCU, NO_SYNC,    GPS_MESSAGE )
REGISTER( REG_OCU_BATT_CURRENT,   DEVICE_READ,  DEVICE_OCU,   SYNC,    int16_t)
REGISTER( REG_OCU_BATT_REL_SOC,   DEVICE_READ,  DEVICE_OCU,   SYNC,    uint16_t)
REGISTER( REG_OCU_BATT_ABS_SOC,   DEVICE_READ,  DEVICE_OCU,   SYNC,    uint16_t)
REGISTER( REG_OCU_BATT_REM_CAP,   DEVICE_READ,  DEVICE_OCU,   SYNC,    uint16_t)
REGISTER( REG_OCU_BATT_VOLTAGE,   DEVICE_READ,  DEVICE_OCU,   SYNC,    uint16_t)
REGISTER( REG_OCU_BATT_FULL_CHARGE_CAP,   DEVICE_READ,  DEVICE_OCU,   SYNC,    uint16_t)
REGISTER( REG_OCU_FIRMWARE_BUILD,   DEVICE_READ,  DEVICE_OCU,   NO_SYNC,    FIRMWARE_BUILD_STRING)

REGISTER( REG_ROBOT_FIRMWARE_BUILD,   DEVICE_READ,  DEVICE_CARRIER,   NO_SYNC,    FIRMWARE_BUILD_STRING)
REGISTER( REG_ROBOT_GPS_MESSAGE,        DEVICE_READ,  DEVICE_CARRIER, NO_SYNC,    GPS_MESSAGE )
REGISTER( REG_ROBOT_BOARD_DATA,        DEVICE_READ,  DEVICE_CARRIER, NO_SYNC,    BOARD_DATA )

REGISTER( REG_ROBOT_REL_SOC_A, DEVICE_READ,  DEVICE_MOTOR, SYNC,        int16_t )
REGISTER( REG_ROBOT_REL_SOC_B, DEVICE_READ,  DEVICE_MOTOR, SYNC,        int16_t )

//REGISTER INDEX: 0x0050 (80)
REGISTER( REG_MOTOR_BOARD_DATA,        DEVICE_READ,  DEVICE_MOTOR, NO_SYNC,    BOARD_DATA )

REGISTER( REG_MOTOR_FIRMWARE_BUILD,   DEVICE_READ,  DEVICE_MOTOR,   NO_SYNC,    FIRMWARE_BUILD_STRING)

REGISTER( REG_CARRIER_SPEAKER_ON,   DEVICE_WRITE,  DEVICE_CARRIER,   SYNC,    uint8_t )
REGISTER( REG_CARRIER_MIC_ON,   DEVICE_WRITE,  DEVICE_CARRIER,   SYNC,    uint8_t )

REGISTER( REG_CAMERA_VEL_BASE,	DEVICE_WRITE,	DEVICE_PTZ_BASE,	SYNC,	int16_t )
REGISTER( REG_CAMERA_VEL_ROT,	DEVICE_WRITE,	DEVICE_PTZ_ROTATION,	SYNC,	REG_CAMERA_VEL_ROT_2EL_16BI )
REGISTER( REG_CAMERA_POS_BASE,	DEVICE_READ,	DEVICE_PTZ_BASE,	SYNC,	int16_t )
REGISTER( REG_CAMERA_BASE_FLIP,	DEVICE_WRITE,	DEVICE_PTZ_BASE,	SYNC,	int8_t )
REGISTER( REG_CAMERA_POS_ROT,	DEVICE_READ,	DEVICE_PTZ_ROTATION,	SYNC,	REG_CAMERA_POS_ROT_3EL_16BI )
REGISTER( REG_CAMERA_FOCUS, DEVICE_READ, DEVICE_PTZ_ROTATION, SYNC, int16_t )
REGISTER( REG_CAMERA_FOCUS_MANUAL, DEVICE_READ, DEVICE_PTZ_ROTATION, SYNC, uint8_t )
REGISTER( REG_CAMERA_FOCUS_SET, DEVICE_READ, DEVICE_PTZ_ROTATION, SYNC, uint16_t )

// value ranges from 0 (off) to 240 (100%)
REGISTER( REG_MOTOR_SIDE_FAN_SPEED,   DEVICE_WRITE,  DEVICE_MOTOR,   SYNC,    uint8_t )

REGISTER( REG_OCU_REL_SOC_L,   DEVICE_READ,  DEVICE_OCU,   SYNC,    uint16_t)
REGISTER( REG_OCU_REL_SOC_R,   DEVICE_READ,  DEVICE_OCU,   SYNC,    uint16_t)

REGISTER( REG_OCU_BACKLIGHT_BRIGHTNESS,   DEVICE_WRITE,  DEVICE_OCU,   SYNC,    uint8_t)

//REGISTER INDEX: 0x0060 (96)
REGISTER( REG_OCU_SPEAKER_ON,   DEVICE_WRITE,  DEVICE_CARRIER,   SYNC,    uint8_t )
REGISTER( REG_OCU_MIC_ON,   DEVICE_WRITE,  DEVICE_CARRIER,   SYNC,    uint8_t )

// turns off power to PTZ.  This will cause PIC and EMPIA chip to lose USB connection.
// PTZ Base PIC will re-enable power after 1000ms
REGISTER( REG_CAMERA_BASE_POWER_DOWN,	DEVICE_WRITE,	DEVICE_PTZ_BASE,	SYNC,	int8_t )

// value ranges from 0 (off) to 240 (100%)
REGISTER( REG_CARRIER_REAR_BLOWER_SPEED,   DEVICE_WRITE,  DEVICE_CARRIER,   SYNC,    uint8_t )

REGISTER( REG_CAMERA_SHUTTER,	DEVICE_READ,	DEVICE_PTZ_ROTATION,	SYNC,	uint16_t )

REGISTER( REG_PWR_A_CURRENT,   DEVICE_READ,  DEVICE_MOTOR,   SYNC,    uint16_t)
REGISTER( REG_PWR_B_CURRENT,   DEVICE_READ,  DEVICE_MOTOR,   SYNC,    uint16_t)

REGISTER( REG_ARM_MOTOR_VELOCITIES,   DEVICE_WRITE,  DEVICE_ARM_LINK2,   SYNC,    ARM_DATA_5EL_16BI )
REGISTER( REG_ARM_JOINT_POSITIONS,   DEVICE_READ,  DEVICE_ARM_LINK2,   SYNC,    ARM_DATA_6EL_16BI )


// Hitch Software Interface
REGISTER(REG_HITCH_OPEN, DEVICE_WRITE, DEVICE_HITCH, SYNC, hitch_t)    // non-zero is software desire to open, zero is software's desire to latch 
REGISTER(REG_HITCH_POSITION, DEVICE_READ, DEVICE_HITCH, SYNC, hitch_t) // 0 (unlatched) to 100 (latched)

REGISTER(REG_OCU_CAMERA_POWER_ON, DEVICE_WRITE, DEVICE_OCU, SYNC, uint8_t)

//  flipper angle, in degrees, from 0 to 359
REGISTER( REG_MOTOR_FLIPPER_ANGLE, DEVICE_READ,  DEVICE_MOTOR,   SYNC,    uint16_t)

//when nonzero, firmware will power cycle robot and copy value to REG_ROBOT_RESET_CODE
REGISTER( REG_ROBOT_RESET_REQUEST,   DEVICE_WRITE,  DEVICE_CARRIER,   SYNC,    uint16_t)
//tells sofware why the robot last reset.  Values 0x0000 through 0x00ff are reserved for
//firmware, and the rest come from REG_ROBOT_RESET_REQUEST
REGISTER( REG_ROBOT_RESET_CODE,   DEVICE_READ,  DEVICE_CARRIER,   SYNC,    uint16_t)

//register that is nonzero whenever the AC Adapter is plugged in
REGISTER( REG_OCU_AC_POWER,   DEVICE_READ,  DEVICE_OCU,   SYNC,    uint8_t)

//register that is nonzero when radio should be powered down or in reset, and zero
//during normal operation
REGISTER( REG_ROBOT_RADIO_RESET,   DEVICE_WRITE,  DEVICE_CARRIER,   SYNC,    uint8_t)

//registers for the boom camera.  These should be used similar to the registers for
//the PTZ.
REGISTER( REG_BOOM_VEL_PAN,	DEVICE_WRITE,	DEVICE_BOOM_CAM,	SYNC,	int16_t )
REGISTER( REG_BOOM_VEL_TILT,	DEVICE_WRITE,	DEVICE_BOOM_CAM,	SYNC,	int16_t )
REGISTER( REG_BOOM_VEL_ZOOM,	DEVICE_WRITE,	DEVICE_BOOM_CAM,	SYNC,	int16_t )
REGISTER( REG_BOOM_POWER_DOWN,	DEVICE_WRITE,	DEVICE_BOOM_CAM,	SYNC,	int8_t )

REGISTER( REG_MOTOR_CHARGER_STATE,	DEVICE_READ,	DEVICE_MOTOR,	SYNC,	uint16_t )

REGISTER( REG_CAMERA_BASE_FAN_ON,	DEVICE_WRITE,	DEVICE_PTZ_BASE,	SYNC,	uint8_t )
REGISTER( REG_CAMERA_ROT_FAN_ON,	DEVICE_WRITE,	DEVICE_PTZ_ROTATION,	SYNC,	uint8_t )

REGISTER( REG_OCU_FAN_ON,	DEVICE_WRITE,	DEVICE_OCU,	SYNC,	uint8_t )



REGISTER_END()

