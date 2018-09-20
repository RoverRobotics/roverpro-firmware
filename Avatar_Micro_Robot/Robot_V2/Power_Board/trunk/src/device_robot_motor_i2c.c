#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "debug_uart.h"
#include "i2clib.h"
#include "device_robot_motor_i2c.h"

int I2C2XmitReset = 0;
int I2C3XmitReset = 0;

int I2C2TimerExpired = 0;
int I2C3TimerExpired = 0;

// PIC24-specific breakpoint command:
#define BREAKPOINT __asm__ __volatile__ (".pword 0xDA4000")
#ifdef __DEBUG
	#define BREAK_IF(condition) if(condition){BREAKPOINT;}
#else
	#define BREAK_IF(condition)
#endif

const i2c_busdef_t I2C2_meta = { (i2c_con_t *) &I2C2CON, (i2c_stat_t *) &I2C2STAT, &I2C2TRN, &I2C2RCV };
const i2c_busdef_t I2C3_meta = { (i2c_con_t *) &I2C3CON, (i2c_stat_t *) &I2C3STAT, &I2C3TRN, &I2C3RCV };

// A step may be reached by either falling through into it (first try)
// Or by jumping back to it (every retry)
// Note an i2c_result of I2C_ILLEGAL probably means there is a coding error, hence the conditional breakpoint
#define STEP(cmd)                                 \
	 case (__LINE__): resume_at=__LINE__;        \
		 i2c_result = (cmd);                      \
		 if (i2c_result == I2C_NOTYET) { break; } \
		 BREAK_IF(i2c_result != I2C_OKAY)         \
	 // fallthrough to next case

//fan controller, TMP112, battery, EEPROM
void I2C2Update(void)
{
	static const i2c_bus_t BUS = (&I2C2_meta);
 	static int resume_at=0; 
	static unsigned char a,b;
	static i2c_ack_t ack;
	i2c_result_t i2c_result;
	
 	if(I2C2XmitReset==True)
 	{
 		I2C2XmitReset=False;//clear the flag
 		resume_at=0; //start from the beginning
 		//assume all the data is good
 		REG_MOTOR_TEMP_STATUS.board=1;
 	}
 	switch(resume_at)
 	{
	 	default: BREAKPOINT;
	 	case 0:
	 	
	// REG_MOTOR_TEMP.left = FanControl ReadByte 0x00
		STEP(i2c_start(BUS))
		STEP(i2c_write_addr(BUS, FAN_CONTROLLER_ADDRESS, I2C_WRITE))
		STEP(i2c_check_ack(BUS, &ack))
		STEP(i2c_write_byte(BUS, 0x01))
		STEP(i2c_start(BUS))
		STEP(i2c_write_addr(BUS, FAN_CONTROLLER_ADDRESS, I2C_READ))
		STEP(i2c_check_ack(BUS, &ack));
		STEP(i2c_receive(BUS))
		STEP(i2c_read_byte(BUS, NACK, &a))
			REG_MOTOR_TEMP_STATUS.left = (ack == ACK);
			REG_MOTOR_TEMP.left = a;
 			
 	// REG_MOTOR_TEMP_STATUS.right = FanControl ReadByte 0x01
 		STEP(i2c_restart(BUS))
 		STEP(i2c_write_addr(BUS, FAN_CONTROLLER_ADDRESS, I2C_WRITE))
		STEP(i2c_write_byte(BUS, 0x01))
 		STEP(i2c_start(BUS))
 		STEP(i2c_write_addr(BUS, FAN_CONTROLLER_ADDRESS, I2C_READ))
 		STEP(i2c_check_ack(BUS, &ack))
 		STEP(i2c_receive(BUS))
		STEP(i2c_read_byte(BUS, NACK, &a))
			REG_MOTOR_TEMP_STATUS.right = (ack == ACK);
			REG_MOTOR_TEMP.right = a;
 			
 	// REG_ROBOT_REL_SOC_A = Battery ReadWord 0x0d [="RelativeStateOfCharge"]
 		STEP(i2c_restart(BUS))
 		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
 		STEP(i2c_write_byte(BUS, 0x0d))	
 		STEP(i2c_start(BUS))
 		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
 		STEP(i2c_receive(BUS))
 		STEP(i2c_read_byte(BUS, ACK, &a))
 		STEP(i2c_receive(BUS))
 		STEP(i2c_read_byte(BUS, NACK, &b))
			REG_ROBOT_REL_SOC_A = a + (b<<8);
		
	// FanControl WriteByte 0x0b REG_MOTOR_SIDE_FAN_SPEED
		STEP(i2c_restart(BUS))
		STEP(i2c_write_addr(BUS, FAN_CONTROLLER_ADDRESS, I2C_WRITE))
		STEP(i2c_write_byte(BUS, 0x0b))
		STEP(i2c_write_byte(BUS, REG_MOTOR_SIDE_FAN_SPEED))
			
	// Battery ReadWord 0x16 [="BatteryStatus"]
		STEP(i2c_restart(BUS))
		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
		STEP(i2c_write_byte(BUS, 0x16))
		STEP(i2c_start(BUS))
		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
		STEP(i2c_check_ack(BUS, &ack))
		STEP(i2c_receive(BUS))
		STEP(i2c_read_byte(BUS, ACK, &a))
		STEP(i2c_receive(BUS))
		STEP(i2c_read_byte(BUS, NACK, &b))
			if(ack == ACK) {
				unsigned battery_status = a + (b<<8);
				if ((battery_status & 0x000f) == 0) { // if battery status returns no error code
					BREAK_IF(battery_status & 0xff00); // 0xff00 are the battery alarm flags
					REG_BATTERY_A_HEALTH.status = battery_status;
				}
			}
			
		// Battery ReadWord 0x03 [="BatteryMode"]
		STEP(i2c_restart(BUS))
		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
		STEP(i2c_write_byte(BUS, 0x03))
		STEP(i2c_start(BUS))
		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
		STEP(i2c_check_ack(BUS, &ack))
		STEP(i2c_receive(BUS))
		STEP(i2c_read_byte(BUS, ACK, &a))
		STEP(i2c_receive(BUS))
		STEP(i2c_read_byte(BUS, NACK, &b))
			if(ack == ACK) {
				REG_BATTERY_A_HEALTH.mode = a + (b<<8);
			}
			
		STEP(i2c_stop(BUS))
 			I2C2TimerExpired=False;//reset the I2C2 update timer
 			resume_at=-1;
 	}
}

void I2C3Update(void)
{
	static const i2c_bus_t BUS = (&I2C3_meta);
 	static int resume_at=0;
	static unsigned char a,b;
	i2c_result_t i2c_result;
	
 	if(I2C3XmitReset==True)
 	{
 		I2C3XmitReset=False;//clear the flag
 		resume_at=0; //  start from the beginning
 	}
 	switch(resume_at)
 	{
	 	default: BREAKPOINT;
	 	case 0:
	 	
	 // Battery ReadWord 0x0d [="RelativeStateOfCharge"]
	 	STEP(i2c_start(BUS))
 		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
 		STEP(i2c_write_byte(BUS, 0x0d))
 		STEP(i2c_start(BUS))
 		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
 		STEP(i2c_receive(BUS))
 		STEP(i2c_read_byte(BUS, ACK, &a))
 		STEP(i2c_receive(BUS))
 		STEP(i2c_read_byte(BUS, NACK, &b))
			REG_ROBOT_REL_SOC_B = a + (b<<8);
 			
 	// REG_MOTOR_CHARGER_STATE = BatteryCharger ReadWord 0xca
 		STEP(i2c_restart(BUS))
 		STEP(i2c_write_addr(BUS, BATTERY_CHARGER_ADDRESS, I2C_WRITE))
 		STEP(i2c_write_byte(BUS, 0xca))
 		STEP(i2c_start(BUS))
 		STEP(i2c_write_addr(BUS, BATTERY_CHARGER_ADDRESS, I2C_READ))
 		STEP(i2c_receive(BUS))
 		STEP(i2c_read_byte(BUS, ACK, &a))
 		STEP(i2c_receive(BUS))
 		STEP(i2c_read_byte(BUS, NACK, &b))
			REG_MOTOR_CHARGER_STATE = a + (b<<8);
 			
 	// Battery ReadWord 0x16 [="BatteryStatus"]
 		STEP(i2c_restart(BUS))
 		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
		STEP(i2c_write_byte(BUS, 0x16))
		STEP(i2c_start(BUS))
		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
		STEP(i2c_receive(BUS))
		STEP(i2c_read_byte(BUS, ACK, &a))
		STEP(i2c_receive(BUS))
		STEP(i2c_read_byte(BUS, NACK, &b))
			unsigned battery_status = a + (b<<8);
			if ((battery_status & 0x000f) == 0) { // if battery status returns no error code
				BREAK_IF(battery_status & 0xff00); // 0xff00 are the battery alarm flags
				REG_BATTERY_B_HEALTH.status = battery_status;
			}	
			
	// Battery ReadWord 0x03 [="BatteryMode"]
		STEP(i2c_restart(BUS))
		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
		STEP(i2c_write_byte(BUS, 0x03))
		STEP(i2c_start(BUS))
		STEP(i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
		STEP(i2c_receive(BUS))
		STEP(i2c_read_byte(BUS, ACK, &a))
		STEP(i2c_receive(BUS))
		STEP(i2c_read_byte(BUS, NACK, &b))
			REG_BATTERY_B_HEALTH.mode = a + (b<<8);
	
		STEP(i2c_stop(BUS))
 			I2C3TimerExpired=False; //reset the I2C2 update timer
 			resume_at=-1;
	}
}

//*********************************************//

void re_init_i2c2(void)
{
  I2C2CON = 0;
  I2C2STAT = 0;

// New Bren-Tronics battery (or existing device interacting with the new
// battery ties up the SMBus line (usually
// seen after the robot is driven for a short time.  The only
// way I've been able to recover from this is by either removing
// the SMBus cable between the battery board and the power board,
// or by changing the i2c pins to outputs when the i2c module is
// disabled.
  _TRISF4 = 0;
  _TRISF5 = 0;
  _LATF4 = 0;
  _LATF5 = 0;

	//FCY should be 16M
	//I2C2BRG = FCY/100000-FCY/10000000-1;	//should be 157.4 (between 9D and 9E)
	I2C2BRG = 0xff;
  I2C2CONbits.I2CEN = 1;


}

void re_init_i2c3(void)
{
  I2C3CON = 0;
  I2C3STAT = 0;
 
  _TRISE6 = 0;
  _TRISE7 = 0;
  _LATE6 = 0;
  _LATE7 = 0;

	//FCY should be 16M
	//I2C2BRG = FCY/100000-FCY/10000000-1;	//should be 157.4 (between 9D and 9E)
	I2C3BRG = 0xff;
  I2C3CONbits.I2CEN = 1;
 
}
