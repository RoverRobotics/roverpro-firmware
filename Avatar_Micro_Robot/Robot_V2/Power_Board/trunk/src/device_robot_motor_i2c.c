#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "debug_uart.h"
#include "i2clib.h"
#include "device_robot_motor_i2c.h"

int CheckI2C2Idle();
int CheckI2C3Idle();

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

#define STEP(n, cmd)                              \
	 case (n):                                    \
		 i2c_result = (cmd);                      \
		 BREAK_IF(i2c_result == I2C_ILLEGAL)      \
		 if (i2c_result == I2C_NOTYET) { break; } \
		 else {StepNumber = (n)+1;}               \
	 break;
  
//fan controller, TMP112, battery, EEPROM
void I2C2Update(void)
{
 	static int StepNumber=1; 
	static unsigned char a,b;
	static i2c_ack_t ack;
	i2c_result_t i2c_result;
	
 	if(I2C2XmitReset==True)
 	{
 		I2C2XmitReset=False;//clear the flag
 		StepNumber=1;//go to step 1, start from the beginning
 		//assume all the data is good
 		REG_MOTOR_TEMP_STATUS.left=1;
 		REG_MOTOR_TEMP_STATUS.right=1;
 		REG_MOTOR_TEMP_STATUS.board=1;
 	}
#define BUS (&I2C2_meta)
 	switch(StepNumber)
 	{
	// REG_MOTOR_TEMP.left = FanControl ReadByte 0x00
		STEP(1, i2c_start(BUS))
		STEP(2, i2c_write_addr(BUS, FAN_CONTROLLER_ADDRESS, I2C_WRITE))
		STEP(3, i2c_check_ack(BUS, &ack))
		STEP(4, i2c_write_data(BUS, 0x01))
		STEP(5, i2c_start(BUS))
		STEP(6, i2c_write_addr(BUS, FAN_CONTROLLER_ADDRESS, I2C_READ))
		STEP(7, i2c_check_ack(BUS, &ack));
		STEP(8, i2c_receive(BUS))
		STEP(9, i2c_read_byte(BUS, NACK, &a))
		case 10:
			REG_MOTOR_TEMP_STATUS.left = (ack == ACK);
 			REG_MOTOR_TEMP.left = a;
 			StepNumber++;
 			// fallthrough
 			
 	// REG_MOTOR_TEMP_STATUS.right = FanControl ReadByte 0x01
 		STEP(11, i2c_restart(BUS))
 		STEP(12, i2c_write_addr(BUS, FAN_CONTROLLER_ADDRESS, I2C_WRITE))
		STEP(13, i2c_write_data(BUS, 0x01))
 		STEP(14, i2c_start(BUS))
 		STEP(15, i2c_write_addr(BUS, FAN_CONTROLLER_ADDRESS, I2C_READ))
 		STEP(16, i2c_check_ack(BUS, &ack))
 		STEP(17, i2c_receive(BUS))
		STEP(18, i2c_read_byte(BUS, NACK, &a))
		case 19:
			REG_MOTOR_TEMP_STATUS.right = (ack == ACK);
			REG_MOTOR_TEMP.right = a;
 			StepNumber++;
 			// fallthrough
 			
 	// REG_ROBOT_REL_SOC_A = Battery ReadWord 0x0d [="RelativeStateOfCharge"]
 		STEP(20, i2c_restart(BUS))
 		STEP(21, i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
 		STEP(22, i2c_write_data(BUS, 0x0d))	
 		STEP(23, i2c_start(BUS))
 		STEP(24, i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
 		STEP(25, i2c_receive(BUS))
 		STEP(26, i2c_read_byte(BUS, ACK, &a));
 		STEP(27, i2c_receive(BUS))
 		STEP(28, i2c_read_byte(BUS, NACK, &b));
 		case 29:
 			REG_ROBOT_REL_SOC_A = a + (b<<8);
 			StepNumber++;
 			// fallthrough
		
	// FanControl WriteByte 0x0b REG_MOTOR_SIDE_FAN_SPEED
		STEP(30, i2c_restart(BUS))
		STEP(31, i2c_write_addr(BUS, FAN_CONTROLLER_ADDRESS, I2C_WRITE))
		STEP(32, i2c_write_data(BUS, 0x0b))
		STEP(33, i2c_write_data(BUS, REG_MOTOR_SIDE_FAN_SPEED))
			
	// Battery ReadWord 0x16 [="BatteryStatus"]
		STEP(34, i2c_restart(BUS))
		STEP(35, i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
		STEP(36, i2c_write_data(BUS, 0x16))
		STEP(37, i2c_start(BUS))
		STEP(38, i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
		STEP(39, i2c_receive(BUS))
		STEP(40, i2c_read_byte(BUS, ACK, &a));
		STEP(41, i2c_check_ack(BUS, &ack));
		STEP(42, i2c_receive(BUS))
		STEP(43, i2c_read_byte(BUS, NACK, &b));
		case 44:
			if(ack == ACK) {
				unsigned battery_status = a + (b<<8);
				if ((battery_status & 0x000f) == 0) { // if battery status returns no error code
					BREAK_IF(battery_status & 0xff00); // 0xff00 are the battery alarm flags
					REG_BATTERY_A_HEALTH.status = battery_status;
				}
			}
			StepNumber++;
			// fallthrough
			
		// Battery ReadWord 0x03 [="BatteryMode"]
		STEP(45, i2c_restart(BUS))
		STEP(46, i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_WRITE))
		STEP(47, i2c_write_data(BUS, 0x03))
		STEP(48, i2c_start(BUS))
		STEP(49, i2c_write_addr(BUS, BATTERY_ADDRESS, I2C_READ))
		STEP(50, i2c_receive(BUS))
		STEP(51, i2c_read_byte(BUS, ACK, &a))
		STEP(52, i2c_check_ack(BUS, &ack));
		STEP(53, i2c_receive(BUS))
		STEP(54, i2c_read_byte(BUS, NACK, &b))
		case 55:
			if(ack == ACK) {
				REG_BATTERY_A_HEALTH.mode = a + (b<<8);
			}
			StepNumber++;
			// fallthrough
			
		STEP(56, i2c_stop(BUS))
		case 57:
 			I2C2TimerExpired=False;//reset the I2C2 update timer
 			StepNumber++;
			break;
			
		default: // Execution should never get to this case.
		    BREAKPOINT;
    		break;
 	}
#undef BUS
}




void I2C3Update(void)
{

 	static int StepNumber=1; 
	static unsigned int a,b;
 	//static int TMPHigh,TMPLow;

 	if(I2C3XmitReset==True)
 	{
 		I2C3XmitReset=False;//clear the flag
 		StepNumber=1;//go to step 1, start from the beginning
 	}
 	switch(StepNumber)
 	{
	 // Battery ReadWord 0x0d [="RelativeStateOfCharge"]
	 	case 1://make sure the module is idle
 			if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C3CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 		 	break;
 		case 2://wait until start is complete, then transmit data
 			if(I2C3CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C3TRN=BATTERY_ADDRESS<<1;//transmit data
 			}
 			break;
 		case 3://wait for transmit to complete
 			if(I2C3STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C3STATbits.IWCOL = 0;
 			 	//REG_MOTOR_TEMP_STATUS.left=0;
				//StepNumber = 0;
 			}
 			if(I2C3STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 4://make sure the module is idle
 			if(CheckI2C3Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 			}
 			break;
 		case 5://check ACK from slave
 			if(I2C3STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
 			{
 				//REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			StepNumber++;
 			break;
 		case 6://send data
 			I2C3TRN=0x0d;//transmit data
 			StepNumber++;
 			break;
 		case 7://wait for transmit to complete
 			if(I2C3STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C3STATbits.IWCOL = 0;
				//StepNumber = 0;
 			 	//REG_MOTOR_TEMP_STATUS.left==0;
 			}
 			if(I2C3STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 8://make sure the module is idle
 			if(CheckI2C3Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 				I2C3CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 			break;
		case 9:
			 if(I2C3CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C3TRN=(BATTERY_ADDRESS<<1)+1;//transmit data
 			}
 			break;
 		case 10:
 			if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C3CONbits.RCEN=1;//enable receive model
 			}
 			break;
 		case 11://wait until receive is completed
 			if(I2C3CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C3STATbits.I2COV = 0;
 				a=I2C3RCV;
				I2C3CONbits.ACKDT = 0; //ACK
				I2C3CONbits.ACKEN = 1;
 			}
 			break;
		case 12:
		 	if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C3CONbits.RCEN=1;//enable receive model
 			}
		break;
		case 13:
		 	if(I2C3CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C3STATbits.I2COV = 0;
 				b=I2C3RCV;
				REG_ROBOT_REL_SOC_B = a  + (b<<8);
				I2C3CONbits.ACKDT = 1; //NACK
				I2C3CONbits.ACKEN = 1;
 			}
		break;		
 		case 14://stop the module
		 	if(CheckI2C3Idle()==True)
 			{
	 			I2C3CONbits.PEN=1;	// initiate Stop on SDA and SCL pins
	 			StepNumber++;
			}
 			break;
 			
 	// REG_MOTOR_CHARGER_STATE = BatteryCharger ReadWord 0xca
		case 15:
 			if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 			}
		break;
	 	case 16://make sure the module is idle
 			if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C3CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 		 	break;
 		case 17://wait until start is complete, then transmit data
 			if(I2C3CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C3TRN=BATTERY_CHARGER_ADDRESS<<1;//transmit data
 			}
 			break;
 		case 18://wait for transmit to complete
 			if(I2C3STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C3STATbits.IWCOL = 0;
 			 	//REG_MOTOR_TEMP_STATUS.left=0;
				//StepNumber = 0;
 			}
 			if(I2C3STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 19://make sure the module is idle
 			if(CheckI2C3Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 			}
 			break;
 		case 20://check ACK from slave
 			if(I2C3STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
 			{
 				//REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			StepNumber++;
 			break;
 		case 21://send data
 			I2C3TRN=0xca;//transmit data
 			StepNumber++;
 			break;
 		case 22://wait for transmit to complete
 			if(I2C3STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C3STATbits.IWCOL = 0;
				//StepNumber = 0;
 			 	//REG_MOTOR_TEMP_STATUS.left==0;
 			}
 			if(I2C3STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 23://make sure the module is idle
 			if(CheckI2C3Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 				I2C3CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 			break;
		case 24:
			 if(I2C3CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C3TRN=(BATTERY_CHARGER_ADDRESS<<1)+1;//transmit data
 			}
 			break;
 		case 25:
 			if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C3CONbits.RCEN=1;//enable receive model
 			}
 			break;
 		case 26://wait until receive is completed
 			if(I2C3CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C3STATbits.I2COV = 0;
 				a=I2C3RCV;
				I2C3CONbits.ACKDT = 0; //ACK
				I2C3CONbits.ACKEN = 1;
 			}
 			break;
		case 27:
		 	if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C3CONbits.RCEN=1;//enable receive model
 			}
			break;
		case 28:
		 	if(I2C3CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C3STATbits.I2COV = 0;
 				b=I2C3RCV;
				REG_MOTOR_CHARGER_STATE = a  + (b<<8);
        		//REG_MOTOR_CHARGER_STATE = b;
				I2C3CONbits.ACKDT = 1; //NACK
				I2C3CONbits.ACKEN = 1;
 			}
 			break;
 			
 	// Battery ReadWord 0x16 [="BatteryStatus"]
 		case 29: //restart the module
		 	if(CheckI2C3Idle()==True)
 			{
	 			I2C3CONbits.RSEN = 1;
	 			StepNumber++;
			}
 			break;
		case 30:
			if(CheckI2C3Idle()) // Ready to write
			{
				I2C3TRN = BATTERY_ADDRESS<<1; // Read From Battery
				StepNumber++;
			}	
			break;			
		case 31:
			if(CheckI2C3Idle()) // Done writing
			{
				I2C3TRN = 0x16;
				BREAK_IF(I2C3STATbits.IWCOL);
				StepNumber++;
			}
			break;
		case 32:
			if(CheckI2C3Idle()) {
				 I2C3CONbits.SEN=1;
				 StepNumber++;
			} 
			break;
		case 33:
			if(CheckI2C3Idle()) {
				I2C3TRN = (BATTERY_ADDRESS<<1) + 1;
				BREAK_IF(I2C3STATbits.IWCOL);
				StepNumber++;
			}
			break;
		case 34:
			if(CheckI2C3Idle())
			{
				I2C3CONbits.RCEN = 1;
				StepNumber++;
			}
			break;
		case 35:
			if(CheckI2C3Idle()) {
				a = I2C3RCV;
				I2C3CONbits.ACKDT=0; // ACK
				I2C3CONbits.ACKEN=1; // send the ACK.
				StepNumber++;
			}	
			break;
		case 36:
			if(CheckI2C3Idle()) {
				I2C3CONbits.RCEN = 1;
				StepNumber++;
			}
			break;
		case 37:
			if(CheckI2C3Idle()) {
				b = I2C3RCV;
				unsigned battery_status = a + (b<<8);
				if ((battery_status & 0x000f) == 0) { // if battery status returns no error code
					BREAK_IF(battery_status & 0xff00); // 0xff00 are the battery alarm flags
					REG_BATTERY_B_HEALTH.status = battery_status;
				}	
				I2C3CONbits.ACKDT=1; // NACK
				I2C3CONbits.ACKEN=1;
				StepNumber++;
			}
			break;
			
		// Battery ReadWord 0x03 [="BatteryMode"]
		case 38:
			if(CheckI2C3Idle()){
				I2C3CONbits.RSEN = 1;
				StepNumber++;
			}
			break;
		case 39:
			if(CheckI2C3Idle()) // Ready to write
			{
				I2C3TRN = BATTERY_ADDRESS<<1;
				BREAK_IF(I2C3STATbits.IWCOL);
				StepNumber++;
			}	
			break;			
		case 40:
			if(CheckI2C3Idle())
			{
				I2C3TRN = 0x03;
				BREAK_IF(I2C3STATbits.IWCOL);
				StepNumber++;
			}
			break;
		case 41:
			if(CheckI2C3Idle()) {
				 I2C3CONbits.SEN=1;
				 StepNumber++;
			} 
			break;
		case 42:
			if(CheckI2C3Idle()) {
				I2C3TRN = (BATTERY_ADDRESS<<1) + 1;
				BREAK_IF(I2C3STATbits.IWCOL);
				StepNumber++;
			}
			break;
		case 43:
			if(CheckI2C3Idle())
			{
				I2C3CONbits.RCEN = 1;
				StepNumber++;
			}
			break;
		case 44:
			if(CheckI2C3Idle()) {
				a = I2C3RCV;
				I2C3CONbits.ACKDT=0; // ACK
				I2C3CONbits.ACKEN=1; // send the ACK.
				StepNumber++;
			}	
			break;
		case 45:
			if(CheckI2C3Idle()) {
				I2C3CONbits.RCEN = 1;
				StepNumber++;
			}
			break;
		case 46:
			if(CheckI2C3Idle()) {
				b = I2C3RCV;
				REG_BATTERY_B_HEALTH.mode = a + (b<<8);
				I2C3CONbits.ACKDT=1; // NACK
				I2C3CONbits.ACKEN=1;
				StepNumber++;
			}
			break;
		
		case 47:
 			if(CheckI2C3Idle())
 			{	
	 			I2C3CONbits.PEN = 1;
 				StepNumber = 1;
 				I2C3TimerExpired = False; //reset the update timer
 			}
			break;
	    default: // Execution should never get to this case.
	    	BREAKPOINT; // Oops. Cases are not contiguous!
	    	break;
	} 

}

//*********************************************//
//I2C library
int CheckI2C2Idle()
{

 	if(I2C2CONbits.SEN||I2C2CONbits.RSEN||I2C2CONbits.PEN||I2C2CONbits.RCEN||I2C2CONbits.ACKEN||I2C2STATbits.TRSTAT)
 	{
 		return False;
 	}else
 	{
 		return True;
 	}
}

int CheckI2C3Idle()
{

 	if(I2C3CONbits.SEN||I2C3CONbits.RSEN||I2C3CONbits.PEN||I2C3CONbits.RCEN||I2C3CONbits.ACKEN||I2C3STATbits.TRSTAT)
 	{
 		return False;
 	}else
 	{
 		return True;
 	}
}

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
