#include "p24FJ256GB106.h"
#include "stdhdr.h"
#include "device_robot_motor.h"
#include "debug_uart.h"
#include "device_robot_motor_i2c.h"

int CheckI2C2Idle();
int CheckI2C3Idle();

int I2C2XmitReset = 0;
int I2C3XmitReset = 0;

int I2C2TimerExpired = 0;
int I2C3TimerExpired = 0;

//fan controller, TMP112, battery, EEPROM
void I2C2Update(void)
{

 	static int StepNumber=1; 
	static unsigned char a,b;
 	//static int TMPHigh,TMPLow;

 	if(I2C2XmitReset==True)
 	{
 		I2C2XmitReset=False;//clear the flag
 		StepNumber=1;//go to step 1, start from the beginning
 		//assume all the data is good
 		REG_MOTOR_TEMP_STATUS.left=1;
 		REG_MOTOR_TEMP_STATUS.right=1;
 		REG_MOTOR_TEMP_STATUS.board=1;
 	}
 	switch(StepNumber)
 	{
 	 	case 1://make sure the module is idle
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 		 	break;
 		case 2://wait until start is complete, then transmit data
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=FANCtrlAddressW;//transmit data
 			}
 			break;
 		case 3://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C2STATbits.IWCOL=0;
 			 	REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 4://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 			}
 			break;
 		case 5://check ACK from slave
 			if(I2C2STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
 			{
 				REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			StepNumber++;
 			break;
 		case 6://send data
 			I2C2TRN=0x00;//transmit data
 			StepNumber++;
 			break;
 		case 7://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
 			 	REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 8://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 			break;
 		case 9:
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=FANCtrlAddressR;//transmit data
 			}
 			break;
 		case 10:
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.RCEN=1;//enable receive model
 			}
 			break;
 		case 11://wait until receive is completed
 			if(I2C2CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C2STATbits.I2COV = 0;
 				REG_MOTOR_TEMP.left=I2C2RCV;
 			}
 			break;
 		case 12://stop the module
 			I2C2CONbits.PEN=1;	// initiate Stop on SDA and SCL pins
 			StepNumber++;
 			break;
 	 	case 13://make sure the module is idle
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 		 	break;
 		case 14://wait until start is complete, then transmit data
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=FANCtrlAddressW;//transmit data
 			}
 			break;
 		case 15://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
 			 	REG_MOTOR_TEMP_STATUS.right=0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 16://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 			}
 			break;
 		case 17://check ACK from slave
 			if(I2C2STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
 			{
 				REG_MOTOR_TEMP_STATUS.right=0;
 			}
 			StepNumber++;
 			break;
 		case 18://send data
 			I2C2TRN=0x01;//transmit register address data
 			StepNumber++;
 			break;
 		case 19://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
 			 	REG_MOTOR_TEMP_STATUS.right=0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 20://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 			break;
 		case 21:
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=FANCtrlAddressR;//transmit data
 			}
 			break;
 		case 22:
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.RCEN=1;//enable receive model
 			}
 			break;
 		case 23://wait until receive is completed
 			if(I2C2CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C2STATbits.I2COV = 0;
 				REG_MOTOR_TEMP.right=I2C2RCV;
 			}
 			break;
 		case 24://stop the module
 			I2C2CONbits.PEN=1;	// initiate Stop on SDA and SCL pins
 			StepNumber++;
 			break;
	 	case 25://make sure the module is idle
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 		 	break;
 		case 26://wait until start is complete, then transmit data
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=BATTERY_ADDRESS<<1;//transmit data
 			}
 			break;
 		case 27://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
 			 	//REG_MOTOR_TEMP_STATUS.left=0;
				//StepNumber = 0;
				StepNumber++;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 28://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 			}
 			break;
 		case 29://check ACK from slave
 			if(I2C2STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
 			{
 				//REG_MOTOR_TEMP_STATUS.left=0;
 			}
 			StepNumber++;
 			break;
 		case 30://send data
 			I2C2TRN=0x0d;//transmit data
 			StepNumber++;
 			break;
 		case 31://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C2STATbits.IWCOL = 0;
 			 	//REG_MOTOR_TEMP_STATUS.left==0;
				StepNumber++;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				StepNumber++;
 			}
 			break;
 		case 32://make sure the module is idle
 			if(CheckI2C2Idle()==True)//make sure the module is idle
 			{
 				StepNumber++;//run next step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 			}
 			break;
 		case 33:
 			if(I2C2CONbits.SEN==0)
 			{
 				StepNumber++;//run next step
 				I2C2TRN=(BATTERY_ADDRESS<<1)+1;//transmit data
 			}
 			break;
 		case 34:
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.RCEN=1;//enable receive model
 			}
 			break;
 		case 35://wait until receive is completed
 			if(I2C2CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C2STATbits.I2COV = 0;
 				a=I2C2RCV;
				I2C2CONbits.ACKDT = 0; //ACK
				I2C2CONbits.ACKEN = 1;
 			}
 			break;
		case 36:
		 	if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				I2C2CONbits.RCEN=1;//enable receive model
 			}
		break;
		case 37:
		 	if(I2C2CONbits.RCEN==0)
 			{
 				StepNumber++;
 				I2C2STATbits.I2COV = 0;
 				b=I2C2RCV;
				REG_ROBOT_REL_SOC_A = a  + (b<<8);
				I2C2CONbits.ACKDT = 1; //NACK
				I2C2CONbits.ACKEN = 1;
 			}
		break;		
 		case 38://stop the module
			I2C2CONbits.ACKDT = 0; //ACK
 			I2C2CONbits.PEN=1;	// initiate Stop on SDA and SCL pins
 			StepNumber++;
 			break;
 		case 39://Make sure the module is idle
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				//StepNumber=1;//cycle ends, go to the first step
 				//I2C2TimerExpired=False;//reset the I2C2 update timer
 			}
 			break;

		case 40://make sure the module is idle
					if(CheckI2C2Idle()==True)
					{
						StepNumber++;//run second step
						I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
					}
				 	break;
		case 41://wait until start is complete, then transmit data
			if(I2C2CONbits.SEN==0)
			{
				StepNumber++;//run next step
				I2C2TRN=FANCtrlAddressW;//transmit data
			}
			break;
		case 42://wait for transmit to complete
			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
			{
			 	//REG_MOTOR_TEMP_STATUS.right=0;
			}
			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
			{
				StepNumber++;
			}
			break;
		case 43://make sure the module is idle
			if(CheckI2C2Idle()==True)//make sure the module is idle
			{
				StepNumber++;//run next step
			}
			break;
		case 44://check ACK from slave
			if(I2C2STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
			{
				//REG_MOTOR_TEMP_STATUS.right=0;
			}
			StepNumber++;
			break;
		case 45://wait until start is complete, then transmit data
			if(I2C2CONbits.SEN==0)
			{
				StepNumber++;//run next step
				I2C2TRN=0x0b;//transmit data
			}
			break;
		case 46://wait for transmit to complete
			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
			{
			 	//REG_MOTOR_TEMP_STATUS.right=0;
			}
			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
			{
				StepNumber++;
			}
			break;
		case 47://make sure the module is idle
			if(CheckI2C2Idle()==True)//make sure the module is idle
			{
				StepNumber++;//run next step
			}
			break;
		case 48://check ACK from slave
			if(I2C2STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
			{
				//REG_MOTOR_TEMP_STATUS.right=0;
			}
			StepNumber++;
			break;
		case 49://send data
			I2C2TRN=REG_MOTOR_SIDE_FAN_SPEED;//transmit register address data
			StepNumber++;
			break;
		case 50://wait for transmit to complete
			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
			{
			 	//REG_MOTOR_TEMP_STATUS.right=0;
			}
			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
			{
				StepNumber++;
			}
			break;
		case 51://make sure the module is idle
			if(CheckI2C2Idle()==True)//make sure the module is idle
			{
				StepNumber++;//run next step
				I2C2CONbits.PEN=1;	// initiate Stop on SDA and SCL pins
			}
		case 52://check ACK from slave
			if(I2C2STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
			{
				//REG_MOTOR_TEMP_STATUS.right=0;
			}
			StepNumber++;
			break;
		case 53:
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//run second step
 				StepNumber=1;//cycle ends, go to the first step
 				I2C2TimerExpired=False;//reset the I2C2 update timer
 			}
 			break;
 	}

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
 		//assume all the data is good

		I2C3CONbits.SEN = 0;
		I2C3CONbits.PEN = 0;
		I2C3CONbits.RCEN = 0;
		I2C3CONbits.ACKEN = 0;
		I2C3STATbits.TRSTAT = 0;

 	}
 	switch(StepNumber)
 	{
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
				StepNumber = 0;
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
				StepNumber = 0;
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
		case 15:
 			if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				StepNumber=1;//cycle ends, go to the first step
 				I2C3TimerExpired=False;//reset the I2C2 update timer
 			}
		break;

	} 

}

//*********************************************//
//I2C library
int CheckI2C2Idle()
{

 	if(I2C2CONbits.SEN||I2C2CONbits.PEN||I2C2CONbits.RCEN||I2C2CONbits.ACKEN||I2C2STATbits.TRSTAT)
 	{
 		return False;
 	}else
 	{
 		return True;
 	}
}

int CheckI2C3Idle()
{

 	if(I2C3CONbits.SEN||I2C3CONbits.PEN||I2C3CONbits.RCEN||I2C3CONbits.ACKEN||I2C3STATbits.TRSTAT)
 	{
 		return False;
 	}else
 	{
 		return True;
 	}
}
