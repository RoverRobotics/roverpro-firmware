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
 			if(CheckI2C2Idle()==True)
 			{
 				StepNumber++;//move to nonexistant step
 				//StepNumber=1;//cycle ends, go to the first step
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
		case 15:
 			if(CheckI2C3Idle()==True)
 			{
 				StepNumber++;//run second step
 				//StepNumber=1;//cycle ends, go to the first step
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

unsigned char I2C2_read_block_nonblocking(unsigned char address, unsigned char reg, unsigned char length, unsigned char *output, unsigned char restart)
{
  static unsigned int step = 0;
  static unsigned int block_index = 0;
  unsigned int i;




  if(restart)
  {
    step = 0;
    block_index = 0;
    for(i=0;i<20;i++)
    {
      output[i] = 'e';
    }
    I2C2CON = 0x1000;
    I2C2STAT = 0;
    I2C2CONbits.PEN=1;
    return 0;
    
  }

  if(CheckI2C2Idle()==False)
  {
    return 0;
  }


  switch(step)
  {
	 	case 0://make sure the module is idle
 				step++;//run second step
 				I2C2CONbits.SEN=1;	// initiate Start on SDA and SCL pins
 		 	break;
 		case 1://wait until start is complete, then transmit data
 			if(I2C2CONbits.SEN==0)
 			{
 				step++;//run next step
 				I2C2TRN=address<<1;//transmit data
 			}
 			break;
 		case 2://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C2STATbits.IWCOL = 0;
				step = 0;
 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				step++;
 			}
 			break;
 		case 3://check ACK from slave
 			if(I2C2STATbits.ACKSTAT==1)//if no ACK, tell the data is bad
 			{

 			}
 			step++;
 			break;
 		case 4://send data
 			I2C2TRN=reg;//transmit data
 			step++;
 			break;
 		case 5://wait for transmit to complete
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C2STATbits.IWCOL = 0;

 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				step++;
        I2C2CONbits.SEN=1;
 			}
 			break;
		case 6:
			 if(I2C2CONbits.SEN==0)
 			{
 				step++;//run next step
 				I2C2TRN=(BATTERY_ADDRESS<<1)+1;//transmit data
 			}
 			break;
 		case 7:
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C2STATbits.IWCOL = 0;

 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				step++;
 				I2C2CONbits.RCEN=1;//enable receive model
 			}
    break;
    case 8:
 			if(I2C2CONbits.RCEN==0)
 			{
        output[block_index] = I2C2RCV;
        block_index++;

        //cycle between the next state and this one, if we're still receiving bytes
        if(block_index >= length)
          step+=2;
        else
          step++;

 				I2C2STATbits.I2COV = 0;
				I2C2CONbits.ACKDT = 0; //ACK
				I2C2CONbits.ACKEN = 1;
 			}
    break;
    case 9:
 			if(I2C2STATbits.IWCOL==1)//write collision occurs, go to the first step
 			{
				I2C2STATbits.IWCOL = 0;

 			}
 			if(I2C2STATbits.TRSTAT==0)// wait for the transmitting to complete
 			{
 				step--;
 				I2C2CONbits.RCEN=1;//enable receive model
 			}
    break;
    case 10:
      I2C2CONbits.PEN=1;
      step++;
    break;
    case 11:
      return 1;
    break;


 			


  }

  return 0;


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
