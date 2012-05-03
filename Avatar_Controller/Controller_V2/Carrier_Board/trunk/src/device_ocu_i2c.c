#include "stdhdr.h"
#include "device_ocu_i2c.h"

unsigned char ocu_i2c1_i2c_read = 0;
unsigned char ocu_i2c1_interrupt_state = 0;
unsigned char ocu_i2c1_slave_address = 0;

unsigned char ocu_i2c1_rx_byte1 = 0;
unsigned char ocu_i2c1_rx_byte2 = 0;
unsigned char ocu_i2c1_rx_byte3 = 0;
unsigned char ocu_i2c1_rx_byte4 = 0;
unsigned char ocu_i2c1_rx_byte5 = 0;

unsigned char ocu_i2c1_tx_byte1 = 0;
unsigned char ocu_i2c1_tx_byte2 = 0;

unsigned char ocu_i2c1_register  = 0;
unsigned char ocu_i2c1_message_length = 0;
unsigned char ocu_i2c1_transmit_word_completed = 0;
unsigned char ocu_i2c1_receive_word_completed = 0;



unsigned char ocu_batt_i2c_slave_address = 0;
unsigned char ocu_batt_i2c_rx_byte1 = 0;
unsigned char ocu_batt_i2c_rx_byte2 = 0;
unsigned char ocu_batt_i2c_rx_byte3 = 0;
unsigned char ocu_batt_i2c_rx_byte4 = 0;
unsigned char ocu_batt_i2c_rx_byte5 = 0;
unsigned char ocu_batt_i2c_read = 0;
unsigned char ocu_batt_i2c_register = 0;
unsigned char ocu_batt_receive_word_completed = 0;
unsigned char ocu_batt_transmit_word_completed  = 0;
unsigned char OCU_Batt_I2C2_state = 0x0a;
unsigned char ocu_batt_i2c_message_length = 0;
unsigned char ocu_batt_i2c_tx_byte1 = 0;
unsigned char ocu_batt_i2c_tx_byte2 = 0;
unsigned char ocu_batt_i2c_tx_byte3 = 0;
unsigned char ocu_batt_i2c_tx_byte4 = 0;


unsigned char fan_temp_1 = 0;
unsigned char fan_temp_2 = 0;
unsigned char fan_duty_cycle = 0;
unsigned char fan_speed = 0;

unsigned int battery_status = 0;
unsigned int battery_temperature = 0;

#define NUM_I2C1_MESSAGES   9

void init_i2c(void)
{
	I2C2CON = 0x1000;

	I2C2CONbits.I2CEN = 1;

	//FCY should be 16M
	//I2C2BRG = FCY/100000-FCY/10000000-1;	//should be 157.4 (between 9D and 9E)
	I2C2BRG = 0xff;

	//initialize I2C interrupts
	I2C2InterruptUserFunction=ocu_batt_smbus_isr;

	OCU_Batt_I2C2_state = 0x0a;
	IEC3bits.MI2C2IE = 0;


	I2C1InterruptUserFunction=ocu_i2c1_isr;
	
	I2C1CON = 0x1000;

	I2C1CONbits.I2CEN = 1;

	//FCY should be 16M
	//I2C2BRG = FCY/100000-FCY/10000000-1;	//should be 157.4 (between 9D and 9E)
	I2C1BRG = 0xff;

}

void ocu_batt_smbus_isr(void)
{

	IFS3bits.MI2C2IF = 0;
	//I2C2STATbits.BCL = 0;

	//test_flag++;
	if(ocu_batt_i2c_read)
	{
		
		switch(OCU_Batt_I2C2_state)
		{
			//start condition has finished, now we send the slave address (with read bit set to 0)
			case 0x00:

					I2C2TRN = (ocu_batt_i2c_slave_address<<1);

					OCU_Batt_I2C2_state++;
	
			break;
	
			//slave address written, now we send register address
			case 0x01:

				I2C2TRN = ocu_batt_i2c_register;
				OCU_Batt_I2C2_state++;

	
			break;
	
			//after the slave register has been sent, we send another start condition
			case 0x02:
	
				I2C2CONbits.RSEN = 1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//start condition completed; we send the slave address again, this time with the read bit set
			case 0x03:
	
				I2C2TRN = ((ocu_batt_i2c_slave_address<<1)+1);
				OCU_Batt_I2C2_state++;

	
			break;
	
			//start pusing clock pulses to the slave
			case 0x04:
	
				I2C2CONbits.RCEN = 1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x05:
	
				ocu_batt_i2c_rx_byte1 = I2C2RCV;


				if(ocu_batt_i2c_message_length == 1)
				{

					//set to NACK
					I2C2CONbits.ACKDT = 1;
					//send NACK
					I2C2CONbits.ACKEN = 1;
					OCU_Batt_I2C2_state= 0x0e;

				}

				else
				{
					//set to ACK
					I2C2CONbits.ACKDT = 0;
					//send ACK
					I2C2CONbits.ACKEN = 1;
					OCU_Batt_I2C2_state++;


				}
	
			break;

			case 0x06:
	
				I2C2CONbits.RCEN = 1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x07:
	
				ocu_batt_i2c_rx_byte2 = I2C2RCV;

				if(ocu_batt_i2c_message_length == 2)
				{

					//set to NACK
					I2C2CONbits.ACKDT = 1;
					//send NACK
					I2C2CONbits.ACKEN = 1;
					OCU_Batt_I2C2_state= 0x0e;

				}
				if(ocu_batt_i2c_message_length == 4)
				{
					//set to ACK
					I2C2CONbits.ACKDT = 0;
					//send ACK
					I2C2CONbits.ACKEN = 1;
					OCU_Batt_I2C2_state++;
				}
				
			break;
			case 0x08:
	
				I2C2CONbits.RCEN = 1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x09:
	
				ocu_batt_i2c_rx_byte3 = I2C2RCV;
				//set to ACK
				I2C2CONbits.ACKDT = 0;
				//send ACK
				I2C2CONbits.ACKEN = 1;
				OCU_Batt_I2C2_state++;

	
			break;
			case 0x0a:
	
				I2C2CONbits.RCEN = 1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x0b:
	
				ocu_batt_i2c_rx_byte4 = I2C2RCV;
				//set to ACK
				I2C2CONbits.ACKDT = 0;
				//send ACK
				I2C2CONbits.ACKEN = 1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//start pushing clock pulses to the slave
			case 0x0c:
	
				I2C2CONbits.RCEN = 1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x0d:
	
				ocu_batt_i2c_rx_byte5 = I2C2RCV;
				//set to NACK
				I2C2CONbits.ACKDT = 1;
				//send NACK
				I2C2CONbits.ACKEN = 1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//NACK has finished.  Send stop condition.
			case 0x0e:
	
				I2C2CONbits.PEN = 1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//stop has finished.  Alert (though module variable) that a new word is ready.
			case (0x0f):
	
					ocu_batt_receive_word_completed = 1;
			
	
			break; 
	
			//gets here if message wasn't acked properly, and we sent a stop condition
			case 0x10:
	
			break;
	
	
	
		}
	}
	else
	{
		switch(OCU_Batt_I2C2_state)
		{
			//start condition has finished, now we send the slave address (with read bit set to 0)
			case 0x00:
	
					I2C2TRN = (ocu_batt_i2c_slave_address<<1);
					OCU_Batt_I2C2_state++;
	
			break;
	
			//slave address written, now we send register address
			case 0x01:
	
				I2C2TRN = ocu_batt_i2c_register;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//after the slave register has been sent, we send another start condition
			case 0x02:
	
				I2C2TRN = ocu_batt_i2c_tx_byte1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//start condition completed; we send the slave address again, this time with the read bit set
			case 0x03:
	
				I2C2TRN = ocu_batt_i2c_tx_byte2;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//Send stop condition.
			case 0x04:
	
				I2C2CONbits.PEN = 1;
				OCU_Batt_I2C2_state++;
	
			break;
	
			//stop has finished.  Alert (though module variable) that a new word is ready.
			case 0x05:
	
					//need more general descriptor
					ocu_batt_receive_word_completed = 1;
	
			break; 
	
			//gets here if message wasn't acked properly, and we sent a stop condition
			case 0x06:
	
			break;
	
	
	
		}
	}

}



void ocu_batt_i2c_fsm(void)
{


	static unsigned char i2c_interrupt_state = 0x00;
	static unsigned char last_i2c_interrupt_state = 0;
	static unsigned char ocu_batt_i2c_busy = 0;
	static unsigned int timeout_counter = 0;
  unsigned char dummy = 0;

	last_i2c_interrupt_state = i2c_interrupt_state;



	//If the interrupt FSM has finished, load the data into the registers
	if(ocu_batt_receive_word_completed == 1)
	{
		//testa[6]++;
		ocu_batt_i2c_busy = 0;

		ocu_batt_receive_word_completed = 0;

		switch(i2c_interrupt_state)
		{
			case 0x00:
//				i2c_loop_counter++;
 				REG_OCU_TOUCH_PEN = ocu_batt_i2c_rx_byte1;			
				REG_OCU_TOUCH_X = 	ocu_batt_i2c_rx_byte2+(ocu_batt_i2c_rx_byte3<<7);
				REG_OCU_TOUCH_Y = 	ocu_batt_i2c_rx_byte4+(ocu_batt_i2c_rx_byte5<<7);
				i2c_interrupt_state++;

			break;
			
			case 0x01:
				
				REG_OCU_TEMP_1 = ((ocu_batt_i2c_rx_byte1<<4)+(ocu_batt_i2c_rx_byte2>>4))*.0625;
				
				//i2c_interrupt_state++;
				//REG_OCU_BATTERY_VOLTAGE = ocu_batt_i2c_rx_byte1+(ocu_batt_i2c_rx_byte2<<8);
				//i2c_interrupt_state++;
				i2c_interrupt_state++;
				
			break;
			case 0x02:
				REG_OCU_TEMP_2 = ((ocu_batt_i2c_rx_byte1<<4)+(ocu_batt_i2c_rx_byte2>>4))*.0625;
			
				//i2c_interrupt_state++;
				//REG_OCU_BATTERY_VOLTAGE = ocu_batt_i2c_rx_byte1+(ocu_batt_i2c_rx_byte2<<8);
				//i2c_interrupt_state++;

			//	REG_OCU_BATTERY_TEMPERATURE = ((ocu_batt_i2c_rx_byte1<<4)+(ocu_batt_i2c_rx_byte2>>4))*.0625;
				i2c_interrupt_state++;
			break;
			case 0x03:
				REG_OCU_ACCEL_X = (ocu_batt_i2c_rx_byte2<<8)+(ocu_batt_i2c_rx_byte1);
				
				//REG_OCU_ACCEL_X = 11;
				i2c_interrupt_state++;
			break;
			case 0x04:
				REG_OCU_ACCEL_Y = (ocu_batt_i2c_rx_byte2<<8)+(ocu_batt_i2c_rx_byte1);
				//REG_OCU_ACCEL_Y = 22;
				i2c_interrupt_state++;
			break;
			case 0x05:
				REG_OCU_ACCEL_Z = (ocu_batt_i2c_rx_byte2<<8)+(ocu_batt_i2c_rx_byte1);
				//REG_OCU_ACCEL_Z = 33;
				i2c_interrupt_state++;
			break;
			case 0x06:
				REG_OCU_MAGNETIC_X = ocu_batt_i2c_rx_byte2+(ocu_batt_i2c_rx_byte1<<8);
				//REG_OCU_MAGNETIC_X = 33;
				i2c_interrupt_state++;
			break;
			case 0x07:
				REG_OCU_MAGNETIC_Y = ocu_batt_i2c_rx_byte2+(ocu_batt_i2c_rx_byte1<<8);
				//REG_OCU_MAGNETIC_Y = 44;
				i2c_interrupt_state++;
			break;
			case 0x08:
				REG_OCU_MAGNETIC_Z = ocu_batt_i2c_rx_byte2+(ocu_batt_i2c_rx_byte1<<8);
				//REG_OCU_MAGNETIC_Z = 55;
				i2c_interrupt_state++;
			break;
			case 0x09:
				fan_temp_1 = ocu_batt_i2c_rx_byte1;
				i2c_interrupt_state++;
			break;
			case 0x0a:
				fan_temp_2 = ocu_batt_i2c_rx_byte1;
				i2c_interrupt_state++;
			break;
			case 0x0b:
				fan_duty_cycle = ocu_batt_i2c_rx_byte1;
				i2c_interrupt_state++;
			break;
			case 0x0c:
				fan_speed = ocu_batt_i2c_rx_byte1;
				i2c_interrupt_state = 0;
			break;
			default:
				i2c_interrupt_state = 0x00;
			break;
	
		}
	}

	//If the interrupt FSM is not running and we've already read the i2c data from the last message, start it for the next message
	if((ocu_batt_i2c_busy == 0) && (ocu_batt_receive_word_completed == 0))
	{

		//make sure bus is idle.  If not, return.
		//if(I2C2STATbits.P)
		//	return;

		ocu_batt_i2c_busy = 1;
		switch(i2c_interrupt_state)
		{
			case 0x00:
				OCU_Batt_I2C2_state = 0x03;
				ocu_batt_i2c_message_length = 4;
				start_ocu_batt_i2c_read(TOUCH_CONTROLLER_I2C_ADD,0x00);
				/*OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 2;
				start_ocu_batt_i2c_read(SMBUS_ADD_TMP112,0x00);	*/
			break;

			case 0x01:
				//main_loop_counter = 0;
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 2;
				start_ocu_batt_i2c_read(SMBUS_ADD_TMP112,0x00);	
				//ocu_batt_i2c_message_length = 2;
				//start_ocu_batt_i2c_read(SMBUS_ADD_BQ2060A,0x09);			
			break;
			case 0x02:
				
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 2;
				start_ocu_batt_i2c_read(SMBUS_ADD_TMP112_2,0x00);			
			break;
			case 0x03:
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 2;
				start_ocu_batt_i2c_read(I2C_ADXL345_ADD,0x32);

			break;
			case 0x04:
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 2;
				start_ocu_batt_i2c_read(I2C_ADXL345_ADD,0x34);
			break;
			case 0x05:
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 2;
				start_ocu_batt_i2c_read(I2C_ADXL345_ADD,0x36);
			break;
			case 0x06:
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 2;
				start_ocu_batt_i2c_read(I2C_ADD_HMC5843,0x03);
			break;
			case 0x07:
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 2;
				start_ocu_batt_i2c_read(I2C_ADD_HMC5843,0x05);
			break;
			case 0x08:
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 2;
				start_ocu_batt_i2c_read(I2C_ADD_HMC5843,0x07);
			break;
			case 0x09:
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 1;
				start_ocu_batt_i2c_read(I2C_ADD_FAN_CONTROLLER,0x19);
			break;
			case 0x0a:
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 1;
				start_ocu_batt_i2c_read(I2C_ADD_FAN_CONTROLLER,0x01);
			break;
			case 0x0b:
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 1;
				start_ocu_batt_i2c_read(I2C_ADD_FAN_CONTROLLER,0x18);
			break;
			case 0x0c:
				OCU_Batt_I2C2_state = 0x00;
				ocu_batt_i2c_message_length = 1;
				//start_ocu_batt_i2c_read(I2C_ADD_FAN_CONTROLLER,0x0d);
				start_ocu_batt_i2c_read(I2C_ADD_FAN_CONTROLLER,0x1c);
			break;

			default:
				i2c_interrupt_state = 0;
				ocu_batt_i2c_busy = 0;
			break;
				

		}
	}	

	if(i2c_interrupt_state == last_i2c_interrupt_state)
	{
		timeout_counter++;
		block_ms(1);
	}
	else
		timeout_counter = 0;

	if(timeout_counter > 10 )
	{
		block_ms(1);
	}

	else if(timeout_counter > 200)
	{


		//i2c_interrupt_state++;
		i2c_interrupt_state++;
		ocu_batt_i2c_rx_byte1 = 0xff;
		ocu_batt_i2c_rx_byte2 = 0xff;
		ocu_batt_i2c_rx_byte3 = 0xff;
		ocu_batt_i2c_rx_byte4 = 0xff;
		OCU_Batt_I2C2_state = 0x0f;
		
    I2C2CONbits.RCEN = 0;
    I2C2CONbits.ACKEN = 0;
    I2C2CONbits.PEN = 0;
    I2C2CONbits.RSEN = 0;
    I2C2CONbits.SEN = 0;
		I2C2STAT = 0;
    //read I2C2RCV to clear RBF
    dummy = I2C2RCV;
		block_ms(100);

		I2C2STAT = 0;
		ocu_batt_receive_word_completed = 0;
		ocu_batt_i2c_busy = 0;
		timeout_counter = 0;
		

	}



}

void start_ocu_batt_i2c_read(unsigned char add, unsigned char reg)
{
	ocu_batt_i2c_slave_address = add;
	ocu_batt_i2c_register = reg;
	ocu_batt_i2c_read = 1;
	


	I2C2CONbits.SEN = 1;

}

void start_ocu_batt_i2c_write(unsigned char add, unsigned char reg, unsigned int data)
{
	ocu_batt_i2c_slave_address = add;
	ocu_batt_i2c_register = reg;

	OCU_Batt_I2C2_state = 0x00;
	ocu_batt_i2c_tx_byte1 = (data & 0xff);
	ocu_batt_i2c_tx_byte2 = data>>8;


	ocu_batt_i2c_read = 0;
	
	I2C2CONbits.SEN = 1;

}

void initialize_i2c_devices(void)
{

	IEC3bits.MI2C2IE = 1;

	start_ocu_batt_i2c_write(I2C_ADD_HMC5843,0x02,0x00);
	block_ms(20);

	start_ocu_batt_i2c_write(I2C_ADXL345_ADD,0x2d,0x08);
	block_ms(20);

	start_ocu_batt_i2c_write(I2C_ADXL345_ADD,0x31,0x0b);
	block_ms(20);
	IEC3bits.MI2C2IE = 0;

}

void ocu_i2c1_isr(void)
{
//	testb[0]++;
	_MI2C1IF = 0;

	if(ocu_i2c1_i2c_read)
	{
		
		switch(ocu_i2c1_interrupt_state)
		{
			//start condition has finished, now we send the slave address (with read bit set to 0)
			case 0x00:

					I2C1TRN = (ocu_i2c1_slave_address<<1);
					ocu_i2c1_interrupt_state++;
	
			break;
	
			//slave address written, now we send register address
			case 0x01:

				I2C1TRN = ocu_i2c1_register;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//after the slave register has been sent, we send another start condition
			case 0x02:
	
				I2C1CONbits.RSEN = 1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//start condition completed; we send the slave address again, this time with the read bit set
			case 0x03:
	
				I2C1TRN = ((ocu_i2c1_slave_address<<1)+1);
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//start pusHing clock pulses to the slave
			case 0x04:
	
				I2C1CONbits.RCEN = 1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x05:
	
				ocu_i2c1_rx_byte1 = I2C1RCV;


				if(ocu_i2c1_message_length == 1)
				{

					//set to NACK
					I2C1CONbits.ACKDT = 1;
					//send NACK
					I2C1CONbits.ACKEN = 1;
					ocu_i2c1_interrupt_state= 0x0e;

				}

				else
				{
					//set to ACK
					I2C1CONbits.ACKDT = 0;
					//send ACK
					I2C1CONbits.ACKEN = 1;
					ocu_i2c1_interrupt_state++;

				}
	
			break;

			case 0x06:
	
				I2C1CONbits.RCEN = 1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x07:
	
				ocu_i2c1_rx_byte2 = I2C1RCV;

				if(ocu_i2c1_message_length == 2)
				{

					//set to NACK
					I2C1CONbits.ACKDT = 1;
					//send NACK
					I2C1CONbits.ACKEN = 1;
					ocu_i2c1_interrupt_state= 0x0e;

				}
				if(ocu_i2c1_message_length == 4)
				{
					//set to ACK
					I2C1CONbits.ACKDT = 0;
					//send ACK
					I2C1CONbits.ACKEN = 1;
					ocu_i2c1_interrupt_state++;
				}
				
			break;
			case 0x08:
	
				I2C1CONbits.RCEN = 1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x09:
	
				ocu_i2c1_rx_byte3 = I2C1RCV;
				//set to ACK
				I2C1CONbits.ACKDT = 0;
				//send ACK
				I2C1CONbits.ACKEN = 1;
				ocu_i2c1_interrupt_state++;

	
			break;
			case 0x0a:
	
				I2C1CONbits.RCEN = 1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x0b:
	
				ocu_i2c1_rx_byte4= I2C1RCV;
				//set to ACK
				I2C1CONbits.ACKDT = 0;
				//send ACK
				I2C1CONbits.ACKEN = 1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//start pushing clock pulses to the slave
			case 0x0c:
	
				I2C1CONbits.RCEN = 1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x0d:
	
				ocu_i2c1_rx_byte5 = I2C1RCV;
				//set to NACK
				I2C1CONbits.ACKDT = 1;
				//send NACK
				I2C1CONbits.ACKEN = 1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//NACK has finished.  Send stop condition.
			case 0x0e:
	
				I2C1CONbits.PEN = 1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//stop has finished.  Alert (though module variable) that a new word is ready.
			case (0x0f):
	
					ocu_i2c1_receive_word_completed = 1;
//					testb[1]++;
	
			break; 
	
			//gets here if message wasn't acked properly, and we sent a stop condition
			case 0x10:
	
			break;
	
	
	
		}
	}
	else
	{
		switch(ocu_i2c1_interrupt_state)
		{
			//start condition has finished, now we send the slave address (with read bit set to 0)
			case 0x00:
	
					I2C1TRN = (ocu_i2c1_slave_address<<1);
					ocu_i2c1_interrupt_state++;
	
			break;
	
			//slave address written, now we send register address
			case 0x01:
	
				I2C1TRN = ocu_i2c1_register;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//after the slave register has been sent, we send another start condition
			case 0x02:
	
				I2C1TRN = ocu_i2c1_tx_byte1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//start condition completed; we send the slave address again, this time with the read bit set
			case 0x03:
	
				I2C1TRN = ocu_i2c1_tx_byte2;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//Send stop condition.
			case 0x04:
	
				I2C1CONbits.PEN = 1;
				ocu_i2c1_interrupt_state++;
	
			break;
	
			//stop has finished.  Alert (though module variable) that a new word is ready.
			case 0x05:
	
					//need more general descriptor
					ocu_i2c1_transmit_word_completed = 1;
	
			break; 
	
			//gets here if message wasn't acked properly, and we sent a stop condition
			case 0x06:
	
			break;
	
	
	
		}
	}

}

void start_ocu_i2c1_i2c_read(unsigned char add, unsigned char reg)
{
	ocu_i2c1_slave_address = add;
	ocu_i2c1_register = reg;
	ocu_i2c1_i2c_read = 1;
	


	I2C1CONbits.SEN = 1;

}

void start_ocu_i2c1_write(unsigned char add, unsigned char reg, unsigned int data)
{
	ocu_i2c1_slave_address = add;
	ocu_i2c1_register = reg;

	ocu_i2c1_interrupt_state = 0x00;
	ocu_i2c1_tx_byte1 = (data & 0xff);
	ocu_i2c1_tx_byte2 = data>>8;


	ocu_i2c1_i2c_read = 0;
	
	I2C1CONbits.SEN = 1;

}


void ocu_i2c1_fsm(void)
{


	static unsigned char i2c1_device_state = 0x00;
	static unsigned char last_i2c1_device_state = 0;
	static unsigned char ocu_i2c1_busy = 0;
	static unsigned int timeout_counter = 0;
  static unsigned char i2c_message_failures[NUM_I2C1_MESSAGES] = {0,0,0,0,0,0,0,0,0};
  unsigned char dummy = 0;
  unsigned char i;

	last_i2c1_device_state = i2c1_device_state;

/*	testb[3] = i2c1_device_state;
	testb[4] = timeout_counter;
	testb[5]++;*/
	

	//If the interrupt FSM has finished, load the data into the registers
	if(ocu_i2c1_receive_word_completed == 1)
	{
    //clear timeout counter since we've receive a valid message
    i2c_message_failures[i2c1_device_state] = 0;

		ocu_i2c1_busy = 0;

		ocu_i2c1_receive_word_completed = 0;

		switch(i2c1_device_state)
		{
			case 0x00:
//				i2c_loop_counter++;
 				REG_OCU_BATT_VOLTAGE = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;

			break;
			
			case 0x01:				
				
				REG_OCU_REL_SOC_R = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;
				
			break;

			case 0x02:				
				
				REG_OCU_BATT_ABS_SOC = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;
				
			break;
			case 0x03:				
				
				REG_OCU_BATT_REM_CAP = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;
				
			break;
			case 0x04:				
				
				battery_status = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;
				
			break;

			case 0x05:
				right_battery_current = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;
			break;

			case 0x06:
				battery_temperature = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;
			break;

			case 0x07:
				REG_OCU_REL_SOC_L = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;
			break;

			case 0x08:
				left_battery_current = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state=0;
			break;

			default:
				i2c1_device_state = 0x00;
			break;
	
		}
	}

	//If the interrupt FSM is not running and we've already read the i2c data from the last message, start it for the next message
	if((ocu_i2c1_busy == 0) && (ocu_i2c1_receive_word_completed == 0))
	{



		ocu_i2c1_busy = 1;
		switch(i2c1_device_state)
		{
			case 0x00:
				I2C_MUX_CH(0);
				ocu_i2c1_interrupt_state = 0x00;
				ocu_i2c1_message_length = 2;
				start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x09);
				//start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x0d);
			break;
			case 0x01:
				ocu_i2c1_interrupt_state = 0x00;
				ocu_i2c1_message_length = 2;
				start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x0d);
			break;
			case 0x02:
				ocu_i2c1_interrupt_state = 0x00;
				ocu_i2c1_message_length = 2;
				start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x0e);
			break;
			case 0x03:
				ocu_i2c1_interrupt_state = 0x00;
				ocu_i2c1_message_length = 2;
				start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x0f);
			break;
			case 0x04:
				
				ocu_i2c1_interrupt_state = 0x00;
				ocu_i2c1_message_length = 2;
				start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x16);
			break;
			case 0x05:
				
				ocu_i2c1_interrupt_state = 0x00;
				ocu_i2c1_message_length = 2;
				start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x0a);
			break;
			case 0x06:
				
				ocu_i2c1_interrupt_state = 0x00;
				ocu_i2c1_message_length = 2;
				start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x08);
			break;
			case 0x07:
				I2C_MUX_CH(1);
				ocu_i2c1_interrupt_state = 0x00;
				ocu_i2c1_message_length = 2;
				start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x0d);
			break;
			case 0x08:
				ocu_i2c1_interrupt_state = 0x00;
				ocu_i2c1_message_length = 2;
				start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x0a);
			break;
			default:
				i2c1_device_state = 0;
				ocu_i2c1_busy = 0;
			break;
				

		}
	}	

	if(i2c1_device_state == last_i2c1_device_state)
	{
		timeout_counter++;
		block_ms(1);
	}
	else
		timeout_counter = 0;

	if(timeout_counter > 10 )
	{
		block_ms(1);
	}

	if(timeout_counter > 200)
	{
//		testb[9] = i2c1_device_state;
//		testb[4]++;
		//i2c1_device_state++;

		ocu_i2c1_rx_byte1 = 0xff;
		ocu_i2c1_rx_byte2 = 0xff;
		ocu_i2c1_rx_byte3 = 0xff;
		ocu_i2c1_rx_byte4= 0xff;
		ocu_i2c1_interrupt_state = 0x00;
		
		//I2C1CONbits.PEN = 1;

    I2C1CONbits.RCEN = 0;
    I2C1CONbits.ACKEN = 0;
    I2C1CONbits.PEN = 0;
    I2C1CONbits.RSEN = 0;
    I2C1CONbits.SEN = 0;
		I2C1STAT = 0;
    //read I2C1RCV to clear RBF
    dummy = I2C1RCV;


    i2c_message_failures[i2c1_device_state]++;

    
		block_ms(100);

		ocu_i2c1_receive_word_completed = 0;
		ocu_i2c1_busy = 0;
		timeout_counter = 0;
    i2c1_device_state++;
    if(i2c1_device_state > NUM_I2C1_MESSAGES)
       i2c1_device_state = 0;
		

	}


  for(i=0;i<NUM_I2C1_MESSAGES;i++)
  {
    if(i2c_message_failures[i] > 5)
    {
      i2c_message_failures[i] = 0;

      switch(i)
      {
  			case 0x00:
   				REG_OCU_BATT_VOLTAGE = 0xffff;  
  			break;  			
  			case 0x01: 				
  				REG_OCU_REL_SOC_R = 0xffff;  				
  			break;  
  			case 0x02: 				
  				REG_OCU_BATT_ABS_SOC = 0xffff;			
  			break;
  			case 0x03:  				
  				REG_OCU_BATT_REM_CAP = 0xffff; 				
  			break;
  			case 0x04:
          battery_status = 0xffff;
        break;  
  			case 0x05:
  				right_battery_current = 0xffff;
  			break;  
  			case 0x06:
  				battery_temperature = 0xffff;
  			break;  
  			case 0x07:
  				REG_OCU_REL_SOC_L = 0xffff;
  			break;  
  			case 0x08:
  				left_battery_current = 0xffff;
  			break;  
  			default:
  			break;
  	
  		}

      }
      
    }

}
