/**
 * @file device_battery.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex battery PIC firmware.
 *
 */

#include "stdhdr.h"
#include "device_battery.h"
#include "i2c.h"

#define SMBUS_ADD_BQ24745 0x09
#define I2C_RATE_SETTING 0xff
//#define I2C_RATE_SETTING 0x9e
//#define I2C_RATE_SETTING 0x50

#define SMBUS_ADD_TMP112 0x49
#define SMBUS_ADD_BQ2060A 0x0b
//#define SMBUS_ADD_BQ2060A 0x0a


#define COM_EXPRESS_ON(a)        _LATF1     = a
#define COM_EXPRESS_EN(a)        _TRISF1  = !a

#define COM_EXPRESS_PWR_OK(a)        _LATB2     = a
#define COM_EXPRESS_PWR_OK_EN(a)        _TRISB2  = !a

void initSMBus(void);
void writeSMBusWord( unsigned char add, unsigned char reg, unsigned int word);
unsigned int readSMBusWord(unsigned char add, unsigned char reg);
unsigned int return_temperature_battery(unsigned char add);
void configure_BQ24745(void);
void reset_BQ24745(void);
unsigned int readSMBusWord2(unsigned char add, unsigned char reg);
void reset_BQ2060A(void);
void read_all_BQ2060A_registers(void);

unsigned int BQ2060A_registers[64];
unsigned int get_good_data(unsigned char add,unsigned int reg);



unsigned char ocu_batt_i2c_slave_address = 0;
unsigned char ocu_batt_i2c_rx_byte1 = 0;
unsigned char ocu_batt_i2c_rx_byte2 = 0;

unsigned char ocu_batt_i2c_read = 0;

unsigned char ocu_batt_i2c_register = 0;
unsigned char ocu_batt_receive_word_completed = 0;
unsigned char ocu_batt_transmit_word_completed  = 0;
unsigned char OCU_Batt_I2C1_state = 0x0a;

unsigned char ocu_batt_i2c_tx_byte1 = 0;
unsigned char ocu_batt_i2c_tx_byte2 = 0;

void init_OCU_Batt_SMBUS(void);

//debug registers
unsigned int mfg_id = 0;
unsigned int dev_id = 0;
unsigned int chargecurrent = 0;
unsigned int chargevoltage = 0;
unsigned int inputcurrent=0;
unsigned char write_word_MSB = 0;
unsigned char write_word_LSB = 0;
unsigned int battery_voltage = 0;
unsigned int battery_current = 0;
unsigned int full_charge_capacity = 0;
unsigned int battery_status = 0;
unsigned int battery_absolute_state_of_charge = 0;
unsigned int battery_relative_state_of_charge = 0;
unsigned int battery_remaining_capacity = 0;
unsigned int pack_status_pack_configuration = 0;


unsigned int dummy = 0;
unsigned int bad_data_counter = 0;
unsigned int good_data_counter = 0;
unsigned int code_state_counter = 0;

void ocu_batt_smbus_isr(void);


#pragma code

void DeviceBatteryInit()
{
	_ADON = 0;
	AD1PCFGL = 0xffff;
	AD1PCFGH = 0xffff;

/*	// turn everything off
	SMBUS_ON(0);
	CODEC_PD_ON(0);
	RADIO_ON(0);
	COM_EXPRESS_PG_ON(0);
	COM_EXPRESS_ON(0);
	//CAMERA1_PWR_ON(0);
	//CAMERA2_PWR_ON(0);

	CLR_MAIN_SW_ON(0);

	ROTATE_UD_ON(0);
	ROTATE_RL_ON(0);
	LCD_POCB_ON(0);

	LCD_PWM1_ON(0);
	LCD_PWM2_ON(0);
	LCD_PWR_ON(0);

	LVDS_PDWN_ON(0);

	GPS_ON(0);

	

	// enable all outputs
	SMBUS_EN(1);
	CODEC_PD_EN(1);
	RADIO_EN(1);
	COM_EXPRESS_PG_EN(1);
	COM_EXPRESS_EN(1);
//	CAMERA1_PWR_EN(1);
//	CAMERA2_PWR_EN(1);

	CLR_MAIN_SW_EN(1);

	ROTATE_UD_EN(1);
	ROTATE_RL_EN(1);
	LCD_POCB_EN(1);

	LCD_PWM1_EN(1);
	LCD_PWM2_EN(1);
	LCD_PWR_EN(1);

	LVDS_PDWN_EN(1);

	GPS_EN(1);
	
	// start turning things on

	__delay_ms(40);
	SMBUS_ON(1);
	CODEC_PD_ON(1);
	RADIO_ON(1);
	GPS_ON(1);
	COM_EXPRESS_ON(1);
	LCD_PWR_EN(1); // we may want to delay this until the OS starts to boot
	// start LCD PWM 

	// tell the COMExpress it's good to go!
	__delay_ms(30);
	COM_EXPRESS_PG_ON(1);

*/
	//turn on 12V supply
//	TRISDbits.TRISD0 = 0;
//	LATDbits.LATD0 = 0;


	OCU_CHRGR_EN_ON(0);
	OCU_CHRGR_EN_EN(1);

	//__delay_ms(2000);

	OCU_V12_EN(1);
	OCU_V12_ON(1);

	OCU_V5_EN(1);
	OCU_V5_ON(1);

	TRISB = 0xffff;

//	OCU_COMEXPRESS_PGOOD_ON(0);
//	OCU_COMEXPRESS_ON(0);
//	OCU_LED_EN(1);
//	OCU_REAR_LED_EN(1);

//	OCU_COMEXPRESS_PGOOD_EN(1);
//	OCU_COMEXPRESS_EN(1);
	//OCU_CHARGE_EN(1);


	//TRISEbits.TRISE7 = 0;
	//LATEbits.LATE7 = 1;


//	COM_EXPRESS_EN(1);
//	COM_EXPRESS_PWR_OK_EN(1);


	__delay_ms(400);

//	OCU_COMEXPRESS_PGOOD_ON(0);
	__delay_ms(100);

	//while(1);
	

//	COM_EXPRESS_PWR_OK(1);
//	COM_EXPRESS_EN(a);
	
	//OCU_CHARGE_ON(0);
//	while(1);
//	initSMBus();
//	init_OCU_Batt_SMBUS();
	init_OCU_Batt_SMBUS();

	__delay_ms(100);

	
	
	

}

void DeviceBatteryProcessIO()
{
	code_state_counter = 1;
	//__delay_ms(5000);
//	configure_BQ24745();
	//OCU_CHARGE_ON(1);
	//reset_BQ24745();


/*		OCU_LED_ON(1);
		OCU_REAR_LED_ON(1);
		
		block_ms(1000);
		OCU_LED_ON(0);
		OCU_REAR_LED_ON(0);*/

		/*block_ms(2000);
		OCU_CHRGR_EN_ON(1);*/
		//block_ms(10000);

//	writeSMBusWord(SMBUS_ADD_BQ2060A,0x00,0x0606);
//	while(1);

//	reset_BQ2060A();
//	block_ms(2000);	
//	read_all_BQ2060A_registers();

//	StopI2C1();
//	while(1);
//	IdleI2C1();
//	while(1);
	code_state_counter = 2;
/*	while(1)
	{
		code_state_counter++;
		battery_voltage = readSMBusWord(SMBUS_ADD_BQ2060A,0x09);
		//battery_voltage = get_good_data(SMBUS_ADD_BQ2060A,0x09);
		block_ms(50);
		battery_current = readSMBusWord(SMBUS_ADD_BQ2060A,0x0a);
		//battery_current = get_good_data(SMBUS_ADD_BQ2060A,0x0a);
		block_ms(50);
		pack_status_pack_configuration = readSMBusWord(SMBUS_ADD_BQ2060A,0x2f);
		block_ms(50);
		//full_charge_capacity = readSMBusWord2(SMBUS_ADD_BQ2060A,0x10);
		//block_ms(50);
		battery_status = readSMBusWord(SMBUS_ADD_BQ2060A,0x16);
		block_ms(50);
		battery_absolute_state_of_charge = readSMBusWord(SMBUS_ADD_BQ2060A,0x0e);
		block_ms(50);
		battery_relative_state_of_charge = readSMBusWord(SMBUS_ADD_BQ2060A,0x0d);
		block_ms(50);
		battery_remaining_capacity = readSMBusWord(SMBUS_ADD_BQ2060A,0x0f);
		block_ms(50);

	}*/

	

/*	while(1)
	{
		block_ms(1000);
		configure_BQ24745();
		block_ms(1000);
		chargecurrent = readSMBusWord(SMBUS_ADD_BQ24745 ,0x14);
		chargevoltage = readSMBusWord(SMBUS_ADD_BQ24745 ,0x15);
		inputcurrent = readSMBusWord(SMBUS_ADD_BQ24745 ,0x3F);
		mfg_id = readSMBusWord(SMBUS_ADD_BQ24745 ,0xFE);
		dev_id = readSMBusWord(SMBUS_ADD_BQ24745 ,0xFF); 
	}*/
		//input_current = return_temperature_battery(SMBUS_ADD_TMP112);
	//	while(1)
	//	{

		//OCU_CHRGR_EN_ON(0);

	//	}
	ocu_batt_i2c_read = 0;
	ocu_batt_i2c_slave_address = SMBUS_ADD_BQ24745;
	ocu_batt_i2c_register = 0x15;
	ocu_batt_i2c_tx_byte1 = 0xa0;
	ocu_batt_i2c_tx_byte2 = 0x40;
//	ocu_batt_i2c_tx_byte1 = 0xa0;
//	ocu_batt_i2c_tx_byte2 = 0x40;
	OCU_Batt_I2C1_state = 0x00;
	I2C1CONbits.SEN = 1;
	block_ms(500);

	while(1)
	{
		
	
	//	if(ocu_batt_receive_word_completed)
	//	{
			ocu_batt_i2c_read = 1;
			//ocu_batt_i2c_register = 0x09;
			//ocu_batt_i2c_slave_address = SMBUS_ADD_BQ2060A;
			//battery_voltage = ocu_batt_i2c_rx_byte1 + (ocu_batt_i2c_rx_byte2<<8);
			ocu_batt_i2c_register = 0x15;
			ocu_batt_i2c_slave_address = SMBUS_ADD_BQ24745;
			chargevoltage = ocu_batt_i2c_rx_byte1 + (ocu_batt_i2c_rx_byte2<<8);
			OCU_Batt_I2C1_state = 0x00;
			I2C1CONbits.SEN = 1;
			ocu_batt_receive_word_completed = 0;
		
	//	}
	block_ms(500);


	}


}

unsigned int get_good_data(unsigned char add,unsigned int reg)
{
	dummy = 0xffff;
	while(dummy == 0xffff)
	{
		dummy = readSMBusWord2(add,reg);
		block_ms(10);
		bad_data_counter++;
	}
	bad_data_counter -= 1;
	good_data_counter++;
	return dummy;

}

void configure_BQ24745(void)
{

	//ChargeVoltage() = 16.8V (0x41A0 mV)
//	writeSMBusWord(SMBUS_ADD_BQ24745,0x15,0x41A0);
	//in case I have my bytes switched
//	writeSMBusWord(SMBUS_ADD_BQ24745,0x15,0x4141);
	//0x1010 = 4.112V
	writeSMBusWord(SMBUS_ADD_BQ24745,0x15,0x41A0);
	__delay_ms(100);
	//ChargeCurrent() = 2.048A (0x800 mA)
	writeSMBusWord(SMBUS_ADD_BQ24745,0x14,0x0800);
//	writeSMBusWord(SMBUS_ADD_BQ24745,0x14,0x0200);
	__delay_ms(100);
	//InputCurrent() = 4.096A (0x1000 mA)
//	writeSMBusWord(SMBUS_ADD_BQ24745,0x3F,0x1840);
	//0x0f80 = 3.968A
	writeSMBusWord(SMBUS_ADD_BQ24745,0x3F,0x0F80);	//this works
//	writeSMBusWord(SMBUS_ADD_BQ24745,0x3F,0x1F80);	//this sets input current to 0
	__delay_ms(100);


}

void init_OCU_Batt_SMBUS(void)
{

	I2C1CON = 0x1000;

	I2C1CONbits.I2CEN = 1;

	//FCY should be 16M
	I2C1BRG = FCY/100000-FCY/10000000-1;	//should be 157.4 (between 9D and 9E)

	//initialize I2C interrupts
	I2C1InterruptUserFunction=ocu_batt_smbus_isr;

	OCU_Batt_I2C1_state = 0x0a;
	IEC1bits.MI2C1IE = 1;

}

void read_all_BQ2060A_registers(void)
{
	unsigned char i;
	for(i=0;i<64;i++)
	{
		BQ2060A_registers[i] = 0;
	}


	while(1)
	{
		for(i=0;i<64;i++)
		{
			BQ2060A_registers[i] = readSMBusWord2(SMBUS_ADD_BQ2060A,i);
			block_ms(10);
		}

	}
	

}

void reset_BQ2060A(void)
{
	writeSMBusWord(SMBUS_ADD_BQ2060A,0x4f,0xff5a);
	block_ms(100);
	writeSMBusWord(SMBUS_ADD_BQ2060A,0x7d,0x0000);
	block_ms(100);
	writeSMBusWord(SMBUS_ADD_BQ2060A,0x7d,0x0080);
	block_ms(100);

}

void reset_BQ24745(void)
{

	writeSMBusWord(SMBUS_ADD_BQ24745,0x15,0x0000);
	__delay_ms(100);

	writeSMBusWord(SMBUS_ADD_BQ24745,0x14,0x0000);
	__delay_ms(100);

	writeSMBusWord(SMBUS_ADD_BQ24745,0x3F,0x0000);	//this works
}

void initSMBus(void) // Initialize the I2C interface to the realtime clock
{
	// Configure I2C for 7 bit address mode 100kHz

//	ODCDbits.ODD9 = 1; // SDA1 is set to open drain
//	ODCDbits.ODD10 = 1; // SCL1 is set to open drain

	OpenI2C1(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD  
                & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS 
				& I2C_NACK, I2C_RATE_SETTING);

	IdleI2C1();

	// wait a little before continuing...
	block_ms(100);
} //initI2C

void writeSMBusWord( unsigned char add, unsigned char reg, unsigned int word) // write an integer v to address add
{
	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1(add << 1);
	IdleI2C1();
	__delay_ms(5);
	// Write data byte
	MasterWriteI2C1( reg); 
	IdleI2C1();
	__delay_ms(5);
	write_word_LSB = word&0xff;
	write_word_MSB = (word >> 8)&0xff;

	//Write LSB of word
	MasterWriteI2C1(write_word_LSB);
	IdleI2C1();
	__delay_ms(5);


	//Write MSB of word
	MasterWriteI2C1(write_word_MSB);
	IdleI2C1();



	// Terminate command sequence with a stop condition
	StopI2C1();
	IdleI2C1();

} //writeI2C

unsigned int return_temperature_battery(unsigned char add)
{
	unsigned int temperature = 0;
	unsigned int a, b, c;
/*	writeI2C(add, 0x00);
	a = readI2C(add);

	writeI2C(add, 0x01);
	b = readI2C(add);*/

	a = readSMBusWord(add,0x00);



	//c = (b >> 4) | (a << 4);
	
	temperature = (b >> 4) | (a << 4);

	return temperature;


}

void ocu_batt_smbus_isr(void)
{

	IFS1bits.MI2C1IF = 0;

	if(ocu_batt_i2c_read)
	{
		switch(OCU_Batt_I2C1_state)
		{
			//start condition has finished, now we send the slave address (with read bit set to 0)
			case 0x00:
	
					I2C1TRN = (ocu_batt_i2c_slave_address<<1);
					OCU_Batt_I2C1_state++;
	
			break;
	
			//slave address written, now we send register address
			case 0x01:
	
				I2C1TRN = ocu_batt_i2c_register;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//after the slave register has been sent, we send another start condition
			case 0x02:
	
				I2C1CONbits.RSEN = 1;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//start condition completed; we send the slave address again, this time with the read bit set
			case 0x03:
	
				I2C1TRN = ((ocu_batt_i2c_slave_address<<1)+1);
				OCU_Batt_I2C1_state++;
	
			break;
	
			//start pusing clock pulses to the slave
			case 0x04:
	
				I2C1CONbits.RCEN = 1;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x05:
	
				ocu_batt_i2c_rx_byte1 = I2C1RCV;
				//set to ACK
				I2C1CONbits.ACKDT = 0;
				//send ACK
				I2C1CONbits.ACKEN = 1;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//start pushing clock pulses to the slave
			case 0x06:
	
				I2C1CONbits.RCEN = 1;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//load received byte into module-level variable
			case 0x07:
	
				ocu_batt_i2c_rx_byte2 = I2C1RCV;
				//set to NACK
				I2C1CONbits.ACKDT = 1;
				//send NACK
				I2C1CONbits.ACKEN = 1;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//NACK has finished.  Send stop condition.
			case 0x08:
	
				I2C1CONbits.PEN = 1;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//stop has finished.  Alert (though module variable) that a new word is ready.
			case 0x09:
	
					ocu_batt_receive_word_completed = 1;
	
			break; 
	
			//gets here if message wasn't acked properly, and we sent a stop condition
			case 0x0a:
	
			break;
	
	
	
		}
	}
	else
	{
		switch(OCU_Batt_I2C1_state)
		{
			//start condition has finished, now we send the slave address (with read bit set to 0)
			case 0x00:
	
					I2C1TRN = (ocu_batt_i2c_slave_address<<1);
					OCU_Batt_I2C1_state++;
	
			break;
	
			//slave address written, now we send register address
			case 0x01:
	
				I2C1TRN = ocu_batt_i2c_register;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//after the slave register has been sent, we send another start condition
			case 0x02:
	
				I2C1TRN = ocu_batt_i2c_tx_byte1;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//start condition completed; we send the slave address again, this time with the read bit set
			case 0x03:
	
				I2C1TRN = ocu_batt_i2c_tx_byte2;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//Send stop condition.
			case 0x04:
	
				I2C1CONbits.PEN = 1;
				OCU_Batt_I2C1_state++;
	
			break;
	
			//stop has finished.  Alert (though module variable) that a new word is ready.
			case 0x05:
	
					ocu_batt_transmit_word_completed = 1;
	
			break; 
	
			//gets here if message wasn't acked properly, and we sent a stop condition
			case 0x06:
	
			break;
	
	
	
		}
	}

}

unsigned int readSMBusWord2(unsigned char add, unsigned char reg)
{
	unsigned char a,b = 0;
	unsigned int r;
	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1((add << 1));
	//__delay_ms(5);

	IdleI2C1();
//	block_ms(5);

	MasterWriteI2C1(reg);


	//__delay_ms(5);
	IdleI2C1();
//	block_ms(5);
	// Terminate command sequence with a stop condition
	StopI2C1();
	IdleI2C1();

//	__delay_ms(20);

//	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1((add << 1) | 0x01);
	IdleI2C1();
//	block_ms(5);
	__delay_us(100);
	IFS1bits.MI2C1IF = 0;

	a = (unsigned int)(MasterReadI2C1());

	AckI2C1();
	IdleI2C1();
//	block_ms(5);
	//__delay_ms(20);
	b = (unsigned int)(MasterReadI2C1());

	NotAckI2C1();
	
	r = a | (b<<8);


	IdleI2C1();
	StopI2C1(); 
	IdleI2C1();

	return r;
}

unsigned int readSMBusWord(unsigned char add, unsigned char reg)
{

	unsigned char a,b = 0;
	unsigned int r;
	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1((add << 1));
	//__delay_ms(5);
	IdleI2C1();

	MasterWriteI2C1(reg);
	//__delay_ms(5);
	IdleI2C1();

	// Terminate command sequence with a stop condition
	//StopI2C1();
	IdleI2C1();

	__delay_us(100);

//	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1((add << 1) | 0x01);

//	MasterWriteI2C1((0x0a << 1) | 0x01);
//	__delay_ms(5);
	IdleI2C1();
	__delay_us(100);
	IFS1bits.MI2C1IF = 0;

	a = (unsigned int)(MasterReadI2C1());

	AckI2C1();
	IdleI2C1();
	b = (unsigned int)(MasterReadI2C1());

	NotAckI2C1();
	
	r = a | (b<<8);


//	IdleI2C1();
	StopI2C1(); 
	IdleI2C1();

	return r;

}


