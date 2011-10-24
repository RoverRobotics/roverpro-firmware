/**
 * @file device_ocu.c
 * @author T. Penn
 * @author Robotex, Inc.
 *
 * Robotex OCU PIC firmware.
 *
 */

#include "stdhdr.h"
#include "device_ocu.h"




//button inputs
#define VOLUME_DOWN()	(_RG6)
#define VOLUME_UP()		(_RG7)
#define TOGGLE1_UP() 	(_RB15)
#define TOGGLE1_DOWN()	(_RB8)
#define TOGGLE2_DOWN()	(_RB14)
#define TOGGLE2_UP()	(_RB9)
#define TALK_BUTTON()	(!_RB3)
#define MENU_BUTTON()	(!_RB2)

#define LIGHT_BUTTON()	(!_RB4)
#define POWER_BUTTON()	(!_RD0)


#define CHARGER_ACOK()	(_RB0)

//joystick inputs
#define JOY1_X_EN(a)	_PCFG11 = !a
#define JOY1_Y_EN(a)	_PCFG12 = !a
#define JOY2_X_EN(a)	_PCFG10 = !a
#define JOY2_Y_EN(a)	_PCFG13 = !a

//joystick AN pin numbers
#define JOY1_X_CH		11
#define JOY1_Y_CH		12
#define JOY2_X_CH		10
#define JOY2_Y_CH		13

//indicators of COM Express power state
#define SUS_S5()			_RC14
#define SUS_S3()			_RD6

//indicates that COM Express has overheated
#define NC_THERM_TRIP	_RD2

//outputs
#define CHARGER_EN(a)	_TRISD11 = !a
#define CHARGER_ON(a)	_LATD11 = a

//power button on COM Express
#define COMPUTER_PWR_EN(a)	_TRISB1 = !a
#define COMPUTER_PWR_ON(a)	_LATB1 = a

#define V3V3_EN(a)		_TRISE7 = !a
#define V3V3_ON(a)		_LATE7 = a

#define V5V_EN(a)		_TRISE5 = !a
#define V5V_ON(a)		_LATE5 = a

#define V12V_EN(a)		_TRISE6	= !a
#define V12V_ON(a)		_LATE6 = !a

#define COMPUTER_PWR_OK_EN(a)	_TRISD1 = !a
#define COMPUTER_PWR_OK(a)		_LATD1 = a

#define GREEN_LED_EN(a)		_TRISD8 = !a
#define GREEN_LED_ON(a)		_LATD8 = a

#define RED_LED_EN(a)		_TRISD3 = !a
#define RED_LED_ON(a)		_LATD3 = a

#define GPS_TX_OR			_RP27R
#define GPS_RX_PIN			19

#define CAMERA_PWR_EN(a)	_TRISB5 = !a
#define CAMERA_PWR_ON(a)	_LATB5 = a		

#define LCD_PWM_OR			_RP25R

/*#define LCD_BACK_ON(a)	TRISDbits.TRISD4 = !a
#define LCD_BACK_ON(a)	LATDbits.LATD4 = a*/


#define TOUCH_CONTROLLER_I2C_ADD 0x4d
#define SMBUS_ADD_TMP112_2 0x48
#define SMBUS_ADD_TMP112 0x49
#define SMBUS_ADD_BQ2060A 0x0b
#define SMBUS_ADD_BQ24745 0x09
#define I2C_ADD_FAN_CONTROLLER	0x18

#define I2C_ADD_HMC5843 0x1e
#define I2C_ADXL345_ADD           0x53


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

unsigned int battery_temperature = 0;

unsigned char ocu_i2c1_receive_word_completed = 0;

//void ocu_batt_smbus_isr(void);


void ocu_i2c1_isr(void);
void ocu_i2c1_fsm(void);
void start_ocu_i2c1_write(unsigned char add, unsigned char reg, unsigned int data);
void start_ocu_i2c1_i2c_read(unsigned char add, unsigned char reg);
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


unsigned int test_flag = 0;
unsigned char testb[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned char testa[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned int adc_int_flag = 0;


void start_ocu_batt_i2c_write(unsigned char add, unsigned char reg, unsigned int data);
void ocu_batt_i2c_fsm(void);
void ocu_batt_smbus_isr(void);

void handle_gas_gauge(void);

void start_ocu_batt_i2c_read(unsigned char add, unsigned char reg);

void init_io(void);
void disable_io(void);

//void test_power_on_sequence(void);
void update_button_states(void);
unsigned int return_adc_value(unsigned char ch);

unsigned char sus_s3_state = 0;
unsigned char sus_s5_state = 0;

//unsigned char is_3v3_up = 0;
unsigned char computer_on_flag = 0;

//void update_touchscreen_registers(void);


void handle_power_button(void);

void initialize_i2c_devices(void);

//unsigned char gps_data[60];
unsigned char gps_message[95];
void test_gps_uart(void);
void ocu_gps_isr(void);

void ocu_gps_init(void);

void init_fan_controller(void);

unsigned char fan_temp_1 = 0;
unsigned char fan_temp_2 = 0;
unsigned char fan_duty_cycle = 0;
unsigned char fan_speed = 0;




//Debugging register - remove eventually
unsigned int gps_interrupt_counter = 0;
unsigned int gps_message_state_debug = 0;
unsigned char turned_on_latch = 0;
unsigned int main_loop_counter = 0;
unsigned int i2c_loop_counter = 0;

unsigned int battery_voltage = 0;
unsigned int battery_relative_SOC = 0;
unsigned int battery_absolute_SOC = 0;
unsigned int battery_remaining_capacity = 10000;
unsigned int battery_status = 0;
int battery_current = 0;
unsigned int battery_full_charge_capacity = 0;
unsigned char battery_too_low = 0;
//unsigned char gps_message_temp[100];
// /debugging

unsigned int lowest_battery_voltage = 20000;
unsigned int last_battery_voltage = 0;

void usb_dummy(void);


unsigned char usb_bus_sense_debug;
 void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void);

void joystick_interrupt(void);

void handle_charging(void);

unsigned int bq2060a_registers[128];

void read_all_bq2060a_registers(void);


#pragma code

void DeviceOcuInit()
{


	unsigned int i;

	const unsigned char build_date[12] = __DATE__; 
	const unsigned char build_time[12] = __TIME__;

	for(i=0;i<12;i++)
	{
		REG_OCU_FIRMWARE_BUILD.data[i] = build_date[i];
		REG_OCU_FIRMWARE_BUILD.data[i+12] = build_time[i];
	}

/*	for(i=0;i<24;i++)
	REG_OCU_FIRMWARE_BUILD.data[i] = 0xaa;*/

	//Sleep();
/*	while(!POWER_BUTTON());
	Sleep();
	while(1);*/
	//When the PIC resets due to an error, the computer freaks out
	//if we don't wait a little bit (it boots up, but video outputs to some
	//as-yet unknown source
	//block_ms(5000);
//test_gps_uart();

	REG_OCU_ACCEL_X = 77;

	init_io();

	//Sleep();
	//initialize backlight PWM
	
	//set to OC1
	LCD_PWM_OR = 18;	
	
	T2CONbits.TCKPS = 0;	


	
	//TPS61161: between 5kHz and 100 kHz PWM dimming frequency
	//Choose 20kHz (50us period)
	//50us = [PR2 + 1]*62.5ns*1
	PR2 = 801;	
	OC1RS = 800;
	set_backlight_brightness(0);
	
	//Sleep();

	OC1CON2bits.SYNCSEL = 0x1f;	
	
	//use timer 2
	OC1CON1bits.OCTSEL2 = 0;
	
	//edge-aligned pwm mode
	OC1CON1bits.OCM = 6;
	
	//turn on timer 2
	T2CONbits.TON = 1;


	
	I2C2CON = 0x1000;

	I2C2CONbits.I2CEN = 1;

	//FCY should be 16M
	//I2C2BRG = FCY/100000-FCY/10000000-1;	//should be 157.4 (between 9D and 9E)
	I2C2BRG = 0xff;

//	Sleep();

	//initialize I2C interrupts
	I2C2InterruptUserFunction=ocu_batt_smbus_isr;
	
	U1RXInterruptUserFunction = ocu_gps_isr;

	ocu_gps_init();	

	OCU_Batt_I2C2_state = 0x0a;
	IEC3bits.MI2C2IE = 0;



	//ADC interrupt



//	ADC1InterruptUserFunction = joystick_interrupt;



	/*

	AD1CSSL = 0x3c00;
	_ASAM = 1;
	_SMPI = 0x03;
	_CSCNA = 1;	//enable scanning
	_AD1IE = 1;
	_ADON = 1;
	*/

	/*AD1CON1bits.ADON = 0;
	AD1CHS0bits.CH0SA = JOY2_Y_CH;
	AD1CON1bits.ADON = 1;
	AD1CON1bits.SAMP = 1;*/
	
	//V3V3_ON(1);
//	block_ms(1000);
//	V3V3_ON(0);


	//Sleep();


	I2C1InterruptUserFunction = ocu_i2c1_isr;
	
	I2C1CON = 0x1000;

	I2C1CONbits.I2CEN = 1;

	//FCY should be 16M
	//I2C2BRG = FCY/100000-FCY/10000000-1;	//should be 157.4 (between 9D and 9E)
	I2C1BRG = 0xff;


	set_backlight_brightness(50);

	//if charger is plugged in when the program starts, the batteries are likely
	//completely dead.  Try to charge to get them to come back.
	if(CHARGER_ACOK())
	{

		_MI2C1IE = 1;
		
		GREEN_LED_ON(0);
		RED_LED_ON(0);
		block_ms(5);
		start_ocu_i2c1_write(SMBUS_ADD_BQ24745,0x15,0x41a0);
		block_ms(20);
		start_ocu_i2c1_write(SMBUS_ADD_BQ24745,0x14,0x0400);
		block_ms(20);
		start_ocu_i2c1_write(SMBUS_ADD_BQ24745,0x3f,0x0f80);
		block_ms(20);

		CHARGER_ON(1);

		for(i=0;i<10;i++)
		{
			GREEN_LED_ON(1);
			block_ms(200);
			GREEN_LED_ON(0);
			block_ms(200);
		}
		CHARGER_ON(0);

		_MI2C1IE = 0;

	}
	

}

void DeviceOcuProcessIO()
{




//	read_all_bq2060a_registers();


	unsigned int dummy;
	//_ASAM = 1;
	main_loop_counter++;

	if( (computer_on_flag == 0) && (CHARGER_ACOK() == 0))
	{
		disable_io();


		//set change interrupt to power button and charger ACOK signal
		_CN49IE = 1;
		_CN2IE = 1;
		dummy = PORTD;
		dummy = PORTB;
		_CNIF = 0;	
		_CNIE = 1;	
		//when watchdog wakes up OCU, make sure that
		//we go right back to sleep, to save power
		while( (POWER_BUTTON() == 0) && (CHARGER_ACOK() == 0) )
		{
			Sleep();
			//reset interrupts so that we get woken up again
			dummy = PORTD;
			dummy = PORTB;
			_CNIF = 0;	
			_CNIE = 1;	
		}
		_CNIE = 0;
		_CNIF = 0;
		dummy = PORTD;
		dummy = PORTB;
		init_io();
		block_ms(50);
	}

	handle_power_button();
	update_button_states();
		

		IEC0bits.U1RXIE = 1;
		IEC3bits.MI2C2IE = 1;
		_MI2C1IE = 1;
		ocu_batt_i2c_fsm();
		ocu_i2c1_fsm();

	handle_charging();
	handle_gas_gauge();


	//computer has shut down.  turn off power supplies
	if( (SUS_S3()==0) && (SUS_S5() == 0) && (computer_on_flag == 1))
	{
					computer_on_flag = 0;
					GREEN_LED_ON(0);
					V3V3_ON(0);
					V5V_ON(0);
					V12V_ON(0);
					COMPUTER_PWR_OK(0);
	}

//	while(!POWER_BUTTON());
//	GREEN_LED_ON(1);


}

void handle_gas_gauge(void)
{

	static unsigned int low_voltage_counter = 0;

	//if the voltage increases and the charger is plugged in, reset low battery vars
	if(CHARGER_ACOK() && (REG_OCU_BATT_VOLTAGE > 14000))
	{
		battery_too_low = 0;

		low_voltage_counter = 0;
		last_battery_voltage = 0;
	}

	if(REG_OCU_BATT_VOLTAGE < 13950)
	{
		//count number of times we get a different voltage that is too low
		if(REG_OCU_BATT_VOLTAGE != last_battery_voltage)
		{
			last_battery_voltage = REG_OCU_BATT_VOLTAGE;
			low_voltage_counter++;

		}


		//if counter gets to 5 different readings, shut off
		if(low_voltage_counter >= 5)
		{
		//only shut off everything if the adapter isn't plugged in
			if(!CHARGER_ACOK())
			{
				battery_too_low = 1;
				computer_on_flag = 0;
				GREEN_LED_ON(0);
				RED_LED_ON(0);
				V3V3_ON(0);
				V5V_ON(0);
				V12V_ON(0);
				COMPUTER_PWR_OK(0);
			}
			low_voltage_counter = 200;
		}


	}

	//turn on red LED to indicate charging, but only if computer is off
	if(REG_OCU_BATT_CURRENT > 0)
	{
		if(computer_on_flag == 0)
			RED_LED_ON(1);
		
	}
	else
	{
		RED_LED_ON(0);
	}

}

void ocu_gps_isr(void)
{

	

	static char gps_message_state = 0;
	static unsigned char gps_message_temp[100];

	//filter for messages starting with $GPGLL
	//unsigned char message_filter[6] = {'$','G','P','G','L','L'};
	//unsigned char message_filter[6] = {'$','G','P','G','S','V'};
	unsigned char message_filter[6] = {'$','G','P','G','G','A'};
	unsigned char i;

	unsigned char new_byte;

	U1STAbits.OERR = 0;

	IFS0bits.U1RXIF = 0;

	new_byte = U1RXREG;
	gps_interrupt_counter++;
	

	if(gps_message_state == 0)
	{

		if(new_byte == '$')
		{
			//gps_message_temp[gps_message_state] = new_byte;
			gps_message_temp[gps_message_state] = new_byte;
			//gps_message_temp[0] = 0x26;
			gps_message_state++;
			

		}


	}
	else
	{
		gps_message_temp[gps_message_state] = new_byte;
		//gps_message_temp[gps_message_state] = gps_message_state;

		if(gps_message_state == 5)
		{
			for(i=0;i<5;i++)
			{
				if(gps_message_temp[i] != message_filter[i])
				{
					gps_message_state = 0;
					
				}
			}
		}

		if(gps_message_state >= 95)
		{
			for(i=0;i<95;i++)
			{
				//gps_message[i] = gps_message_temp[i];
				REG_OCU_GPS_MESSAGE.data[i] = gps_message_temp[i];
			}
				gps_message_state = -1;
				//IEC0bits.U1RXIE = 0;
			

		}
		gps_message_state++;
	}

	gps_message_state_debug = gps_message_state;

}

void handle_power_button(void)
{

	unsigned int i;
	//static unsigned char computer_on_flag = 0;
	static unsigned char power_button_press_counter = 0;
	power_button_press_counter= 0;
	if(computer_on_flag == 0)
	{

		if(POWER_BUTTON())
		{

				if(battery_too_low == 0)
				{	
					computer_on_flag = 1;
					V3V3_ON(1);
					V5V_ON(1);
					V12V_ON(1);
					block_ms(100);
					while(SUS_S5() | SUS_S3());
					RED_LED_ON(1);
					while(!SUS_S5());
	
					GREEN_LED_ON(1);
					COMPUTER_PWR_OK(1);
	
					
	
	
					set_backlight_brightness(100);
					
	
					turned_on_latch++;
					while(POWER_BUTTON());
					while(!SUS_S3());
				//	block_ms(5000);					


					RED_LED_ON(0);
					block_ms(250);
					initialize_i2c_devices();
					init_fan_controller();
				}
				else
				{
					RED_LED_ON(0);
					GREEN_LED_ON(0);
					for(i=0;i<10;i++)
					{
						RED_LED_ON(1);
						block_ms(50);
						RED_LED_ON(0);
						block_ms(50);
					}
				}

		}


	}
	else
	{

			while(POWER_BUTTON())
			{
				power_button_press_counter++;
				if(power_button_press_counter > 10)
				{
					
					computer_on_flag = 0;
					GREEN_LED_ON(0);
					V3V3_ON(0);
					V5V_ON(0);
					V12V_ON(0);
					COMPUTER_PWR_OK(0);

				//	block_ms(100);
					power_button_press_counter = 0;
					
					break;
				}
				block_ms(100);
			}
			//if we've switched off power supplies but the power button is still depressed, block until it is released
			if(computer_on_flag == 0)
			{


				while(POWER_BUTTON())
				{
					power_button_press_counter++;
					if(power_button_press_counter > 40)
					{
						
					}
					if(power_button_press_counter > 50)
					{
					

	
							V5V_ON(1);
							V12V_ON(1);
							RED_LED_ON(1);
							GREEN_LED_ON(1);
							while(!SUS_S5())
							COMPUTER_PWR_OK(1);
							while(!SUS_S3());
							V3V3_ON(1);
							block_ms(1000);
							GREEN_LED_ON(0);
							V3V3_ON(0);
							V5V_ON(0);
							V12V_ON(0);
							COMPUTER_PWR_OK(0);
							RED_LED_ON(0);
							while(POWER_BUTTON());


							
							


						//__asm__ volatile ("RESET");
					}
					block_ms(100);
				}
				power_button_press_counter = 0;
			}
		//a little delay for debouncing
	

	}
}



void update_button_states(void)
{
	//get button states
	REG_SWITCH1_UP = TOGGLE1_UP();
	REG_SWITCH1_DOWN = TOGGLE1_DOWN();
	REG_SWITCH2_UP = TOGGLE2_UP();
	REG_SWITCH2_DOWN = TOGGLE2_DOWN();
	REG_VOLUME_UP = VOLUME_UP();
	REG_VOLUME_DOWN = VOLUME_DOWN();
	

	REG_HOME_BUTTON = MENU_BUTTON();
	REG_AUX_BUTTON1 = LIGHT_BUTTON();
	REG_AUX_BUTTON2 = TALK_BUTTON();
	

	
	
	REG_JOYSTICK2_X = return_adc_value(JOY2_X_CH);
	REG_JOYSTICK2_Y = return_adc_value(JOY2_Y_CH);
	REG_JOYSTICK1_X = return_adc_value(JOY1_X_CH);
	REG_JOYSTICK1_Y = return_adc_value(JOY1_Y_CH);


}

unsigned int return_adc_value(unsigned char ch)
{

	unsigned int return_value = 0;
	AD1CON1bits.ADON = 0;
	AD1CHS0bits.CH0SA = ch;
	AD1CON1bits.ADON = 1;	
	AD1CON1bits.SAMP = 1;
	while(!AD1CON1bits.DONE);
	return_value = ADC1BUF0;
	return return_value;
}



void ocu_batt_smbus_isr(void)
{

	IFS3bits.MI2C2IF = 0;
	//I2C2STATbits.BCL = 0;

	test_flag++;
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

	last_i2c_interrupt_state = i2c_interrupt_state;



	//If the interrupt FSM has finished, load the data into the registers
	if(ocu_batt_receive_word_completed == 1)
	{
		testa[6]++;
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
				main_loop_counter = 0;
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
		
		//I2C2CONbits.PEN = 1;
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

void set_backlight_brightness(unsigned char percentage)
{
	if(percentage > 100)
		percentage = 100;

	OC1R = percentage*8;

}

void ocu_gps_init(void)
{
	//set GPS tx to U1TX
	GPS_TX_OR = 3;
	
	//set U1RX to GPS rx pin
	RPINR18bits.U1RXR = GPS_RX_PIN;	
	
	U1BRG = 103;
	
	IEC0bits.U1RXIE = 0;
	
	U1MODEbits.UARTEN = 1;


}

void init_fan_controller(void)
{

	unsigned char temp1 = 0;
	unsigned char temp2 = 0;	

	IEC3bits.MI2C2IE = 1;

	//set fan configuration
	start_ocu_batt_i2c_write(I2C_ADD_FAN_CONTROLLER,0x02,0b00011010);
	block_ms(5);

	//make thermistor 1 control fan 2, and vice versa
	start_ocu_batt_i2c_write(I2C_ADD_FAN_CONTROLLER,0x11,0b00011000);
	block_ms(5);

	//set fan start duty cycle -> 120/240 = 50%
	start_ocu_batt_i2c_write(I2C_ADD_FAN_CONTROLLER,0x07,120);
	block_ms(5);

	//fan turns on at 35C
	start_ocu_batt_i2c_write(I2C_ADD_FAN_CONTROLLER,0x10,35);
	//fan turns on at 15C
	//start_ocu_batt_i2c_write(I2C_ADD_FAN_CONTROLLER,0x10,15);	
	block_ms(5);

	IEC3bits.MI2C2IE = 0;

}

/*void usb_dummy(void)
{
	V3V3_EN(1);
	V5V_EN(1);
	V3V3_ON(1);
	V5V_ON(1);
	GREEN_LED_EN(1);
	block_ms(1000);
			COMPUTER_PWR_OK(1);
			//set_backlight_brightness(50);
	TRISDbits.TRISD4 = 0;
	LATDbits.LATD4 = 1;
	
	while(!POWER_BUTTON());
	GREEN_LED_ON(1);


}*/



void init_io(void)
{
	AD1PCFGL = 0xffff;
	TRISB = 0xffff;
	TRISC = 0xffff;
	TRISD = 0xffff;
	TRISE = 0xffff;
	TRISF = 0xffff;
	TRISG = 0xffff;



    USBDeviceInit();
	USBDeviceAttach();
	

	//PMD1bits.ADC1MD = 1;
	//make sure all outputs are set to their powered-down state before the outputs are enabled
	CHARGER_ON(0);
	V3V3_ON(0);
	V5V_ON(0);
	V12V_ON(0);
	COMPUTER_PWR_OK(0);
	RED_LED_ON(0);
	GREEN_LED_ON(0);
	CAMERA_PWR_ON(0);
	
	COMPUTER_PWR_OK_EN(1);

	//CAMERA_PWR_ON(1);


	
	//Enable all outputs
	CHARGER_EN(1);
	V3V3_EN(1);
	V5V_EN(1);
	V12V_EN(1);
	RED_LED_EN(1);
	GREEN_LED_EN(1);
	CAMERA_PWR_EN(1);



	JOY1_X_EN(1);
	JOY1_Y_EN(1);
	JOY2_X_EN(1);
	JOY2_Y_EN(1);

	

	//initialize ADC
	AD1CON1 = 0x0000;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x0000;
	
	AD1CON3bits.ADCS = 0xff;
	AD1CON3bits.SAMC = 0x1f;

	//auto convert
	AD1CON1bits.SSRC = 7;

	//ocu_gps_init();

	//I2C1CONbits.I2CEN = 1;


}

void disable_io(void)
{


	//disable and enable ADC module.  Might be a silicon error, but
	//once ADON turns on module, Sleep() draws an extra 350uA in certain cases
	//(seems related to timing)
	PMD1bits.ADC1MD = 1; 
	Nop();
	PMD1bits.ADC1MD = 0;

	U1CON = 0;
	U1OTGCON = 0;
	_USBPWR = 0;


	V3V3_ON(0);
	V5V_ON(0);
	V12V_ON(0);

	//turn off LEDs -- they'll fade away disconcertingly otherwise
	RED_LED_ON(0);
	GREEN_LED_ON(0);
	block_ms(5);


	AD1CON1 = 0x0000;
	AD1CON2 = 0x0000;
	AD1CON3 = 0x0000;

	AD1PCFGL = 0xffff;
	TRISB = 0xffff;
	TRISC = 0xffff;
	TRISD = 0xffff;
	TRISE = 0xffff;
	TRISF = 0xffff;
	TRISG = 0xffff;
	
}



 void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void)
 {
	_CNIF = 0;
	_CNIE = 0;
 }



void joystick_interrupt(void)
{

//	_ASAM = 0;
	/*static int adc_state = 0;
	unsigned char ch = 0;
	testa[0] = _DONE;
	IFS0bits.AD1IF = 0;
	testa[1] = _DONE;*/
	_ASAM = 0;
	_AD1IF = 0;
/*	switch(adc_state)
	{
		case 0x00:
			REG_JOYSTICK2_Y = ADC1BUF0;
			ch = JOY1_X_CH;
		break;
		case 0x01:
			REG_JOYSTICK1_X = ADC1BUF0;
			ch = JOY1_Y_CH;
		break;
		case 0x02:
			REG_JOYSTICK1_Y = ADC1BUF0;
			ch = JOY2_X_CH;
		break;
		case 0x03:
			REG_JOYSTICK2_X = ADC1BUF0;
			ch = JOY2_Y_CH;
			adc_state = -1;
		break;
		default:
			ch = JOY2_Y_CH;
			adc_state = -1;
		break;
	}

	
	adc_state++;
	testa[2] = adc_state;


	AD1CON1bits.ADON = 0;
	AD1CHS0bits.CH0SA = ch;
	AD1CON1bits.ADON = 1;
	AD1CON1bits.SAMP = 1;*/

	REG_JOYSTICK1_X = ADC1BUF1;
	REG_JOYSTICK1_Y = ADC1BUF2;
	REG_JOYSTICK2_X = ADC1BUF0;
	REG_JOYSTICK2_Y = ADC1BUF3;

	
	adc_int_flag++;



}





void test_sleep_current(void)
{
while(1)
{
	unsigned int dummy;

	ClrWdt();

	//disable_io();
	//block_ms(1000);
	//set change interrupt to power button and charger ACOK signal
	_CN49IE = 1;
	_CN2IE = 1;
	dummy = PORTD;
	dummy = PORTB;
	_CNIF = 0;


	_CNIE = 1;	


	Sleep();

	_CNIE = 0;
	_CNIF = 0;
	dummy = PORTD;
	dummy = PORTB;
	//init_io();
	//block_ms(100);

	_ADON = 1;
	block_ms(1000);
	_ADON = 0;

	//need to wait at least 4 seconds,or the PIC uses about 350 uA extra current
	block_ms(4000);

}

}



void ocu_i2c1_isr(void)
{
	testb[0]++;
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
					testb[1]++;
	
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

	last_i2c1_device_state = i2c1_device_state;

	testb[3] = i2c1_device_state;
	testb[4] = timeout_counter;
	testb[5]++;
	

	//If the interrupt FSM has finished, load the data into the registers
	if(ocu_i2c1_receive_word_completed == 1)
	{

		ocu_i2c1_busy = 0;

		ocu_i2c1_receive_word_completed = 0;

		switch(i2c1_device_state)
		{
			case 0x00:
				i2c_loop_counter++;
 				REG_OCU_BATT_VOLTAGE = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;

			break;
			
			case 0x01:				
				
				REG_OCU_BATT_REL_SOC = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
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
				REG_OCU_BATT_CURRENT = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;
			break;

			case 0x06:
				battery_temperature = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state++;
			break;

			case 0x07:
				REG_OCU_BATT_FULL_CHARGE_CAP = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
				i2c1_device_state = 0;
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
				
				ocu_i2c1_interrupt_state = 0x00;
				ocu_i2c1_message_length = 2;
				start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,0x10);
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
		testb[9] = i2c1_device_state;
		testb[4]++;
		//i2c1_device_state++;
		i2c1_device_state++;
		ocu_i2c1_rx_byte1 = 0xff;
		ocu_i2c1_rx_byte2 = 0xff;
		ocu_i2c1_rx_byte3 = 0xff;
		ocu_i2c1_rx_byte4= 0xff;
		ocu_i2c1_interrupt_state = 0x00;
		
		//I2C1CONbits.PEN = 1;
		block_ms(100);

		I2C1STAT = 0;
		ocu_i2c1_receive_word_completed = 0;
		ocu_i2c1_busy = 0;
		timeout_counter = 0;
		

	}



}

void handle_charging(void)
{
//	static unsigned int charge_counter = 0;
	static unsigned char last_charging = 0;
	static unsigned char charge_counter = 0;
	
	if(CHARGER_ACOK())
	{

		//this kind of messes with the i2c stuff, so I don't want to do it every time
		if(charge_counter == 0)
			{
	
				last_charging = 1;
				//block_ms(500);
				block_ms(50);
	//			RED_LED_ON(1);
				block_ms(5);
			/*	start_ocu_i2c1_write(SMBUS_ADD_BQ24745,0x15,0x41a0);
				block_ms(20);
				//start_ocu_i2c1_write(SMBUS_ADD_BQ24745,0x14,0x0800);
				start_ocu_i2c1_write(SMBUS_ADD_BQ24745,0x14,0x0400);
				block_ms(20);*/
				start_ocu_i2c1_write(SMBUS_ADD_BQ24745,0x3f,0x0f80);
				block_ms(20);
	
				CHARGER_ON(1);
				//block_ms(500);
	
			}
		charge_counter++;
		
		
	}
	else if(CHARGER_ACOK() == 0)
	{
		last_charging = 0;
		CHARGER_ON(0);
		charge_counter = 0;
		//RED_LED_ON(0);
	}


	//if(charge_counter > 1000)
	//	charge_counter = 1000;


}


void read_all_bq2060a_registers(void)
{
	unsigned int i = 0;
	_MI2C1IE = 1;
	while(1)
	{
	for(i=0;i<128;i++)
	{
		ocu_i2c1_interrupt_state = 0x00;
		ocu_i2c1_message_length = 2;
		start_ocu_i2c1_i2c_read(SMBUS_ADD_BQ2060A,i);
		ocu_i2c1_receive_word_completed = 0;
		while(ocu_i2c1_receive_word_completed == 0);
		bq2060a_registers[i] = ocu_i2c1_rx_byte1 + (ocu_i2c1_rx_byte2<<8);
	}
	block_ms(1000);
	}
	

}































