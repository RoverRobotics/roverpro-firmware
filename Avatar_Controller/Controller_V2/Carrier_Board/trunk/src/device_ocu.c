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
#include "device_ocu_i2c.h"

#define BATTERY_SOC_CUTOFF  10

/*

Logic: 

if(BUS_PWR_STATE()) turn on everything

if(CHARGER_ACOK()) handle charging

*/


//button inputs
#define VOLUME_UP()		(_RG7)
#define VOLUME_DOWN()	(_RG6)
#define TOGGLE1_UP() 	(_RB15)
#define TOGGLE1_DOWN()	(_RB8)
#define TOGGLE2_DOWN()	(_RB14)
#define TOGGLE2_UP()	(_RB9)
#define TALK_BUTTON()	(!_RB3)
#define MENU_BUTTON()	(!_RB2)

#define LIGHT_BUTTON()	(!_RB4)
//#define POWER_BUTTON()	(!_RD0)
#define POWER_BUTTON()		0


#define CHARGER_ACOK()	(_RB0)

#define HUMIDITY_EN(a)	_PCFG1 = !a
#define HUMIDITY_CH		1

//joystick inputs
#define JOY1_X_EN(a)	_PCFG11 = !a
#define JOY1_Y_EN(a)	_PCFG12 = !a
#define JOY2_X_EN(a)	_PCFG10 = !a
#define JOY2_Y_EN(a)	_PCFG13 = !a

//joystick AN pin numbers
#define JOY1_X_CH		12
#define JOY1_Y_CH		11
#define JOY2_X_CH		13
#define JOY2_Y_CH		10

//indicators of COM Express power state
#define SUS_S5()			_RC14
#define SUS_S3()			_RD6

//indicates that COM Express has overheated
#define NC_THERM_TRIP	_RD2

//outputs
#define CHARGER_EN(a)	_TRISD11 = !a
#define CHARGER_ON(a)	_LATD11 = a

//power button on COM Express
//#define COMPUTER_PWR_EN(a)	_TRISB1 = !a
//#define COMPUTER_PWR_ON(a)	_LATB1 = a

#define V3V3_EN(a)		_TRISE7 = !a
#define V3V3_ON(a)		_LATE7 = a

#define V5V_EN(a)		_TRISE5 = !a
#define V5V_ON(a)		_LATE5 = a

#define V12V_EN(a)		_TRISE6	= !a
#define V12V_ON(a)		_LATE6 = !a

#define COMPUTER_PWR_OK_EN(a)	_TRISD1 = !a
#define COMPUTER_PWR_OK(a)		_LATD1 = a

#define GREEN_LED_EN(a)		_TRISD3 = !a
#define GREEN_LED_ON(a)		_LATD3 = a

#define RED_LED_EN(a)		_TRISD8 = !a
#define RED_LED_ON(a)		_LATD8 = a

#define GPS_TX_OR			_RP27R
#define GPS_RX_PIN			19

#define CAMERA_PWR_EN(a)	_TRISB5 = !a
#define CAMERA_PWR_ON(a)	_LATB5 = a		

#define LCD_PWM_OR			_RP25R

//microphone/speaker amp power

#define MIC_PWR_EN(a)		_TRISC13 = !a
#define MIC_PWR_ON(a)		_LATC13 = a

#define AMP_PWR_EN(a)		_TRISD7 = !a
#define AMP_PWR_ON(a)		_LATD7 = a

//pushbutton controller definitions

#define PWR_DWN_REQ()		!_RF0

#define PWR_KILL_EN(a)		_TRISF1 = !a
#define PWR_KILL_ON(a)		_LATF1 = !a

//#define BUS_PWR_STATE()		_RD0

#define PWR_BUTTON()		(!_RD0)



/*#define LCD_BACK_ON(a)	TRISDbits.TRISD4 = !a
#define LCD_BACK_ON(a)	LATDbits.LATD4 = a*/











//void ocu_batt_smbus_isr(void);






unsigned int test_flag = 0;
unsigned char testb[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned char testa[10] = {0,0,0,0,0,0,0,0,0,0};
unsigned int adc_int_flag = 0;





void handle_gas_gauge(void);



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

int battery_current = 0;
unsigned int battery_full_charge_capacity = 0;
unsigned char battery_too_low = 0;
//unsigned char gps_message_temp[100];
// /debugging

unsigned int lowest_battery_voltage = 20000;
unsigned int last_battery_voltage = 0;


int left_battery_current = 0;
int right_battery_current = 0;

void usb_dummy(void);


unsigned char usb_bus_sense_debug;
 void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void);

void joystick_interrupt(void);


void handle_charging(void);

unsigned int bq2060a_registers[128];

void read_all_bq2060a_registers(void);

void bringup_board(void);

int DeviceControllerBoot(void);

void set_backlight_brightness(unsigned char percentage);

//temp sensors work,as does accelerometer and compass
//humidity sensor changes voltage, haven't tested adc yet
//touchscreen works
//need to still test  battery gas gauges and battery charger

#pragma code

void read_touchscreen(void)
{
//	unsigned char r;
	unsigned char add, reg;
	unsigned char touch_pen;
	unsigned int touch_x;
	unsigned int touch_y;
	unsigned char a,b;

	add = TOUCH_CONTROLLER_I2C_ADD;
	reg = 0x00;

	IdleI2C2();
	StartI2C2();
	IdleI2C2();

	MasterWriteI2C2(add<<1);

	IdleI2C2();

	MasterWriteI2C2(reg);

	IdleI2C2();
/*	StopI2C2();
	IdleI2C2();
	StartI2C2();*/
	RestartI2C2();
	IdleI2C2();

	MasterWriteI2C2((add << 1) | 0x01);
	IdleI2C2();

	__delay_us(100);

	_MI2C2IF = 0;

	//r = (unsigned char)(MasterReadI2C1());

	//AckI2C1();

	//IdleI2C1();

	touch_pen = (unsigned char)(MasterReadI2C2());	

	AckI2C2();

	IdleI2C2();

	a = (unsigned char)(MasterReadI2C2());	

	AckI2C2();

	IdleI2C2();

	b = (unsigned char)(MasterReadI2C2());	

	touch_x = (b<<7) | a;

	AckI2C2();

	IdleI2C2();

	a = (unsigned char)(MasterReadI2C2());	

	AckI2C2();

	IdleI2C2();

	b = (unsigned char)(MasterReadI2C2());	
	touch_y = (b<<7) | a;
	//IdleI2C1();
	NotAckI2C2();

	// terminate read sequence (do not send ACK, send  STOP)
	IdleI2C2();
	StopI2C2(); 
	IdleI2C2();

	Nop();
	Nop();





}

void read_battery_rsoc(void)
{
	unsigned char battery_0_RSOC = 0;
	unsigned char battery_1_RSOC = 0;
	I2C_MUX_EN(1);

	OpenI2C1(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD  
                & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS 
				& I2C_NACK, I2C_RATE_SETTING);

	IdleI2C1();

	OpenI2C2(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD  
                & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS 
				& I2C_NACK, I2C_RATE_SETTING);

	IdleI2C2();

	CHARGER_EN(1);
	CHARGER_ON(1);

	while(1)
	{
		ClrWdt();
		I2C_MUX_CH(0);
		block_ms(200);
		battery_0_RSOC = readI2C1_Reg(0x0b, 0x0d);
		I2C_MUX_CH(1);
		block_ms(200);
		battery_1_RSOC = readI2C1_Reg(0x0b, 0x0d);
		block_ms(200);
		writeI2C2Word(SMBUS_ADD_BQ24745,0x15,0x41a0);
		block_ms(50);
		writeI2C2Word(SMBUS_ADD_BQ24745,0x14,0x0bb8);
		block_ms(50);
		writeI2C2Word(SMBUS_ADD_BQ24745,0x3f,0x0f80);
		block_ms(50);
		Nop();
		Nop();

	}

}

void test_fan_controller(void)
{

writeI2C2Reg(I2C_ADD_FAN_CONTROLLER,0x02,0b00011010);

writeI2C2Reg(I2C_ADD_FAN_CONTROLLER,0x11,0b00011000);
writeI2C2Reg(I2C_ADD_FAN_CONTROLLER,0x07,120);
writeI2C2Reg(I2C_ADD_FAN_CONTROLLER,0x10,15);




}

void bringup_i2c1(void)
{
	float temp_1, temp_2, accel_x, accel_y, accel_z;
	unsigned int mag_x, mag_y, mag_z;
	int a,b,c;

/*#define TOUCH_CONTROLLER_I2C_ADD 0x4d
#define SMBUS_ADD_TMP112_2 0x48
#define SMBUS_ADD_TMP112 0x49
#define SMBUS_ADD_BQ2060A 0x0b
#define SMBUS_ADD_BQ24745 0x09
#define I2C_ADD_FAN_CONTROLLER	0x18*/

	OpenI2C1(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD  
                & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS 
				& I2C_NACK, I2C_RATE_SETTING);

	IdleI2C1();

	OpenI2C2(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD  
                & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS 
				& I2C_NACK, I2C_RATE_SETTING);

	IdleI2C2();

	// wait a little before continuing...
	block_ms(100);


	writeI2C2Reg( I2C_ADD_HMC5843, 0x02,0x00);
	block_ms(5);
	writeI2C2Reg( I2C_ADXL345_ADD  , 0x2d,0x08);	
	block_ms(5);
	writeI2C2Reg( I2C_ADXL345_ADD  , 0x31,0x0b);	
	block_ms(5);

	while(1)
	{

		ClrWdt();
		a = readI2C2_Reg(SMBUS_ADD_TMP112,0x00);
		b = readI2C2_Reg(SMBUS_ADD_TMP112,0x01);
		c = (b >> 4 ) | (a <<4);
		temp_1 =  (float)c/16.0;

		block_ms(5);

		a = readI2C2_Reg(SMBUS_ADD_TMP112_2,0x00);
		b = readI2C2_Reg(SMBUS_ADD_TMP112_2,0x01);
		c = (b >> 4 ) | (a <<4);
		temp_2 =  (float)c/16.0;

		block_ms(5);

		a = readI2C2_Reg(I2C_ADXL345_ADD,0x32);
		b = readI2C2_Reg(I2C_ADXL345_ADD,0x33);
		c = a | (b << 8);
		accel_x = -(float)c/256.0;

		block_ms(5);

		a = readI2C2_Reg(I2C_ADXL345_ADD,0x34);
		b = readI2C2_Reg(I2C_ADXL345_ADD,0x35);
		c = a | (b << 8);
		accel_y = -(float)c/256.0;

		block_ms(5);

		a = readI2C2_Reg(I2C_ADXL345_ADD,0x36);
		b = readI2C2_Reg(I2C_ADXL345_ADD,0x37);
		c = a | (b << 8);
		accel_z = -(float)c/256.0;

		block_ms(5);

		a = readI2C2_Reg(I2C_ADD_HMC5843,0x03);
		b = readI2C2_Reg(I2C_ADD_HMC5843,0x04);
		c = b | (a << 8);
		mag_x = c;

		block_ms(5);
		
		a = readI2C2_Reg(I2C_ADD_HMC5843,0x05);
		b = readI2C2_Reg(I2C_ADD_HMC5843,0x06);
		c = b | (a << 8);
		mag_y = c;

		block_ms(5);

		a = readI2C2_Reg(I2C_ADD_HMC5843,0x07);
		b = readI2C2_Reg(I2C_ADD_HMC5843,0x08);
		c = b | (a << 8);
		mag_z = c;

		read_touchscreen();

		Nop();	

		block_ms(200);
		

	}




}

void bringup_board(void)
{

//	static unsigned int power_down_counter = 0;

/*
	init_io();

	PWR_KILL_EN(1);
	PWR_KILL_ON(0);
	MIC_PWR_EN(1);
	AMP_PWR_EN(1);
	//MIC_PWR_ON(1);
	//AMP_PWR_ON(1);
	CAMERA_PWR_EN(1);
	CAMERA_PWR_ON(1);

	while(1)
	{
		ClrWdt();
		if(PWR_BUTTON())
		{	
			GREEN_LED_ON(1);
			//V3V3_ON(1);
			//V5V_ON(1);
			//V12V_ON(1);
			//bringup_i2c1();
			//read_battery_rsoc();
			//test_fan_controller();


			if(computer_on_flag == 0)
			{
					//set to OC1
				LCD_PWM_OR = 18;	
				
				T2CONbits.TCKPS = 0;	
			
			
				
				//TPS61161: between 5kHz and 100 kHz PWM dimming frequency
				//Choose 20kHz (50us period)
				//50us = [PR2 + 1]*62.5ns*1
				PR2 = 801;	
				OC1RS = 800;
				set_backlight_brightness(50);
				
				//Sleep();
			
				OC1CON2bits.SYNCSEL = 0x1f;	
				
				//use timer 2
				OC1CON1bits.OCTSEL2 = 0;
				
				//edge-aligned pwm mode
				OC1CON1bits.OCM = 6;
				
				//turn on timer 2
				T2CONbits.TON = 1;
				if(DeviceControllerBoot() )
				{
					GREEN_LED_ON(1);
					computer_on_flag = 1;
				}
				V12V_ON(1);
			}
		
		
		}
		else
		{
			GREEN_LED_ON(0);
		}
		if(CHARGER_ACOK())
		{
			RED_LED_ON(1);
		}
		else
		{
			RED_LED_ON(0);
		}

		if(PWR_DWN_REQ())
		{
			while(PWR_DWN_REQ())
			{
				power_down_counter++;
				block_ms(10);
				if(power_down_counter > 50)
				{
					computer_on_flag = 0;
					PWR_KILL_ON(1);
					while(PWR_BUTTON())
					{
						ClrWdt();
					}
					block_ms(20);
				}
			}
			power_down_counter = 0;

		}
		

	}

	if(PWR_BUTTON())
	{
		if(computer_on_flag == 0)
		{
			if(DeviceControllerBoot() )
			{
				GREEN_LED_ON(1);
				computer_on_flag = 1;
			}
		}

	}
	else
	{
		computer_on_flag = 0;

	}

*/

}

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

	init_i2c();
	
	U1RXInterruptUserFunction = ocu_gps_isr;

	ocu_gps_init();	

	REG_OCU_BACKLIGHT_BRIGHTNESS = 0;
//	set_backlight_brightness(50);



}

void DeviceOcuProcessIO()
{
  static unsigned int initial_backlight_counter = 0;
	main_loop_counter++;

	handle_power_button();
	update_button_states();
		

		IEC0bits.U1RXIE = 1;
		IEC3bits.MI2C2IE = 1;
		_MI2C1IE = 1;
		ocu_batt_i2c_fsm();
		ocu_i2c1_fsm();

	handle_charging();
	handle_gas_gauge();

  //I need to check if the computer is on, since these buttons are "pressed" when the power supply
  //if soff
  if(computer_on_flag)
  {
  	if(MENU_BUTTON() && TALK_BUTTON() && LIGHT_BUTTON())
  	{
  		while(MENU_BUTTON() || TALK_BUTTON() || LIGHT_BUTTON())
  		{
  			RED_LED_ON(1);
  			GREEN_LED_ON(1);
  			//ClrWdt();
  		}
  			RED_LED_ON(0);
        //GREEN_LED_ON(0);
  	}
  }


	if( (computer_on_flag) && (NC_THERM_TRIP==0) )
	{
		while(1)
		{
			ClrWdt();
			GREEN_LED_ON(1);
			block_ms(200);
			GREEN_LED_ON(0);
			RED_LED_ON(1);
			block_ms(200);
			RED_LED_ON(0);
/*			if(POWER_BUTTON())
			{
				computer_on_flag = 0;
				GREEN_LED_ON(0);
				V3V3_ON(0);
				V5V_ON(0);
				V12V_ON(0);
				COMPUTER_PWR_OK(0);
				while(POWER_BUTTON());
				block_ms(100);
				break;
			}*/
			


		}
	}

	//computer has shut down.  turn off power supplies
	if( (SUS_S3()==0) && (SUS_S5() == 0) && (computer_on_flag == 1))
//  if( (SUS_S3()==0) && (SUS_S5() == 0) )
	{
          block_ms(50);
          if(PWR_BUTTON() == 0)
          {
  					computer_on_flag = 0;
  					GREEN_LED_ON(0);
  					V3V3_ON(0);
  					V5V_ON(0);
  					V12V_ON(0);
  					COMPUTER_PWR_OK(0);
  					PWR_KILL_ON(1);
  					block_ms(100);
          }
	}

//	while(!POWER_BUTTON());
//	GREEN_LED_ON(1);


  //implement a timeout on the black screen
  //if software doesn't start, or BIOS needs to be changed
  //we'll be able to see the screen
  if(computer_on_flag)
  {
    if(REG_OCU_BACKLIGHT_BRIGHTNESS == 0)
    {
      if(initial_backlight_counter > 300)
      {
        initial_backlight_counter = 301;
        REG_OCU_BACKLIGHT_BRIGHTNESS = 50;
      }
      else
      {
        initial_backlight_counter++;
        block_ms(100);
      }
    }
  }
  //if the computer is off, reset the backlight counter and register,
  //so that the backlight stays off during power cycles when
  //the AC adapter is plugged in
  else
  {
      REG_OCU_BACKLIGHT_BRIGHTNESS = 0;
      initial_backlight_counter = 0;
  }

	set_backlight_brightness(REG_OCU_BACKLIGHT_BRIGHTNESS);
  if(REG_OCU_CAMERA_POWER_ON)
  {
    CAMERA_PWR_ON(1);
  }
  else
  {
    CAMERA_PWR_ON(0);
  }


}

void handle_gas_gauge(void)
{

//	static unsigned int low_voltage_counter = 0;
  static unsigned int low_capacity_counter = 0;
  static unsigned int initial_low_capacity_counter = 0;
  unsigned char i;

  if( (left_battery_current == 0xffff) || (right_battery_current == 0xffff))
  {
    REG_OCU_BATT_CURRENT = 0xffff;
  }
  else
  {
    REG_OCU_BATT_CURRENT = left_battery_current+right_battery_current;
  }

	if( (REG_OCU_REL_SOC_L < BATTERY_SOC_CUTOFF) || (REG_OCU_REL_SOC_R < BATTERY_SOC_CUTOFF) )
	{
    initial_low_capacity_counter++;
    if(initial_low_capacity_counter > 1000)
      initial_low_capacity_counter = 2000;

    if( ((REG_OCU_REL_SOC_L == 0 ) || (REG_OCU_REL_SOC_R == 0 )) && (initial_low_capacity_counter < 1000) )   
    {
      //initally, relative SOC registers will be 0.  Let's not turn off due to low capacity until we get a
      //valid capacity reading, or the counter goes too high.
    }
    else
    { 
      low_capacity_counter++;
    }
	}

  //if we're getting bad SOC data on both the batteries -- if we have a valid measurement on just one, then we'll assume
  //that the other battery approximates the SOC of the one we have an invalid measurement for.
  else if( (REG_OCU_REL_SOC_L == 0xffff) && (REG_OCU_REL_SOC_R == 0xffff) )
  {
      low_capacity_counter++;
  }
  else if( (REG_OCU_REL_SOC_L >= BATTERY_SOC_CUTOFF) && (REG_OCU_REL_SOC_R >= BATTERY_SOC_CUTOFF) )
  {
    low_capacity_counter = 0;
    battery_too_low = 0;

  }

	//if counter gets to 10 different readings, shut off
	if(low_capacity_counter >= 10)
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
      for(i=0;i<4;i++)
      {
        RED_LED_ON(1);
        block_ms(200);
        RED_LED_ON(0);
        block_ms(200);
      }
      PWR_KILL_ON(1);
      block_ms(100);
		}
		low_capacity_counter = 200;
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

//	unsigned int i;
	static unsigned int power_down_counter = 0;
	//static unsigned char computer_on_flag = 0;
	static unsigned char power_button_press_counter = 0;
	power_button_press_counter= 0;

			if(PWR_BUTTON() && (computer_on_flag == 0) )
			{

//					set_backlight_brightness(50);

					if(DeviceControllerBoot() )
					{
						GREEN_LED_ON(1);
						computer_on_flag = 1;
						block_ms(100);
						initialize_i2c_devices();
						V12V_ON(1);
						init_fan_controller();
					}
					

				
			}


		if(PWR_DWN_REQ())
		{
			while(PWR_DWN_REQ())
			{
				power_down_counter++;
				block_ms(10);
				if(power_down_counter > 50)
				{
					PWR_KILL_ON(1);
					//if we're still running code, then AC adapter 
					//plugged in.  Turn off supplies
					GREEN_LED_ON(0);
					computer_on_flag = 0;
					V12V_ON(0);
					V5V_ON(0);
					V3V3_ON(0);
					//make sure that the power button is released and has finished
					//bouncing before we return (so controller doesn't think the
					//power button has been pressed again, and turn right back on.
					while(PWR_BUTTON())
					{
						ClrWdt();
					}
					//so power button controller will work properly
					PWR_KILL_ON(0);
					block_ms(20);
				}
			}
			power_down_counter = 0;

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

	REG_OCU_HUMIDITY = return_adc_value(HUMIDITY_CH);


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

//	unsigned char temp1 = 0;
//	unsigned char temp2 = 0;	

	IEC3bits.MI2C2IE = 1;

  //wait to make sure the +3.3V supply stabilizes
  block_ms(500);

	//set fan configuration
	start_ocu_batt_i2c_write(I2C_ADD_FAN_CONTROLLER,0x02,0b00011010);
	block_ms(5);

	//make thermistor 1 control fan 2, and vice versa
	start_ocu_batt_i2c_write(I2C_ADD_FAN_CONTROLLER,0x11,0b00011000);
	block_ms(5);

	//set fan start duty cycle -> 120/240 = 50%
	start_ocu_batt_i2c_write(I2C_ADD_FAN_CONTROLLER,0x07,120);
	block_ms(5);


	//fan turns on at 0C
	start_ocu_batt_i2c_write(I2C_ADD_FAN_CONTROLLER,0x10,0);	
  block_ms(200);
	//fan turns on at 35C
	start_ocu_batt_i2c_write(I2C_ADD_FAN_CONTROLLER,0x10,35);

	
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
	MIC_PWR_ON(1);
	AMP_PWR_ON(1);
	I2C_MUX_CH(0);
	PWR_KILL_ON(0);
	
	COMPUTER_PWR_OK_EN(1);

	MIC_PWR_EN(1);
	AMP_PWR_EN(1);
	I2C_MUX_EN(1);
	PWR_KILL_EN(1);
	
	//Enable all outputs
	CHARGER_EN(1);
	V3V3_EN(1);
	V5V_EN(1);
	V12V_EN(1);
	RED_LED_EN(1);
	GREEN_LED_EN(1);
	CAMERA_PWR_EN(1);

	HUMIDITY_EN(1);

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


int DeviceControllerBoot(void)
{
//	unsigned char a;
	unsigned int i = 0;

	V3V3_ON(1);

	//while(!V3V3_PGOOD());

	//weird -- long interval between turning on V3V3 and V5 doesn't allow COM Express to boot
	block_ms(100);

	V5V_ON(1);

	while(SUS_S5() | SUS_S3())
	{
		i++;
		if(i > 5) return 0;
		ClrWdt();
		block_ms(100);
	}
	i=0;
	ClrWdt();
	block_ms(100);

//	send_lcd_string("Boot 1  \r\n",10);

/*	for(i=0;i<400;i++)
	{
		block_ms(10);
		ClrWdt();
	}*/

	//while(!V5_PGOOD());

	while(!SUS_S5())
	{
		i++;
		if(i > 20) return 0;
		ClrWdt();
		block_ms(100);

	}

//	send_lcd_string("Boot 2  \r\n",10);
	i=0;

	COMPUTER_PWR_OK(1);

	while(!SUS_S3())
	{
		i++;
		if(i > 20) return 0;
		ClrWdt();
		block_ms(100);
	}

//	blink_led(6,500);

	return 1;


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





void handle_charging(void)
{
//	static unsigned int charge_counter = 0;
	static unsigned char last_charging = 0;
	static unsigned char charge_counter = 0;
	
	if(CHARGER_ACOK())
	{

		V3V3_ON(1);

    last_charging = 1;
    CHARGER_ON(1);
/*
		//this kind of messes with the i2c stuff, so I don't want to do it every time
		if(charge_counter == 0)
			{
	
				last_charging = 1;
				//block_ms(500);
        
				block_ms(50);
	//			RED_LED_ON(1);
				block_ms(5);
				start_ocu_batt_i2c_write(SMBUS_ADD_BQ24745,0x15,0x41a0);
				block_ms(20);
				//start_ocu_i2c1_write(SMBUS_ADD_BQ24745,0x14,0x0800);
				start_ocu_batt_i2c_write(SMBUS_ADD_BQ24745,0x14,0x0bb8);
				block_ms(20);
				start_ocu_batt_i2c_write(SMBUS_ADD_BQ24745,0x3f,0x0bb8);
				block_ms(20);
	
				CHARGER_ON(1);
				//block_ms(500);
	
			}
		charge_counter++;*/
		
		
	}
	else if(CHARGER_ACOK() == 0)
	{
		last_charging = 0;
		CHARGER_ON(0);
		charge_counter = 0;
		//RED_LED_ON(0);
	}

	//turn on red LED to indicate charging, but only if computer is off
	if(REG_OCU_BATT_CURRENT > 0)
	{
		if(computer_on_flag == 0)
			RED_LED_ON(1);
    else
      RED_LED_ON(0);
		
	}
	else
	{
		RED_LED_ON(0);
	}

	//if(charge_counter > 1000)
	//	charge_counter = 1000;


}


void read_all_bq2060a_registers(void)
{
/*	unsigned int i = 0;
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
	}*/
	

}































