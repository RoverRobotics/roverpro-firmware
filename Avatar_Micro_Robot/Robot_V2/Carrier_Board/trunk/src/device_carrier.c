/**
 * @file device_carrier.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex carrier PIC firmware.
 *
 * DEVICE MAPPING
 * Hardware Revision 30 (Robotex Robot Carrier Board)
 *
 * ----------------------------------------------------------------------------
 * DIGITAL OUTPUTS
 * ----------------------------------------------------------------------------
 * NAME                 PIN PORT  DEF FUNCTION
 * ------------------------------
 * EN_SMBPWR            4   RG6   0   EN +3.3V_SMB, HW pulldown
 * CODEC_PD# (EN_AUDIO) 5   RG7   0   EN +3.3V_AUDIO and +5V_AUDIO, HW pulldown
 * 12V_ENABLE (EN_12V#) 6   RG8   1   EN +12V supply, HW pullup
 * EN_RADIOBRD          8   RG9   0   EN +3.3V_RBRD, HW pulldown
 * NCOM_WAKE0#          30  RB15  1   COMEx PCIE wake (pullup on COMEx)
 * NCOM_SYSRST#         22  RB9   Z   Rst push-button on PCB, (pullup on COMEx & PCB), don't override button
 * EN_COMXPRESS         59  RF1   0   EN 3.3V to +3.3V_C-XPRS, HW pulldown
 * NCOM_WAKE1#          58  RF0   1   COMEx AUX Wake (pull-up on COMEx)
 * PWR_OK               23  RB10  0   COMEx power good signal
 * EN_PAYLOAD2          24  RB11  0   EN raw power to payload #2, HW pulldown
 * EN_PAYLOAD1          27  RB12  0   EN raw power to payload #1, HW pulldown
 * EN_MOTORCNTL         28  RB13  0   EN 5V to Motor Controller, HW pulldown
 * 3V3_ENABLE           48  RC14  0   EN +3.3V_ALWS, HW pulldown
 * 5V_ENABLE            32  RF5   0   EN +5V, HW pulldown
 * EN_GPS               31  RF4   0   EN 3.3V to +3.3V_GPS, HW pulldown
 * PWRBTN#              42  RD8   Z   Pwr push-button on PCB, (pull-up on COMEx & PCB), don't override button
 * PWM_IRLED            51  RD3   0   Straight out to Camera Board 
 * PWM_LED              52  RD4   0   Straight out to Camera Board
 * EN_CAMERABRD         2   RE6   0   EN 5V, 3.3V and 12V to Camera Board
 *
 * ---------------------------------------------------------------------------
 * DIGITAL INPUTS
 * ---------------------------------------------------------------------------
 * PIC_WAKE0#           29  RB14  Z   wake from radio PCIE
 * PAYLOAD1_PRESENT#    21  RB8   Z   indicates payload #1 connected
 * PAYLOAD2_PRESENT#    13  RB3   Z   indicates payload #2 connected
 * COMPASS_DRDY#        14  RB2   Z   indicates compass data ready (check!)
 * PWR_BTN#             12  RB4   Z   Pwr push-button on PCB, (pull-up on COMEx & PCB), don't override button
 * SUS_S5#              11  RB5   Z   indicates system is suspended to disk
 * SMB_ALERT#           47  RC13  Z   indicates message pending on SMBus, HW pullup
 * 33V_ALWS_GOOD        49  RD1   Z   if not good probably not alive
 * 12V_GOOD             50  RD2   Z   12V main good signal
 * 5V_PGOOD             54  RD6   Z   indicates 5V is good
 * SUS_S3#              1   RE5   Z   indicates system is suspended to RAM
 * NC_THRMTRIP#         3   RE7   Z   indicates thermal problem with COMEx
 *
 * ---------------------------------------------------------------------------
 * ANALOG INPUTS
 * ---------------------------------------------------------------------------
 * EX_TEMP_SNS          16   AN0  Z   pulled to 3.3V with 100k
 * HUMIDITY_SENSOR      15   AN1  Z   humidity sensor
 *
 *
 */

//#define NO_COMPUTER_INSTALLED

#include "stdhdr.h"
#include "device_carrier.h"
#include "testing.h"

#define V3V3_ON(a)               _LATC14    = a
#define V3V3_EN(a)               _TRISC14 = !a

#define V5_ON(a)                 _LATG8     = a
#define V5_EN(a)                 _TRISG8  = !a

#define V12_ON(a)                _LATE6     = !a // inverse logic
#define V12_EN(a)                _TRISE6  = !a

#define VBAT_DIGI_EN(a)			_TRISB11 = !a
#define VBAT_DIGI_ON(a)			_LATB11 = a

#define V5_PGOOD()				_RD6
#define V12_PGOOD()				_RD2

#define V3V3_PGOOD()			_RD1	

#define AMP_PWR_EN(a)			_TRISD7 = !a
#define AMP_PWR_ON(a)			_LATD7 = a

#define COM_EXPRESS_PGOOD_EN(a)	_TRISB10 = !a
#define COM_EXPRESS_PGOOD_ON(a)	_LATB10 = a

#define MIC_PWR_EN(a)			_TRISG6 = !a
#define MIC_PWR_ON(a)			_LATG6 = a

#define CODEC_PWR_EN(a)			_TRISG7 = !a
#define CODEC_PWR_ON(a)			_LATG7 = a


#define SUS_S5()					_RB5
#define SUS_S3()					_RE5

#define HUMIDITY_SENSOR_CH       1 
#define HUMIDITY_SENSOR_EN(a)	_PCFG1 = !a

#define ADC_REF_VOLTAGE          3.3f
#define ADC_SAMPLE_COUNT         1024

#define WHITE_LED_OR			_RP25R
#define IR_LED_OR				_RP22R

#define POWER_BUTTON()			!_RB4

#define GPS_TX_OR				_RP12R
#define GPS_RX_PIN				11

#define NC_THERM_TRIP()			_RE7

#define WDT_PIN_EN(a)			_TRISF1 = !a
#define WDT_PIN()				_LATF1

#define REAR_PL_PWR_EN(a)		_TRISB12 = !a
#define REAR_PL_PWR_ON(a)		_LATB12 = a

#define REAR_PL_PRESENT()		_RB8

void toggle_wdt_loop(void);





#define ADXL345_ADDRESS           0x53
#define TMP112_0_ADDRESS          0x48
#define TMP112_1_ADDRESS          0x49
#define HMC5843_ADDRESS           0x1E
#define EEPROM_ADDRESS            0x50 // to 0x57 (0x50 to 0x53 address four 256-byte blocks
#define FAN_CONTROLLER_ADDRESS		0x18


void read_EEPROM_string(void);

static unsigned int number_of_resets = 0;
unsigned int usb_timeout_counter = 0;
unsigned char first_usb_message_received = 0;

#define RST_POWER_ON                  0x0000
#define RST_SHUTDOWN                  0x0001
#define RST_SHUTDOWN_OVERHEAT         0x0002
#define RST_SHUTDOWN_USB_TIMEOUT      0x0003
#define RST_SOFTWARE_FAIL             0x0004


//unsigned int reg_robot_gps_message[100];
//unsigned char eeprom_string[78];


// STATE MACHINE

typedef enum
{
	CARRIER_INIT = 0,      ///< TRANSITIONAL when carrier board is power cycled everything is driven off
	CARRIER_WAIT = 1,      ///<              wait for ext PWR button to be pressed
	CARRIER_BOOT = 2,      ///< TRANSITIONAL boot sequence for full power
	CARRIER_RUN  = 3,      ///<              running normally with full power
	CARRIER_SHUTDOWN = 4,  ///< TRANSITIONAL go back into all off mode
	CARRIER_SUSPEND = 5,   ///< TRANSITIONAL S3 suspend to RAM with wake-on-lan or PWR button
	CARRIER_SUSPENDED = 6  ///<              running suspended
} CARRIER_STATE_ENUM;

int gRegCarrierState;

// PROTOTYPES

void DeviceCarrierGetTelemetry();
int DeviceCarrierBoot();
void initI2C( void );



#define IR_LED		0
#define WHITE_LED	1
void set_led_brightness(unsigned char led_type, unsigned char duty_cycle);

void robot_gps_isr(void);
void robot_gps_init(void);


void init_io(void);
void de_init_io(void);
void blink_led(unsigned int n, unsigned int ms);
void init_pwm(void);
void init_fan(void);
void update_audio_power_state(void);
void hard_reset_robot(void);
static void handle_reset(void);

// FUNCTIONS

#pragma code

void DeviceCarrierInit()
{
	int i = 0;

  if(number_of_resets == 0)
  {  
    REG_ROBOT_RESET_CODE = RST_POWER_ON;
  }

	de_init_io();

 


	//enable software watchdog if power button is not held down.  Otherwise, sleep forever.
	if(POWER_BUTTON())
	{
    WDT_PIN_EN(1);
    while(1)
    {
      WDT_PIN() = 0;
      //block_ms(100);
      Sleep();
      WDT_PIN() = 1;
      //loop, so that WDT_PIN stays high at least 1us
      for(i=0;i<50;i++)
      {
        Nop();
      }
      ClrWdt();
     }
	}
	else
	{
    //start robot normally
	}

	WDT_PIN_EN(1);

	handle_watchdogs();

	//Sleep();

	//ClrWdt();
	//wait some time to stabilize voltages
	block_ms(50);
//	while(POWER_BUTTON());

	init_io();

	//turn on rear payload power (if necessary) now,
	//in case we need power to a keyboard to control the BIOS
	if(REAR_PL_PRESENT() == 0)
	{
		REAR_PL_PWR_ON(1);
	}
	else
	{
		REAR_PL_PWR_ON(0);
	}

	WDT_PIN_EN(1);

	const unsigned char build_date[12] = __DATE__; 
	const unsigned char build_time[12] = __TIME__;



	for(i=0;i<12;i++)
	{
		if((build_date[i] == 0))
		{
			REG_ROBOT_FIRMWARE_BUILD.data[i] = ' ';
		}
		else
		{
			REG_ROBOT_FIRMWARE_BUILD.data[i] = build_date[i];
		}
		if(build_time[i] == 0)
		{
			REG_ROBOT_FIRMWARE_BUILD.data[i+12] = ' ';
		}
		else
		{
			REG_ROBOT_FIRMWARE_BUILD.data[i+12] = build_time[i];
		}
	}




	init_lcd_uart();

	init_pwm();
	
	//turn on 12V regulator, and MOSFET for 12V regulator input
	//This is so that the side fan will turn on, indicating that the power board has turned on
	VBAT_DIGI_ON(1);
	V12_ON(1);

	//if COM Express isn't installed, don't try to
	//boot it (just turn on supplies)
	#ifdef NO_COMPUTER_INSTALLED
		V3V3_ON(1);
		V5_ON(1);
	#else
		//keep trying to boot COM Express until successful
		while (1)
			{
		
				if(DeviceCarrierBoot() == 0)
				{
					send_lcd_string("Computer boot failed  ",22);
					blink_led(3,2000);
					{__asm__ volatile ("reset");};
				}
				else
					break;
			}
	#endif
	
	send_lcd_string("Computer booted  \r\n",19);

	block_ms(100);
	CODEC_PWR_ON(1);
//	MIC_PWR_ON(1);
//	AMP_PWR_ON(1);
	handle_watchdogs();

	initI2C();

	block_ms(100);
	handle_watchdogs();

	writeI2CReg( HMC5843_ADDRESS, 0x02,0x00);
	block_ms(5);
	writeI2CReg( ADXL345_ADDRESS, 0x2d,0x08);	
	block_ms(5);
	writeI2CReg( ADXL345_ADDRESS, 0x31,0x0b);	
	block_ms(5);




	// enable A/D Converter
	_SSRC = 0x07; // auto-convert
	_SAMC = 0x1f; // holding (enable this to sample)
	_ADON = 1;


	init_fan();



	U2RXInterruptUserFunction = robot_gps_isr;

	robot_gps_init();
	_U2RXIE = 1;

	read_EEPROM_string();

	display_board_number();

	REG_CARRIER_SPEAKER_ON = 1;
	REG_CARRIER_MIC_ON = 1;

	send_lcd_string("Init finished  \r\n",17);

	//T1InterruptUserFunction = DeviceCarrierGetTelemetry;
}

void init_fan(void)
{

	unsigned int i;

	handle_watchdogs();

	//set fan configuration
	writeI2CReg( FAN_CONTROLLER_ADDRESS,0x02,0b00011010);
	block_ms(5);


/* For thermistor control */
	//make thermistor 1 control fan 2, and vice versa
	writeI2CReg( FAN_CONTROLLER_ADDRESS,0x11,0b00011000);
	block_ms(5);

	//set fan start duty cycle -> 120/240 = 50%
	writeI2CReg( FAN_CONTROLLER_ADDRESS,0x07,120);
	block_ms(5);

	//first, set fan to turn on at 15C
	writeI2CReg( FAN_CONTROLLER_ADDRESS,0x10,15);

	block_ms(5);

	//wait a while
	for(i=0;i<50;i++)
	{
		handle_watchdogs();
		block_ms(20);
	}

	//fan turns on at 40C
	writeI2CReg( FAN_CONTROLLER_ADDRESS,0x10,40);
	block_ms(5);
	//End thermistor control

/* For software control

	writeI2CReg(FAN_CONTROLLER_ADDRESS,0x11,0x00);

	block_ms(5);

	writeI2CReg(FAN_CONTROLLER_ADDRESS,0x12,0);

	block_ms(5);

	REG_CARRIER_REAR_BLOWER_SPEED = 100;
	
	writeI2CReg(FAN_CONTROLLER_ADDRESS,0x0B,240);
	ClrWdt();
	block_ms(1000);*/

	//end software control

	handle_watchdogs();


}

void init_pwm(void)
{


	//setup LED PWM channels

	T2CONbits.TCKPS = 0;	


	WHITE_LED_OR = 18;	//OC1
	IR_LED_OR	= 19;	//OC2


	//AP8803: dimming frequency must be below 500Hz
	//Choose 400Hz (2.5ms period)
	//2.5ms = [PR2 + 1]*62.5ns*1
	PR2 = 40000;	
	OC1RS = 39999;
	OC2RS = 39999;

	OC1CON2bits.SYNCSEL = 0x1f;	
	OC2CON2bits.SYNCSEL = 0x1f;	
	
	//use timer 2
	OC1CON1bits.OCTSEL2 = 0;
	OC2CON1bits.OCTSEL2 = 0;
	
	//edge-aligned pwm mode
	OC1CON1bits.OCM = 6;
	OC2CON1bits.OCM = 6;
	
	//turn on timer 2
	T2CONbits.TON = 1;

}

void blink_led(unsigned int n, unsigned int ms)
{
	unsigned int i,j, max_j;
	
	handle_watchdogs();

	max_j = ms/20;

	for(i=0;i<n;i++)
	{
		set_led_brightness(WHITE_LED,50);
		for(j=0;j<max_j;j++)
		{
			block_ms(10);
			handle_watchdogs();
		}
		set_led_brightness(WHITE_LED,0);
		for(j=0;j<max_j;j++)
		{
			block_ms(10);
			handle_watchdogs();
		}

	}


}

void set_led_brightness(unsigned char led_type, unsigned char duty_cycle)
{


	if(duty_cycle > 100)
		duty_cycle = 100;

	switch(led_type)
	{
		case WHITE_LED:
			OC1R = 39990 - duty_cycle*399;
		break;
		case IR_LED:
			OC2R = 39990 - duty_cycle*399;
		break;

	}


}


void init_io(void)
{

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


	VBAT_DIGI_ON(0);
	V3V3_ON(0);
	V5_ON(0);
	V12_ON(0);
	AMP_PWR_ON(0);
	CODEC_PWR_ON(0);
	COM_EXPRESS_PGOOD_ON(0);
	REAR_PL_PWR_ON(0);


	VBAT_DIGI_EN(1);
	V3V3_EN(1);
	V5_EN(1);
	V12_EN(1);
	AMP_PWR_EN(1);
	MIC_PWR_EN(1);
	CODEC_PWR_EN(1);
	COM_EXPRESS_PGOOD_EN(1);
	HUMIDITY_SENSOR_EN(1);
	REAR_PL_PWR_EN(1);


}

void de_init_io(void)
{

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

//reads PCB information from the EEPROM.  This is pretty inefficient, but it should only run once, while the COM Express
//is booting.
void read_EEPROM_string(void)
{
	unsigned int i;

	for(i=0;i<78;i++)
	{
		handle_watchdogs();
		REG_ROBOT_BOARD_DATA.data[i] = readI2C_Reg(EEPROM_ADDRESS,i);
		block_ms(5);

	}


}


void initI2C(void) // Initialize the I2C interface to the realtime clock
{
	// Configure I2C for 7 bit address mode 100kHz

	ODCGbits.ODG3 = 1; // SDA1 is set to open drain
	ODCGbits.ODG2 = 1; // SCL1 is set to open drain

	OpenI2C1(I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD & I2C_IPMI_DIS & I2C_7BIT_ADD  
                & I2C_SLW_DIS & I2C_SM_DIS & I2C_GCALL_DIS & I2C_STR_DIS 
				& I2C_NACK, I2C_RATE_SETTING);

	IdleI2C1();

	// wait a little before continuing...
	handle_watchdogs();
	block_ms(100);
	handle_watchdogs();
} //initI2C


int DeviceCarrierReadAdxl345Register( unsigned char add, unsigned char reg )
{
	int a,b;
	int c;

	a = readI2C_Reg(add,reg);
	b = readI2C_Reg(add,reg+1);


	c = a | (b << 8);

	return c;
}

int DeviceCarrierReadTmp112Register( unsigned char add, unsigned char reg )
{
	int a, b, c;

	a = readI2C_Reg(add,reg);

	b = readI2C_Reg(add,reg+1);

	c = (b >> 4) | (a << 4);

	return c;
}

int DeviceCarrierReadHmc5843Register( unsigned char add, unsigned char reg )
{
	unsigned char a, b;
	int c;

	a = readI2C_Reg(add,reg);

	b = readI2C_Reg(add,reg+1);

	c = b | (a << 8);

	return c;
}


void robot_gps_init(void)
{
	//set GPS tx to U2TX
	GPS_TX_OR = 5;
	
	//set U2RX to GPS rx pin
	_U2RXR = GPS_RX_PIN;	
	
	U2BRG = 103;
	
	_U2RXIE = 0;
	
	U2MODEbits.UARTEN = 1;


}

void robot_gps_isr(void)
{

	

	static unsigned char gps_message_state = 0;
	static unsigned char gps_message_temp[100];

	//filter for messages starting with $GPGGA
	unsigned char message_filter[6] = {'$','G','P','G','G','A'};
	unsigned char i;

	unsigned char new_byte;

	U2STAbits.OERR = 0;

	_U2RXIF = 0;

	new_byte = U2RXREG;
//	gps_interrupt_counter++;
	

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
				REG_ROBOT_GPS_MESSAGE.data[i] = gps_message_temp[i];
			}
				gps_message_state = 0;
        return;
				//IEC0bits.U1RXIE = 0;
			

		}
		gps_message_state++;
	}
	//gps_message_state_debug = gps_message_state;

}

void DeviceCarrierGetTelemetry()
{
	unsigned char a, b;
	signed int c;
	int d;

 	IFS0bits.T1IF = 0;	//clear interrupt flag ??

	a = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x32 ); // Accel X-Axis Data 0
	b = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x33 ); // Accel X-Axis Data 1
	c = a | (b << 8);
	REG_ACCEL_X = (float)c * 16.0 / 4096.0;


	a = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x34 ); // Accel Y-Axis Data 0
	b = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x35 ); // Accel Y-Axis Data 1
	c = a | (b << 8);
	REG_ACCEL_Y = (float)c * 16.0 / 4096.0;


	a = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x36 ); // Accel Z-Axis Data 0
	b = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x37 ); // Accel Z-Axis Data 1
	c = a | (b << 8);
	REG_ACCEL_Z = -(float)c * 16.0 / 4096.0;


	d = DeviceCarrierReadTmp112Register( TMP112_0_ADDRESS, 0x00 ); // Temp
	REG_TEMP_INT0 = (float)d / 16.0; 


	d = DeviceCarrierReadHmc5843Register( HMC5843_ADDRESS, 0x03 ); // Magnet X-Axis Data
	REG_MAGNETIC_X = (float)d;


	d = DeviceCarrierReadHmc5843Register( HMC5843_ADDRESS, 0x05 ); // Magnet Y-Axis Data 0
	REG_MAGNETIC_Y = (float)d;


	d = DeviceCarrierReadHmc5843Register( HMC5843_ADDRESS, 0x07 ); // Magnet Z-Axis Data 0
	REG_MAGNETIC_Z = (float)d;


	_ADON = 0;
	_CH0SA = HUMIDITY_SENSOR_CH;
	_ADON = 1;
	_SAMP = 1;
	
	while (!_DONE) { }

	REG_ROBOT_HUMIDITY = ADC1BUF0 * ADC_REF_VOLTAGE / ADC_SAMPLE_COUNT;

//	writeI2CReg(FAN_CONTROLLER_ADDRESS,0x0B,REG_CARRIER_REAR_BLOWER_SPEED);

	REG_TELEMETRY_COUNT++;
}




int DeviceCarrierBoot()
{
	unsigned int i = 0;

	V3V3_ON(1);

	//while(!V3V3_PGOOD());

	//weird -- long interval between turning on V3V3 and V5 doesn't allow COM Express to boot
	block_ms(100);

	V5_ON(1);

	while(SUS_S5() | SUS_S3())
	{
		i++;
		if(i > 5) return 0;
		handle_watchdogs();
		block_ms(100);
	}
	i=0;
	handle_watchdogs();
	block_ms(100);

	send_lcd_string("Boot 1  \r\n",10);

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
		handle_watchdogs();
		block_ms(100);

	}

	send_lcd_string("Boot 2  \r\n",10);
	i=0;

	COM_EXPRESS_PGOOD_ON(1);

	while(!SUS_S3())
	{
		i++;
		if(i > 20) return 0;
		handle_watchdogs();
		block_ms(100);
	}

	blink_led(6,500);

	return 1;


}

void DeviceCarrierProcessIO()
{

	static unsigned int i = 0;

	i++;

  usb_timeout_counter++;
  
  //see if a reset is required for any reason
  handle_reset();

/*
	//if computer has shut down, flash white LED forever
	#ifndef NO_COMPUTER_INSTALLED
	if( (SUS_S3()==0) && (SUS_S5() == 0) )
	#else
	if(0)
	#endif
	{

		block_ms(20);
		send_lcd_string("Computer shutdown detected  \r\n",30);

		while(1)
		{
			set_led_brightness(WHITE_LED, 50);
	
			for(i=0;i<10;i++)
			{
				block_ms(50);
				handle_watchdogs();
				if(POWER_BUTTON())
				{
					while(POWER_BUTTON())
					{
						handle_watchdogs();
					}
					{__asm__ volatile ("reset");}
				}
			}

			if(displayed_overtemp_flag == 0)
			{
				//displayed_overtemp_flag = 1;
				if(NC_THERM_TRIP()==0)
				{
					send_lcd_string("Computer overheat detected  \r\n",30);
          REG_ROBOT_RESET_CODE = RST_SHUTDOWN_OVERHEAT;
          hard_reset_robot();
          break;
				}
				else
				{
					send_lcd_string("No computer overheat detected  \r\n",33);
          REG_ROBOT_RESET_CODE = RST_SHUTDOWN;
          hard_reset_robot();
          break;
				}
			}
	
			set_led_brightness(WHITE_LED, 0);
	
			for(i=0;i<10;i++)
			{
				block_ms(50);
				handle_watchdogs();
				if(POWER_BUTTON())
				{
					while(POWER_BUTTON())
					{
						handle_watchdogs();
					}
					{__asm__ volatile ("reset");}
				}
			}
		}
	}*/



	if(i>10)
	{
		i=0;
		DeviceCarrierGetTelemetry();
		set_led_brightness(WHITE_LED, REG_WHITE_LED);
		set_led_brightness(IR_LED, REG_IR_LED);
		update_audio_power_state();

	}

	if(REAR_PL_PRESENT() == 0)
	{
		REAR_PL_PWR_ON(1);
	}
	else
	{
		REAR_PL_PWR_ON(0);
	}

	block_ms(10);
	print_loop_number();

}

void update_audio_power_state(void)
{
	if(REG_CARRIER_SPEAKER_ON)
	{
		AMP_PWR_ON(1);
	}
	else
	{
		AMP_PWR_ON(0);
	}

	if(REG_CARRIER_MIC_ON)
	{
		MIC_PWR_ON(1);
	}
	else
	{
		MIC_PWR_ON(0);
	}

}

void handle_watchdogs(void)
{

	static unsigned char previous_wdt_pin_state = 0;


	//toggle pin to hardware WDT
	if (previous_wdt_pin_state)
	{
		WDT_PIN() = 0;
		previous_wdt_pin_state = 0;
	}
	else
	{
		WDT_PIN() = 1;
		previous_wdt_pin_state = 1;
	}

	//clear PIC's WDT
	ClrWdt();


}
void toggle_wdt_loop(void)
{
	while(1)
	{
		handle_watchdogs();
		block_ms(10);
	}

}

//reset the entire robot, without resetting the carrier board PIC
void hard_reset_robot(void)
{
  unsigned int i;

  number_of_resets++;
  usb_timeout_counter = 0;
  first_usb_message_received = 0;

  //turn on white LED so we know the this was a planned reset
  set_led_brightness(WHITE_LED, 50);
  set_led_brightness(IR_LED, 0);


	_U2RXIE = 0;
  _ADON = 0;
  _U1TXIE = 0;
  U1STAbits.UTXEN = 0;

  I2C1CON = 0x0000;
  I2C1STAT = 0x0000;
  U1MODE = 0x0000;
  U1STA = 0x0000;
  U2MODE = 0x0000;
  U2STA = 0x0000;

  //disable/enable USB?
  

  //reenable WDT, so PIC stays up
  WDT_PIN_EN(1);

  //explicitly turn off all supplies
  V3V3_ON(0);
  V5_ON(0);
  V12_ON(0);
  COM_EXPRESS_PGOOD_ON(0);

  //wait some time for power supplies to decay
  for(i=0;i<5;i++)
  {
    handle_watchdogs();
    block_ms(50);
  }

  //set all pins to default state (inputs)
  de_init_io();

  //wait some more time
  for(i=0;i<5;i++)
  {
    handle_watchdogs();
    block_ms(50);
  }

  DeviceCarrierInit();

}

//if computer has shut down, or software has requested
//a reset, set correct reset code
//and reset robot
static void handle_reset(void)
{

  //counter so that we can wait after shutdown is detected
  //for a certain number of cycles, before we restart robot
  static unsigned int computer_shutdown_counter = 0;

  //check if software has requested a reset
  //(0 through ff are invalid values)
  if(REG_ROBOT_RESET_REQUEST > 0xff)
  {
    REG_ROBOT_RESET_CODE = REG_ROBOT_RESET_REQUEST;
    display_int("RST_CODE:         \r\n", REG_ROBOT_RESET_CODE);
    REG_ROBOT_RESET_REQUEST = 0;

    //blink 10 times if wifi module never detected
    if(REG_ROBOT_RESET_CODE == 0x0101)
    {
       blink_led(10,300);
    }
    else
    {
      blink_led(2,300);
    }
    hard_reset_robot();
    return;
  }


  //if the software hasn't started in a reasonable amount of time
  //(evidenced by the lack of new USB messages), restart robot.
  //This is only done after the robot has restarted, so that we are
  //able to debug or push software on a clean boot.
  //Also, if software starts and then stops, no reset will occur, since
  //first_usb_message_received will be true
  if( (usb_timeout_counter > 2000) && (number_of_resets > 0) && (first_usb_message_received == 0) )
  {
    REG_ROBOT_RESET_CODE = RST_SOFTWARE_FAIL;
    blink_led(4,300);
    hard_reset_robot();
    return;
  }


	//if computer has shut down, trigger a hard reset (on everything but the PIC)
	#ifndef NO_COMPUTER_INSTALLED
	if( (SUS_S3()==0) && (SUS_S5() == 0) )
	#else
	if(0)
	#endif
	{
    computer_shutdown_counter++;
    //if computer has been shut down for several cycles
    if(computer_shutdown_counter > 10)
    {
      computer_shutdown_counter = 0;
      if(NC_THERM_TRIP()==0)
      {
      	send_lcd_string("Computer overheat detected  \r\n",30);
        REG_ROBOT_RESET_CODE = RST_SHUTDOWN_OVERHEAT;
      }
      else
      {
      	send_lcd_string("No computer overheat detected  \r\n",33);
        REG_ROBOT_RESET_CODE = RST_SHUTDOWN;
      }
      //blink four times so we know that the COM Express has shut down
      blink_led(4,300);
      hard_reset_robot();
   }
 }   

}
