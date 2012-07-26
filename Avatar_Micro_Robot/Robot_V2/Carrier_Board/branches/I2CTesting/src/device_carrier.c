/*******************************************************************************
File: device_carrier.c
*******************************************************************************/
#define NO_COMPUTER_INSTALLED
/*---------------------------Dependencies-------------------------------------*/
#include "./I2C_new.h"
#include "device_carrier.h"
#include "stdhdr.h"
#include "testing.h"

/*---------------------------Macros-------------------------------------------*/
#define V3V3_ON(a)              (_LATC14 = (a))
#define V3V3_EN(a)              (_TRISC14 = !(a))

#define V5_ON(a)                (_LATG8 = (a))
#define V5_EN(a)                (_TRISG8 = !(a))

#define V12_ON(a)               (_LATE6 = !(a)) // inverse logic
#define V12_EN(a)               (_TRISE6 = !(a))

#define VBAT_DIGI_EN(a)			    (_TRISB11 = !(a))
#define VBAT_DIGI_ON(a)			    (_LATB11 = (a))

#define V5_PGOOD()			  	    _RD6
#define V12_PGOOD()				      _RD2

#define V3V3_PGOOD()			      _RD1	

#define AMP_PWR_EN(a)			      (_TRISD7 = !(a))
#define AMP_PWR_ON(a)			      (_LATD7 = (a))

#define COM_EXPRESS_PGOOD_EN(a)	(_TRISB10 = !(a))
#define COM_EXPRESS_PGOOD_ON(a)	(_LATB10 = (a))

#define MIC_PWR_EN(a)			      (_TRISG6 = !(a))
#define MIC_PWR_ON(a)			      _LATG6 = a

#define CODEC_PWR_EN(a)			    (_TRISG7 = !(a))
#define CODEC_PWR_ON(a)		  	  (_LATG7 = (a))


#define SUS_S5()					      _RB5
#define SUS_S3()					      _RE5

#define HUMIDITY_SENSOR_CH       1 
#define HUMIDITY_SENSOR_EN(a)	  (_PCFG1 = !(a))

#define ADC_REF_VOLTAGE         3.3f
#define ADC_SAMPLE_COUNT        1024

#define WHITE_LED_OR			      _RP25R
#define IR_LED_OR				        _RP22R

#define POWER_BUTTON()			    !_RB4

#define GPS_TX_OR				        _RP12R
#define GPS_RX_PIN				      11

#define NC_THERM_TRIP()			    _RE7

#define WDT_PIN_EN(a)			      (_TRISF1 = !(a))
#define WDT_PIN()				        _LATF1

#define REAR_PL_PWR_EN(a)		    (_TRISB12 = !(a))
#define REAR_PL_PWR_ON(a)		    _LATB12 = a

#define REAR_PL_PRESENT()		    _RB8

// I2C slave addresses
#define ADXL345_ADDRESS         0x53
#define TMP112_0_ADDRESS        0x48
#define TMP112_1_ADDRESS        0x49
#define HMC5843_ADDRESS         0x1E
#define EEPROM_ADDRESS          0x50 // to 0x57 (0x50 to 0x53 address four 256-byte blocks
#define FAN_CONTROLLER_ADDRESS  0x18

#define RST_POWER_ON            0x0000
#define RST_SHUTDOWN            0x0001
#define RST_SHUTDOWN_OVERHEAT   0x0002
#define RST_SHUTDOWN_USB_TIMEOUT  0x0003
#define RST_SOFTWARE_FAIL       0x0004

#define IR_LED		              0
#define WHITE_LED	              1

/*---------------------------Type Definitions---------------------------------*/
// state machine states
typedef enum {
	CARRIER_INIT = 0,      // TRANSITIONAL when carrier board is power cycled everything is driven off
	CARRIER_WAIT = 1,      //              wait for ext PWR button to be pressed
	CARRIER_BOOT = 2,      // TRANSITIONAL boot sequence for full power
	CARRIER_RUN  = 3,      //              running normally with full power
	CARRIER_SHUTDOWN = 4,  // TRANSITIONAL go back into all off mode
	CARRIER_SUSPEND = 5,   // TRANSITIONAL S3 suspend to RAM with wake-on-lan or PWR button
	CARRIER_SUSPENDED = 6  //              running suspended
} CARRIER_STATE_ENUM;

/*---------------------------Helper Function Prototypes-----------------------*/
static void set_led_brightness(unsigned char led_type, unsigned char duty_cycle);
static int DeviceCarrierBoot(void);

static void robot_gps_isr(void);
static void robot_gps_init(void);

static void init_io(void);
static void de_init_io(void);
static void blink_led(unsigned int n, unsigned int ms);
static void init_pwm(void);
static void init_fan(void);
static void update_audio_power_state(void);
static void hard_reset_robot(void);
static void handle_reset(void);

static void initI2C(void);
static void DeviceCarrierGetTelemetry(void);
static void read_EEPROM_string(void);


/*---------------------------Module Variables---------------------------------*/
static unsigned int number_of_resets = 0;
unsigned int usb_timeout_counter = 0;
unsigned char first_usb_message_received = 0;

int gRegCarrierState;

typedef struct {
  float magneticX;
  float magneticY;
  float magneticZ;
  float accelerometerX;
  float accelerometerY;
  float accelerometerZ;
  float temperature;
} dummy_t;

extern dummy_t debuggingOutputs = {0, 0, 0, 0, 0, 0, 0};
extern int dummyData = 0; // TODO: delete this after testing!!!


/*---------------------------Public Function Definitions----------------------*/
#pragma code
void DeviceCarrierInit(void) {
  if (number_of_resets == 0) REG_ROBOT_RESET_CODE = RST_POWER_ON;

	de_init_io();

	//enable software watchdog if power button is not held down.  Otherwise, sleep forever.
  if (POWER_BUTTON()) {
    WDT_PIN_EN(1);
    while (1) {
      WDT_PIN() = 0;
      Sleep();
      WDT_PIN() = 1;
      //loop, so that WDT_PIN stays high at least 1us
      int i = 0;
      for (i = 0; i < 50; i++) Nop();
      ClrWdt();
    }
	}

	WDT_PIN_EN(1);
	handle_watchdogs();

	//wait some time to stabilize voltages
	block_ms(50);
	init_io();

	//turn on rear payload power (if necessary) now,
	//in case we need power to a keyboard to control the BIOS
	if (REAR_PL_PRESENT() == 0) REAR_PL_PWR_ON(1);
	else REAR_PL_PWR_ON(0);

	WDT_PIN_EN(1);

	const unsigned char build_date[12] = __DATE__; 
	const unsigned char build_time[12] = __TIME__;
  int i = 0;
	for (i = 0; i < 12; i++) {
		if ((build_date[i] == 0)) REG_ROBOT_FIRMWARE_BUILD.data[i] = ' ';
		else REG_ROBOT_FIRMWARE_BUILD.data[i] = build_date[i];
		
		if (build_time[i] == 0) REG_ROBOT_FIRMWARE_BUILD.data[i+12] = ' ';
		else REG_ROBOT_FIRMWARE_BUILD.data[i+12] = build_time[i];
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
	while (1) {
	  if (DeviceCarrierBoot() == 0) {
		  send_lcd_string("Computer boot failed  ",22);
			blink_led(3,2000);
			{__asm__ volatile ("reset");};
		} else {
		  break;
		}			
	}
	#endif
	
	send_lcd_string("Computer booted  \r\n",19);

	block_ms(100);
	CODEC_PWR_ON(1);
	handle_watchdogs();
  
  /*
	initI2C();  // TODO: undo when done

	block_ms(100);
	handle_watchdogs();

	writeI2CReg( HMC5843_ADDRESS, 0x02,0x00);
	block_ms(5);
	writeI2CReg( ADXL345_ADDRESS, 0x2d,0x08);	
	block_ms(5);
	writeI2CReg( ADXL345_ADDRESS, 0x31,0x0b);	
	block_ms(5);
  */

	// enable A/D Converter
	_SSRC = 0x07; // auto-convert
	_SAMC = 0x1f; // holding (enable this to sample)
	_ADON = 1;

	//init_fan(); // TODO: undo when done

	U2RXInterruptUserFunction = robot_gps_isr;

	robot_gps_init();
	_U2RXIE = 1;
	read_EEPROM_string();
	  
  //---I2C testing
  // TODO: remove when done
  #define TEMP_SENSOR_ADDRESS   0x48
  I2C_Init(kBaudRate100kHz);
  while (!I2C_IsBusIdle()) {};
  I2C_RequestData(TEMP_SENSOR_ADDRESS);     
  //---end I2C testing
	
	display_board_number();

	REG_CARRIER_SPEAKER_ON = 1;
	REG_CARRIER_MIC_ON = 1;

 
  
	send_lcd_string("Init finished  \r\n",17);
}


void DeviceCarrierProcessIO(void) {
	static unsigned int i = 0;
	i++;
  usb_timeout_counter++;
  handle_reset();   //see if a reset is required for any reason

	if (10 < i) {
		i = 0;
		//DeviceCarrierGetTelemetry();
		set_led_brightness(WHITE_LED, REG_WHITE_LED);
		set_led_brightness(IR_LED, REG_IR_LED);
		update_audio_power_state();
	
	  //---I2C testing
	  static int blah = 0;
	  if (I2C_IsNewDataAvailable(TEMP_SENSOR_ADDRESS) && I2C_IsBusIdle()) {
  	  dummyData = I2C_GetData(TEMP_SENSOR_ADDRESS);
  	  blah++;
  	  I2C_RequestData(TEMP_SENSOR_ADDRESS);
  	}
  	//---end I2C testing
	}

	if (REAR_PL_PRESENT() == 0) REAR_PL_PWR_ON(1);
	else REAR_PL_PWR_ON(0);

	block_ms(10);
	print_loop_number();
}


void handle_watchdogs(void) {
	static unsigned char previous_wdt_pin_state = 0;

	//toggle pin to hardware WDT
	if (previous_wdt_pin_state) {
		WDT_PIN() = 0;
		previous_wdt_pin_state = 0;
	} else {
		WDT_PIN() = 1;
		previous_wdt_pin_state = 1;
	}

	ClrWdt();
}

/*---------------------------Interrupt Service Routines-----------------------*/
static void robot_gps_isr(void) {
	static unsigned char gps_message_state = 0;
	static unsigned char gps_message_temp[100];

	//filter for messages starting with $GPGGA
	unsigned char message_filter[6] = {'$','G','P','G','G','A'};
	unsigned char i;
	unsigned char new_byte;
	U2STAbits.OERR = 0;
	_U2RXIF = 0;

	new_byte = U2RXREG;
	
	if (gps_message_state == 0) {
		if (new_byte == '$') {
			gps_message_temp[gps_message_state] = new_byte;
			gps_message_state++;
		}
	} else {
		gps_message_temp[gps_message_state] = new_byte;
		if (gps_message_state == 5) {
			for (i = 0; i < 5; i++) {
				if (gps_message_temp[i] != message_filter[i]) {
					gps_message_state = 0;
				}
			}
		}

		if (95 <= gps_message_state) {
			for (i = 0; i < 95; i++) {
				REG_ROBOT_GPS_MESSAGE.data[i] = gps_message_temp[i];
			}
			gps_message_state = 0;
      return;
		}
		gps_message_state++;
	}
}

/*---------------------------Private Function Definitions---------------------*/
/*---------------------------I2C-Related--------------------------------------*/
static void initI2C(void) {
  /*
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
  */
}

//reads PCB information from the EEPROM.  This is pretty inefficient, but it should only run once, while the COM Express
//is booting.
static void read_EEPROM_string(void) {
  /*
	unsigned int i;
	for (i = 0; i < 78; i++) {
		handle_watchdogs();
		REG_ROBOT_BOARD_DATA.data[i] = readI2C_Reg(EEPROM_ADDRESS,i);
		block_ms(5);
	}
	*/
}
/*---------------------------end I2C-Related--------------------------------------*/
static int DeviceCarrierReadAdxl345Register(unsigned char add, unsigned char reg) {
	/*
	int a,b;
	int c;
	a = readI2C_Reg(add,reg);
	b = readI2C_Reg(add,reg+1);
	c = a | (b << 8);
	return c;
	*/
	return 0;
}


static int DeviceCarrierReadTmp112Register(unsigned char add, unsigned char reg) {
	/*
	int a, b, c;
	a = readI2C_Reg(add,reg);
	b = readI2C_Reg(add,reg+1);
	c = (b >> 4) | (a << 4);
	return c;
	*/
	return 0;
}


static int DeviceCarrierReadHmc5843Register(unsigned char add, unsigned char reg) {
	/*
	unsigned char a, b;
	int c;
	a = readI2C_Reg(add,reg);
	b = readI2C_Reg(add,reg+1);
	c = b | (a << 8);
	return c;
	*/
	return 0;
}


static void init_fan(void) {
  /*
	handle_watchdogs();

	//set fan configuration
	writeI2CReg(FAN_CONTROLLER_ADDRESS, 0x02, 0b00011010);
	block_ms(5);

  // For thermistor control
	//make thermistor 1 control fan 2, and vice versa
	writeI2CReg(FAN_CONTROLLER_ADDRESS, 0x11, 0b00011000);
	block_ms(5);

	//set fan start duty cycle -> 120/240 = 50%
	writeI2CReg(FAN_CONTROLLER_ADDRESS, 0x07, 120);
	block_ms(5);

	//first, set fan to turn on at 15C
	writeI2CReg(FAN_CONTROLLER_ADDRESS, 0x10, 15);

	block_ms(5);

	//wait a while
	int i;
	for (i = 0; i < 50; i++) {
		handle_watchdogs();
		block_ms(20);
	}

	//fan turns on at 40C
	writeI2CReg(FAN_CONTROLLER_ADDRESS, 0x10, 40);
	block_ms(5);
	
	handle_watchdogs();
	*/
}


static void blink_led(unsigned int n, unsigned int ms) {
	unsigned int i,j, max_j;
	
	handle_watchdogs();
	max_j = ms/20;

	for (i = 0; i<n; i++) {
		set_led_brightness(WHITE_LED,50);
		for (j = 0;j < max_j; j++) {
			block_ms(10);
			handle_watchdogs();
		}
		set_led_brightness(WHITE_LED,0);
		for (j = 0; j < max_j; j++) {
			block_ms(10);
			handle_watchdogs();
		}
	}
}


static void DeviceCarrierGetTelemetry() {
  /*
	unsigned char a, b;
	signed int c;
	int d;

 	IFS0bits.T1IF = 0;	//clear interrupt flag ??

	a = DeviceCarrierReadAdxl345Register(ADXL345_ADDRESS, 0x32); // Accel X-Axis Data 0
	b = DeviceCarrierReadAdxl345Register(ADXL345_ADDRESS, 0x33); // Accel X-Axis Data 1
	c = a | (b << 8);
	REG_ACCEL_X = (float)c * 16.0 / 4096.0;

	a = DeviceCarrierReadAdxl345Register(ADXL345_ADDRESS, 0x34); // Accel Y-Axis Data 0
	b = DeviceCarrierReadAdxl345Register(ADXL345_ADDRESS, 0x35); // Accel Y-Axis Data 1
	c = a | (b << 8);
	REG_ACCEL_Y = (float)c * 16.0 / 4096.0;

	a = DeviceCarrierReadAdxl345Register(ADXL345_ADDRESS, 0x36); // Accel Z-Axis Data 0
	b = DeviceCarrierReadAdxl345Register(ADXL345_ADDRESS, 0x37); // Accel Z-Axis Data 1
	c = a | (b << 8);
	REG_ACCEL_Z = -(float)c * 16.0 / 4096.0;

	d = DeviceCarrierReadTmp112Register(TMP112_0_ADDRESS, 0x00); // Temperature
	REG_TEMP_INT0 = (float)d / 16.0;
	
	REG_MAGNETIC_X = (float)DeviceCarrierReadHmc5843Register(HMC5843_ADDRESS, 0x03); // Magnet X-Axis Data
	REG_MAGNETIC_Y = (float)DeviceCarrierReadHmc5843Register(HMC5843_ADDRESS, 0x05); // Magnet Y-Axis Data 0
	REG_MAGNETIC_Z = (float)DeviceCarrierReadHmc5843Register(HMC5843_ADDRESS, 0x07); // Magnet Z-Axis Data 0

  // monitor reads in debugger
  debuggingOutputs.magneticX = REG_MAGNETIC_X;
  debuggingOutputs.magneticY = REG_MAGNETIC_Y;
  debuggingOutputs.magneticZ = REG_MAGNETIC_Z;
  debuggingOutputs.accelerometerX = REG_ACCEL_X;
  debuggingOutputs.accelerometerY = REG_ACCEL_Y;
  debuggingOutputs.accelerometerZ = REG_ACCEL_Z;
  debuggingOutputs.temperature = REG_TEMP_INT0;

  // get an analog reading from the humidity sensor
	_ADON = 0;
	_CH0SA = HUMIDITY_SENSOR_CH;
	_ADON = 1;
	_SAMP = 1;
	
	while (!_DONE) {};

	REG_ROBOT_HUMIDITY = ADC1BUF0 * ADC_REF_VOLTAGE / ADC_SAMPLE_COUNT;

	REG_TELEMETRY_COUNT++;
	*/
}


static void init_io(void) {
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


static void de_init_io(void) {
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


static void robot_gps_init(void) {
	//set GPS tx to U2TX
	GPS_TX_OR = 5;
	//set U2RX to GPS rx pin
	_U2RXR = GPS_RX_PIN;	
	U2BRG = 103;
	_U2RXIE = 0;
	U2MODEbits.UARTEN = 1;
}

static void update_audio_power_state(void) {
	if (REG_CARRIER_SPEAKER_ON) AMP_PWR_ON(1);
	else AMP_PWR_ON(0);

	if (REG_CARRIER_MIC_ON) MIC_PWR_ON(1);
	else MIC_PWR_ON(0);
}


//reset the entire robot, without resetting the carrier board PIC
static void hard_reset_robot(void) {
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

  //I2C1CON = 0x0000; // TODO: undo when done
  //I2C1STAT = 0x0000;
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
  unsigned int i;
  for (i = 0; i < 5; i++) {
    handle_watchdogs();
    block_ms(50);
  }

  //set all pins to default state (inputs)
  de_init_io();

  //wait some more time
  for (i = 0; i < 5; i++) {
    handle_watchdogs();
    block_ms(50);
  }

  DeviceCarrierInit();
}


//if computer has shut down, or software has requested
//a reset, set correct reset code
//and reset robot
static void handle_reset(void) {
  //counter so that we can wait after shutdown is detected
  //for a certain number of cycles, before we restart robot
  static unsigned int computer_shutdown_counter = 0;

  //check if software has requested a reset
  //(0 through ff are invalid values)
  if (0xff < REG_ROBOT_RESET_REQUEST) {
    REG_ROBOT_RESET_CODE = REG_ROBOT_RESET_REQUEST;
    display_int("RST_CODE:         \r\n", REG_ROBOT_RESET_CODE);
    REG_ROBOT_RESET_REQUEST = 0;

    //blink 10 times if wifi module never detected
    if (REG_ROBOT_RESET_CODE == 0x0101) blink_led(10,300);
    else blink_led(2,300);
    
    hard_reset_robot();
    return;
  }


  //if the software hasn't started in a reasonable amount of time
  //(evidenced by the lack of new USB messages), restart robot.
  //This is only done after the robot has restarted, so that we are
  //able to debug or push software on a clean boot.
  //Also, if software starts and then stops, no reset will occur, since
  //first_usb_message_received will be true
  if ((2000 < usb_timeout_counter) && 
      (0 < number_of_resets) && 
      (first_usb_message_received == 0)) {
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


static void init_pwm(void) {
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
	
	T2CONbits.TON = 1;
}


static void set_led_brightness(unsigned char led_type, unsigned char duty_cycle) {
	if (100 < duty_cycle) duty_cycle = 100;

	switch (led_type) {
		case WHITE_LED:
			OC1R = 39990 - duty_cycle*399;
		  break;
		case IR_LED:
			OC2R = 39990 - duty_cycle*399;
		  break;
	}
}


static int DeviceCarrierBoot(void) {
	unsigned int i = 0;

	V3V3_ON(1);

	//while(!V3V3_PGOOD());

	//weird -- long interval between turning on V3V3 and V5 doesn't allow COM Express to boot
	block_ms(100);

	V5_ON(1);

	while(SUS_S5() | SUS_S3()) {
		i++;
		if (5 < i) return 0;
		handle_watchdogs();
		block_ms(100);
	}
	i = 0;
	handle_watchdogs();
	block_ms(100);

	send_lcd_string("Boot 1  \r\n",10);

	while (!SUS_S5()) {
		i++;
		if (20 < i) return 0;
		handle_watchdogs();
		block_ms(100);
	}

	send_lcd_string("Boot 2  \r\n",10);
	i = 0;

	COM_EXPRESS_PGOOD_ON(1);

	while (!SUS_S3()) {
		i++;
		if (20 < i) return 0;
		handle_watchdogs();
		block_ms(100);
	}

	blink_led(6,500);

	return 1;
}
