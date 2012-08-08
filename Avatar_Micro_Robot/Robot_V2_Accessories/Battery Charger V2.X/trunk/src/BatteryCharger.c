/*******************************************************************************
File: BatteryCharger.c

Description: Overarching file encompassing the application-level logic of the 
  battery charger (external to the robot).
*******************************************************************************/
/*---------------------------Dependencies-------------------------------------*/
#include "./StandardHeader.h"
#include "./Timers.h"
#include "./ADC.h"          // for analog sensing of current and voltage
#include "./TWI.h"
#include "./SMBusDeviceHeaders/BQ242745.h"

/*---------------------------Macros-------------------------------------------*/
// wiring macros
//---battery charger
#define ENABLE_5V(a)        (_TRISB12 = (a))
#define TURN_5V(a)          (_RB12 = (a))

#define ENABLE_CHARGE_A(a)  (_TRISB10 = (a))
#define TURN_CHARGE_A(a)    (_RB10 = (a))

#define ENABLE_CHARGE_B(a)  (_TRISB11 = (a))
#define TURN_CHARGE_B(a)    (_RB11 = (a))

#define ENABLE_CONNECT_BATTERY(a) (_TRISB9 = (a))
#define CONNECT_BATTERY(a)  (_RB9 = (a))

#define ENABLE_BQ242745_EN  (_TRISB13 = (a))
#define TURN_BQ242745(a)    (_RB13 = (a))


//---indicators
#define ENABLE_HEARTBEAT(a) (_TRISE5 = (a))
#define HEARTBEAT_PIN       (_RE5)
#define ENABLE_RED_LED(a)
#define TURN_RED_LED(a)
#define ENABLE_GREEN_LED(a)
#define TURN_GREEN_LED(a)

// timers
#define HEARTBEAT_TIMER     0
#define HEARTBEAT_TIME      500
#define SMB1_TIMEOUT_TIMER  1
#define SMB1_TIMEOUT_TIME   10
#define SMB2_TIMEOUT_TIMER  2
#define SMB2_TIMEOUT_TIME   10
#define SMB3_TIMEOUT_TIMER  3
#define SMB3_TIMEOUT_TIME   10

/*---------------------------Type Definitions---------------------------------*/
typedef enum {
  kWaiting = 0,
  kCharging
} ChargerState;

/*---------------------------Helper Function Prototypes-----------------------*/
static void InitPins(void);
static void DeinitPins(void);

static void InitBattery(void);
static void UpdateSMBusSensors(void);
static inline float DecodeCellData(TWIDevice *device);

/*---------------------------Module Variables---------------------------------*/

static unsigned char dater2CZ[2] = {0};
static I2CDevice temperatureSensor = {
  .address = TMP112_0_ADDRESS,
  .subaddress = 0x00, // subaddress which contains the sensor data
  .numDataBytes = 2,  // number of data bytes the sensor will send us
  .data = dater2temp
};

/*---------------------------Public Function Definitions----------------------*/
void InitBatteryCharger(void) {
  DeinitPins();
	InitPins();

  InitTimers();
  ADC_Init((1 << HUMIDITY_SENSOR_CH));
  TWI_Init(kBaudRate100kHz);
  InitBattery();
  
  StartTimer(SMB1_TIMEOUT_TIMER, SMB1_TIMEOUT_TIME);
  StartTimer(SMB2_TIMEOUT_TIMER, SMB2_TIMEOUT_TIME);
  StartTimer(SMB3_TIMEOUT_TIMER, SMB3_TIMEOUT_TIME);  
}


void DeviceCarrierProcessIO(void) {
	static unsigned int i = 0;
	i++;
  usb_timeout_counter++;
  handle_reset();   // see if a reset is required for any reason

	if (10 < i) {
		i = 0;
		PWM_UpdateDutyCycle(WHITE_LED_RPN, REG_WHITE_LED);
    PWM_UpdateDutyCycle(IR_LED_RPN, REG_IR_LED);
		update_audio_power_state();
	}

	if (REAR_PL_PRESENT() == 0) REAR_PL_PWR_ON(1);
	else REAR_PL_PWR_ON(0);

  UpdateSensors();
  
  PrintLoopNumber();
}


static void UpdateSensors(void) {
  StartTimer(I2C_TIMEOUT_TIMER, I2C_TIMEOUT_TIME);
    
  // if an error has occurred, restart the I2C module
  if (I2C_ErrorHasOccurred() || IsTimerExpired(I2C_TIMEOUT_TIMER)) {
	  I2C_Init(kI2CBaudRate100kHz);
    I2C_RequestData(&temperatureSensor); // prime the sensor update loop
  }
  
  if (I2C_IsBusIdle() && IsTimerExpired(I2C_UPDATE_TIMER)) {
    StartTimer(I2C_UPDATE_TIMER, I2C_UPDATE_TIME);
	  UpdateI2CSensors();
	}
	
	// populate the shared registers with the latest values
  REG_TEMP_INT0 = DecodeTemperatureData(&temperatureSensor);
  REG_ACCEL_X = DecodeAccelerometerData(&accelerometerX);
  REG_ACCEL_Y = DecodeAccelerometerData(&accelerometerY);
  REG_ACCEL_Z = DecodeAccelerometerData(&accelerometerZ);
  REG_MAGNETIC_X = DecodeMagnetometerData(&magnetometerX);
  REG_MAGNETIC_Y = DecodeMagnetometerData(&magnetometerY);
  REG_MAGNETIC_Z = DecodeMagnetometerData(&magnetometerZ);
	REG_ROBOT_HUMIDITY = ADC_GetConversion(HUMIDITY_SENSOR_CH);
}  


static void UpdateI2CSensors(void) {
  #define NUM_I2C_SENSORS     7
  static unsigned char currentSensor = 0;

  // get your data, then request it for the next guy
  switch (currentSensor) {
    case 0:
      I2C_GetData(&temperatureSensor);
      I2C_RequestData(&accelerometerX);
      break;
    case 1:
      I2C_GetData(&accelerometerX);
      I2C_RequestData(&accelerometerY);
      break;
    case 2:
      I2C_GetData(&accelerometerY);
      I2C_RequestData(&accelerometerZ);
      break;
    case 3:
      I2C_GetData(&accelerometerZ);
      I2C_RequestData(&magnetometerX);
      // 67 milli-second typical delay should be allowed by the I2C master
  	  // before querying the HMC5883L data registers for new measurements
      break;
    case 4:
      I2C_GetData(&magnetometerX);
      I2C_RequestData(&magnetometerY);      
      break;
    case 5:
      I2C_GetData(&magnetometerY);
      I2C_RequestData(&magnetometerZ);
      break;
    case 6:
      I2C_GetData(&magnetometerZ);
      I2C_RequestData(&temperatureSensor);
      break;
    /*
    case 3:
      I2C_GetData(&accelerometerZ);
      I2C_RequestData(&temperatureSensor);
      break;
    */
  }
  
  // advance to sample the next sensor, considering rollover
  if (NUM_I2C_SENSORS <= ++currentSensor) currentSensor = 0;
}

  
void handle_watchdogs(void) {
	static unsigned char previous_wdt_pin_state = 0;

	// toggle the pin to hardware WDT
	if (previous_wdt_pin_state) {
		WDT_PIN() = 0;
		previous_wdt_pin_state = 0;
	} else {
		WDT_PIN() = 1;
		previous_wdt_pin_state = 1;
	}

	ClrWdt(); // clear the software WDT
}


/*---------------------------Private Function Definitions---------------------*/
static void robot_gps_isr(void) {
	static unsigned char gps_message_state = 0;
	static unsigned char gps_message_temp[100];

	//filter for messages starting with $GPGGA
	unsigned char message_filter[6] = {'$', 'G', 'P', 'G', 'G', 'A'};
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


static inline float DecodeAccelerometerData(I2CDevice *device) {
  //const float kScalingDivisor = 256;  // = G-range / numResolutionBits
                                      // = 16 / 2^12 => 1 / 256
  // Note: it's important to first multiply so as to lose less resolution
  
  // for ADXL345
  int placeholder = (device->data[0] << 8) | (device->data[1]);
   
  return ((float)placeholder) * 16.0 / 4096.0;
}


static inline float DecodeTemperatureData(I2CDevice *device) {
  #define COUNTS_PER_DEGREE_C   16.0
  // for TMP112 (see p.8 of datasheet)
  unsigned int placeholder = device->data[1];
  placeholder = (placeholder << 4) | (device->data[0] >> 4);
  
  // if the MSB is high (negative number), take the two's complement
  if (placeholder & (1 << 12)) placeholder = ~placeholder + 1;
  
  return (placeholder / COUNTS_PER_DEGREE_C);
}  


static inline float DecodeMagnetometerData(I2CDevice *device) {
  // for HMC5843
	return ((device->data[0] << 8) | device->data[1]);
}


/*
Description: Reads the PCB information from the EEPROM.  It should
  only run once while the OCM Express is booting.
  TODO: test this with an actual EEPROM
*/
static void ReadEEPROM(void) {
  // TODO: what is 78?
  #define NUM_EEPROM_BYTES    78
  unsigned char buffer[1] = {0};
  
  I2CDevice eeprom;
  eeprom.address = EEPROM_ADDRESS;
  eeprom.numDataBytes = 1;
  eeprom.data = buffer;
	unsigned int i;
	for (i = 0; i < NUM_EEPROM_BYTES; i++) {
		handle_watchdogs(); Delay(10);
		eeprom.subaddress = i;
		I2C_RequestData(&eeprom); handle_watchdogs(); Delay(10);
		// assume that 10ms is long enough and that no errors occurr
		I2C_GetData(&eeprom);
		REG_ROBOT_BOARD_DATA.data[i] = eeprom.data[0]; 
	}
}


static void InitI2CDevices(void) {
	InitFan();
  InitTemperatureSensor();
  InitAccelerometer();
  InitMagnetometer();
  I2C_RequestData(&temperatureSensor); // prime the sensor update loop
}
  

static void InitAccelerometer(void) {
  handle_watchdogs();
  
  // TODO: populate the fields of the static variable(s)?
  I2CDevice *device;
  device->address = ADXL345_ADDRESS;
  device->subaddress = ADXL345_POWER_CTL;
  unsigned char data[1] = {0};
  data[0] = (1 << ADXL345_MEASURE_BIT);
  I2C_WriteData(device, data); Delay(5); handle_watchdogs();
  
  device->subaddress = ADXL345_DATA_FORMAT;
  data[0] = (1 << ADXL345_FULL_RES_BIT) | (1 << ADXL345_RANGE_BIT1) | (1 << ADXL345_RANGE_BIT0);
  I2C_WriteData(device, data); Delay(5); handle_watchdogs();
}


static void InitMagnetometer(void) {
  handle_watchdogs();
  
  I2CDevice device;
  device.address = HMC5843_ADDRESS;
  device.subaddress = HMC5843_MODE;
  unsigned char data[1] = {0};
  data[0] = 0x00; // continuous-conversion mode
  I2C_WriteData(&device, data); Delay(5); handle_watchdogs();
}


static void InitTemperatureSensor(void) {
  // TODO: implement
}


static void InitFan(void) {
	handle_watchdogs();

	// set fan configuration
	unsigned char data[1] = {0b00011010};
	fan.subaddress = 0x02;
	I2C_WriteData(&fan, data); Delay(5); handle_watchdogs();
	
	// set the duty cycle rate-of-change to be instant for fan 2
	// TODO: remove when done
	fan.subaddress = 0x12;
	data[1] = 0b00011100;
	I2C_WriteData(&fan, data); Delay(5); handle_watchdogs();
	
  // make fan 2 start as a function of thermistor 1 data
  data[0] = 0b00011000;
  fan.subaddress = 0x11;
	I2C_WriteData(&fan, data); Delay(5); handle_watchdogs();
	
	// set fan start duty cycle = 120 / 240 => 50%
	data[0] = 120;
	fan.subaddress = 0x07;
	I2C_WriteData(&fan, data); Delay(5); handle_watchdogs();
	
	// set fan to turn on at 15C
	data[0] = 15;
	fan.subaddress = 0x10;
	I2C_WriteData(&fan, data); Delay(5); handle_watchdogs();

	/*
  //wait a while
	unsigned char i;
	for (i = 0; i < 50; i++) {
		handle_watchdogs();
		Delay(20);
	}

	// set fan to turn on at 40C
	data[0] = 40;
	fan.subaddress = 0x10;
	I2C_WriteData(&fan, data);
	Delay(5);
	handle_watchdogs();
	*/
}


static void blink_led(unsigned int n, unsigned int ms) {
	unsigned int i, j, max_j;
	
	handle_watchdogs();
	max_j = ms / 20;

	for (i = 0; i < n; i++) {
		PWM_UpdateDutyCycle(WHITE_LED_RPN, 50);
		for (j = 0; j < max_j; j++) {
			Delay(10);
			handle_watchdogs();
		}
		PWM_UpdateDutyCycle(WHITE_LED_RPN, 0);
		for (j = 0; j < max_j; j++) {
			Delay(10);
			handle_watchdogs();
		}
	}
}


static void InitPins(void) {
	// reset the A/D module
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
	ENABLE_POWER_BUTTON(1);
	ENABLE_DRDY(1);
	REAR_PL_PWR_EN(1);
}


static void DeinitPins(void) {
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
	
	DeinitTimers();
	ADC_Deinit();
	I2C_Deinit();
	PWM_Deinit();
}


static void InitRobotGPS(void) {
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
  PWM_UpdateDutyCycle(WHITE_LED_RPN, 50);
  PWM_UpdateDutyCycle(WHITE_LED_RPN, 0);

	_U2RXIE = 0;
  _ADON = 0;
  _U1TXIE = 0;
  U1STAbits.UTXEN = 0;

  U1MODE = 0x0000;
  U1STA = 0x0000;
  U2MODE = 0x0000;
  U2STA = 0x0000;

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
    Delay(50);
  }

  //set all pins to default state (inputs)
  DeinitPins();

  //wait some more time
  for (i = 0; i < 5; i++) {
    handle_watchdogs();
    Delay(50);
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
    //display_int("RST_CODE:         \r\n", REG_ROBOT_RESET_CODE);
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
      	//send_lcd_string("Computer overheat detected  \r\n",30);
        REG_ROBOT_RESET_CODE = RST_SHUTDOWN_OVERHEAT;
      }
      else
      {
      	//send_lcd_string("No computer overheat detected  \r\n",33);
        REG_ROBOT_RESET_CODE = RST_SHUTDOWN;
      }
      //blink four times so we know that the COM Express has shut down
      blink_led(4,300);
      hard_reset_robot();
   }
 }   
}


static int DeviceCarrierBoot(void) {
	unsigned int i = 0;

	V3V3_ON(1);

	//while(!V3V3_PGOOD());

	//weird -- long interval between turning on V3V3 and V5 doesn't allow COM Express to boot
	Delay(100);

	V5_ON(1);

	while(SUS_S5() | SUS_S3()) {
		i++;
		if (5 < i) return 0;
		handle_watchdogs();
		Delay(100);
	}
	i = 0;
	handle_watchdogs();
	Delay(100);

	SendLCDString("Boot 1  \r\n",10);

	while (!SUS_S5()) {
		i++;
		if (20 < i) return 0;
		handle_watchdogs();
		Delay(100);
	}

	SendLCDString("Boot 2  \r\n",10);
	i = 0;

	COM_EXPRESS_PGOOD_ON(1);

	while (!SUS_S3()) {
		i++;
		if (20 < i) return 0;
		handle_watchdogs();
		Delay(100);
	}

	blink_led(6,500);

	return 1;
}


static void WriteBuildTime(void) {
	const unsigned char build_date[12] = __DATE__; 
	const unsigned char build_time[12] = __TIME__;
  int i = 0;
	for (i = 0; i < 12; i++) {
		if ((build_date[i] == 0)) REG_ROBOT_FIRMWARE_BUILD.data[i] = ' ';
		else REG_ROBOT_FIRMWARE_BUILD.data[i] = build_date[i];
		
		if (build_time[i] == 0) REG_ROBOT_FIRMWARE_BUILD.data[i+12] = ' ';
		else REG_ROBOT_FIRMWARE_BUILD.data[i+12] = build_time[i];
	}
}
