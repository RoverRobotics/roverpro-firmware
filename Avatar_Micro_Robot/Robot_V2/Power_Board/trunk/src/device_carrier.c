/**
 * @file device_carrier.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex carrier PIC firmware.
 *
 * DEVICE MAPPING
 * Hardware Revision E0 (Robotex Robot Carrier Board)
 *
 * ---------------------------------------------------------------------------
 * DIGITAL OUTPUTS
 * ---------------------------------------------------------------------------
 * NAME               PIN  PORT  DEFAULT  FUNCTION
 * ------------------------------
 * EN_COMXPRESS       59   RF1   0        EN 3.3V to COMEx, HW pull-down
 * EN_12VSUPPLY       6    RG8   0        EN 12V main, HW pull-down
-* V3V3               48   RC14  0        3.3V main, HW pull-down
-* V5                 32   RF5   0        5V main, HW pull-down
 * 5V_PWRENABLE       32   RF5   0        EN 5V main, HW pull-down
 * NCOM_WAKE0#        30   RB15  1        COMEx PCIE wake (pull-up on COMEx)
 * NCOM_WAKE1#        58   RF0   1        COMEx AUX Wake (pull-up on COMEx)
 * NCOM_SYSRST#       22   RB9   Z        Rst push-button on PCB, (pull-up on COMEx & PCB), don't override button
 * PWR_BTN#           42   RD8   Z        Pwr push-button on PCB, (pull-up on COMEx & PCB), don't override button
 * ------------------------------
 * EN_CAMERABRD       2    RE6   0        EN 3.3V and 12V to Camera Board, HW pull-down
 * EN_SMBPWR          4    RG6   0        EN 3.3V SMBus, HW pull-down
 * CODEC_PD#          5    RG7   1        SHTDWN# pin of TPA6211 codec
 * EN_RADIOBRD        8    RG9   0        EN 3.3V to radio, HW pull-up
 * EN_MOTORCNTL       15   RB1   0        EN 5V to Motor Controller, HW pull-down
 * EN_PAYLOAD1        14   RB2   0        EN raw power to payload #1, HW pull-down
 * EN_PAYLOAD2        47   RC13  0        EN raw power to payload #2, HW pull-down
 * PWM_LED            52   RD4   0        Straight out to Camera Board
 * PWM_IRLED          51   RD3   0        Straight Out to Camera Board
 * SMB_SCL            44   RD10  0        SMBus clock
 *
 * ---------------------------------------------------------------------------
 * DIGITAL INPUTS
 * ---------------------------------------------------------------------------
 * PAYLOAD1_PRESENT#  3    RE7   Z        indicates payload #1 connected
 * PAYLOAD2_PRESENT#  48   RC14  Z        indicates payload #2 connected
 * CAMERA_PRESENT#    1    RE5   Z        indicates camera board is connected
 * ------------------------------
 * SUS_S3#            54   RD6   Z        indicates the system is suspended to RAM
 * PIC_WAKE0#         29   RB14  Z        wake from radio PCIE
 * SMB_ALERT#         55   RD7   Z        indicates message pending on SMBus
 * NC_THRMTRIP#       50   RD2   Z        indicates thermal problem with COMEx
 * PWR_BUTTON#        12   RB4   Z        external power button
 * ------------------------------
 * RAW_PGOOD          46   RD0   Z        indicates raw power is good
 * 12V_PGOOD          31   RF4   Z        12V main good signal
 * 5V_ALWS_PGOOD      45   RD11  Z        indicates 5V is good
 * 3.3V_ALWS_PGOOD    13   RB3   Z        if not good probably not alive
 *
 * ---------------------------------------------------------------------------
 * DIGITAL I/O
 * ---------------------------------------------------------------------------
 * SMB_SDA            43   RD9   Z        SMBus data I/O
 * SECURE_PIC         53   RD5   Z        Atmel authentication chip I/O
 *
 * ---------------------------------------------------------------------------
 * ANALOG INPUTS
 * ---------------------------------------------------------------------------
 * 14V_RAW            21   AN8   Z        gain=0.194V/V
 * EX_TEMP_SNS        16   AN0   Z        pulled to 3.3V with 100k
 *
 */

#include "stdhdr.h"
#include "device_carrier.h"

#define V3V3_ON(a)               _LATC14    = a
#define V3V3_EN(a)               _TRISC14 = !a

#define V5_ON(a)                 _LATF5     = a
#define V5_EN(a)                 _TRISF5  = !a

#define V12_ON(a)                _LATG8     = !a // inverse logic
#define V12_EN(a)                _TRISG8  = !a

#define COM_EXPRESS_ON(a)        _LATF1     = a
#define COM_EXPRESS_EN(a)        _TRISF1  = !a

#define COM_EXPRESS_PG_ON(a)     _LATB10    = a // PWR_OK
#define COM_EXPRESS_PG_EN(a)     _TRISB10 = !a

#define GPS_ON(a)                _LATF4     = a
#define GPS_EN(a)                _TRISF4  = !a

#define RADIO_ON(a)              _LATG9     = a
#define RADIO_EN(a)              _TRISG9  = !a

#define MOTOR_CONTROLLER_ON(a)   _LATB13    = a
#define MOTOR_CONTROLLER_EN(a)   _TRISB13 = !a

#define ALL_PAYLOAD_POWER_ON(a)  _LATC13    = a
#define ALL_PAYLOAD_POWER_EN(a)  _TRISC13 = !a

#define CAMERA_BOARD_ON(a)       _LATE6     = a
#define CAMERA_BOARD_EN(a)       _TRISE6  = !a

#define SMBUS_ON(a)              _LATG6     = a
#define SMBUS_EN(a)              _TRISG6  = !a

#define CODEC_PD_ON(a)           _LATG7     = !a
#define CODEC_PD_EN(a)           _TRISG7  = a

#define EXT_PWR_BUTTON()         (!_RB4)

#define GPS_1PPS()               (_RB8) // input from GPS (mode)
#define GPS_LCKFIX()             (_RB3) // digital in from GPS

#define NCOM_WAKE0_ON(a)         _LATB15    = !a
#define NCOM_WAKE0_EN(a)         _TRISB15 = !a

#define NCOM_WAKE1_ON(a)         _LATF0     = !a
#define NCOM_WAKE1_EN(a)         _TRISF0  = !a

#define PIC_WAKE0()              (!_RB14)

#define NCOM_SYSRST_ON(a)        _LATB9     = !a
#define NCOM_SYSRST_EN(a)        _TRISB9  = !a

#define EXTERNAL_TEMP_SENSOR     0 // CH0 : input wth 100k pull-up (not used)
#define HUMIDITY_SENSOR          1 // CH1 : analog input (0 - 3V ?)
#define ADC_REF_VOLTAGE          3.3f
#define ADC_SAMPLE_COUNT         1024

#define SUS_S5                   (!_RB5)

#define PAYLOAD1_PWR_ON(a)       _LATB12    = a
#define PAYLOAD1_PWR_EN(a)       _TRISB12 = !a

#define PAYLOAD2_PWR_ON(a)       _LATB11    = a
#define PAYLOAD2_PWR_EN(a)       _TRISB11 = !a

// board ids

#define GPS_TXD // pin 45 (RP12)
#define GPS_RXD // pin 46 (RP11)
#define SMB_SDA // pin 43 (SDA1)
#define SMB_SCL // pin 44 (SCLK1)

#define MPLEXED_A_ON(a)          _LATD6    = a // pin 54 RD6
#define MPLEXED_A_EN(a)          _TRISD6 = !a

#define MPLEXED_B_ON(a)          _LATD2    = a // pin 50 RD2
#define MPLEXED_B_EN(a)          _TRISD2 = !a

#define MPLEXED_C_ON(a)          _LATD1    = a // pin 49 RD1
#define MPLEXED_C_EN(a)          _TRISD1 = !a

	#define V5_GOOD              0 // MPLX[0] - 5V_GOOD
	#define V12_GOOD             1 // MPLX[1] - 12V_GOOD
	#define V3V3_GOOD            2 // MPLX[2] - 3V3_ALWS_GOOD
                                   // MPLX[3] - SMBus Alert (SMB_ALERT#)
	#define NOT_CAMERA_PRESENT   4 // MPLX[4] - CAMERA_PRESENT#
	#define NOT_PAYLOAD1_PRESENT 5 // MPLX[5] - PAYLOAD1_PRESENT#
	#define NOT_PAYLOAD2_PRESENT 6 // MPLX[6] - PAYLOAD2_PRESENT#
	#define NOT_NC_THRMTRIP      7 // MPLX[7] - NC_THRMTRIP# (from COMExpress)

#define MPLEXED_INPUT()          (_RE7) // pin 3

// inverse logic from on board button (also goes to COMExpres)
#define PWR_BTN()                 (!_RD8)
#define PWR_BTN_ON(a)             _LATD8    = !a
#define PWR_BTN_EN(a)             _TRISD8 = !a

#define SECURE_PIC // pin 53

#define PWM_IRLED_ON(a)          _LATD3    = !a // digital PWM output 100% duty cycle
#define PWM_IRLED_EN(a)          _TRISD3 = !a

#define PWM_LED_ON(a)            _LATD4    = !a
#define PWM_LED_EN(a)            _TRISD4 = !a

#define SUS_S3                   (!_RE5)

#define ADXL345_ADDRESS           0x53
#define TMP112_0_ADDRESS          0x48
#define TMP112_1_ADDRESS          0x49
#define HMC5843_ADDRESS           0x1E


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
void DeviceCarrierBoot();
void initI2C( void );
void writeI2C( unsigned char add, unsigned char v );
int readI2C( unsigned char add );


// FUNCTIONS

#pragma code

void DeviceCarrierInit()
{
	// initialize global data
	REG_TELEMETRY_COUNT = 0;
	REG_PAYLOAD_1_PRESENT = 0;
	REG_PAYLOAD_2_PRESENT = 0;
	REG_CAMERA_PRESENT = 0;
	REG_RAW_POWER_GOOD = 0;
	REG_V12_POWER_GOOD = 0;
	REG_V5_POWER_GOOD = 0;
	REG_V14_VALUE = 0.0f;
	REG_ACCEL_X = 0.0f;
	REG_ACCEL_Y = 0.0f;
	REG_ACCEL_Z = 0.0f;
	REG_TEMP_INT0 = 0.0f;
	REG_TEMP_INT1 = 0.0f;
	REG_TEMP_EXT0 = 0.0f;
	//memcpy(&REG_CRYPTO.data, "ABCDEFGH", 8);

	// turn off analog input
	AD1PCFGL = 0xFFFF;

	// set outputs
	COM_EXPRESS_ON(0);
	COM_EXPRESS_PG_ON(0);
	V12_ON(0);
	V5_ON(0);
	V3V3_ON(0);
	PWR_BTN_ON(1);       // default: enabled but don't enable output
	CAMERA_BOARD_ON(0);
	SMBUS_ON(0);
	CODEC_PD_ON(0);
	RADIO_ON(0);
	MOTOR_CONTROLLER_ON(0);
	ALL_PAYLOAD_POWER_ON(0);
	PAYLOAD1_PWR_ON(0);
	PAYLOAD2_PWR_ON(0);
	PWM_IRLED_ON(0);
	PWM_LED_ON(0);

	// enable outputs
	COM_EXPRESS_EN(1);
	COM_EXPRESS_PG_EN(1);
	V12_EN(1);
	V5_EN(1);
	V3V3_EN(1);
	PWR_BTN_EN(0);       // default: enabled but don't enable output
	CAMERA_BOARD_EN(1);
	SMBUS_EN(1);
	RADIO_EN(1);
	MOTOR_CONTROLLER_EN(1);
	ALL_PAYLOAD_POWER_EN(1);
	PAYLOAD1_PWR_EN(1);
	PAYLOAD2_PWR_EN(1);
	PWM_IRLED_EN(1);
	PWM_LED_EN(1);
	MPLEXED_A_EN(1);
	MPLEXED_B_EN(1);
	MPLEXED_C_EN(1);

	gRegCarrierState = CARRIER_WAIT;

/*	initI2C();

	// enable data acquisition 
	T1CON=0x0000;//clear register
	_TCKPS=0b11;//timer stops,1:256 prescale,
	TMR1=0;//clear timer1 register
	PR1=625;//interrupt every 10ms
	T1CONbits.TON=1;

	// enable A/D Converter
	_PCFG0 = 0;
	_PCFG1 = 0;
	_SSRC = 0x07;
	_SAMC = 1;
	_ADON = 1;

	T1InterruptUserFunction = DeviceCarrierGetTelemetry;*/
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
	__delay_ms(100);
} //initI2C


void writeI2C( unsigned char add, unsigned char v) // write an integer v to address add
{
	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1(add << 1);
	IdleI2C1();

	// Write data byte
	MasterWriteI2C1( v); 
	IdleI2C1();

	// Terminate command sequence with a stop condition
	StopI2C1();
	IdleI2C1();

} //writeI2C

void writeI2CReg( unsigned char add, unsigned char v, unsigned char w) // write an integer v to address add
{
	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1(add << 1);
	IdleI2C1();

	// Write data byte
	MasterWriteI2C1( v); 
	IdleI2C1();

	// Write data byte
	MasterWriteI2C1( w); 
	IdleI2C1();

	// Terminate command sequence with a stop condition
	StopI2C1();
	IdleI2C1();

} //writeI2C


int readI2C( unsigned char add)  // read an integer from address add
{
	int r;

	IdleI2C1();
	StartI2C1();
	IdleI2C1();

	MasterWriteI2C1((add << 1) | 0x01);
	IdleI2C1();

	__delay_us(100);

	IFS1bits.MI2C1IF = 0;

	r = (unsigned int)(MasterReadI2C1());

	// terminate read sequence (do not send ACK, send  STOP)

	StopI2C1(); 
	IdleI2C1();

	return r;
} //readI2C

int DeviceCarrierReadAdxl345Register( unsigned char add, unsigned char reg )
{
	int a;

	writeI2C(add, reg);
	a = readI2C(add);

	return a;
}

int DeviceCarrierReadTmp112Register( unsigned char add, unsigned char reg )
{
	int a, b, c;

	writeI2C(add, reg);
	a = readI2C(add);

	writeI2C(add, reg + 0x01);
	b = readI2C(add);

	c = (b >> 4) | (a << 4);

	return c;
}

int DeviceCarrierReadHmc5843Register( unsigned char add, unsigned char reg )
{
	unsigned char a, b;
	int c;

	writeI2C(add, reg);
	a = readI2C(add);

	writeI2C(add, reg + 0x01);
	b = readI2C(add);

	c = b | (a << 8);

	return c;
}


int DeviceCarrierMux(int ch)
{
	MPLEXED_A_ON((ch & 0x0001) >> 0);
	MPLEXED_B_ON((ch & 0x0002) >> 1);
	MPLEXED_C_ON((ch & 0x0004) >> 2);

	return MPLEXED_INPUT();
}

void DeviceCarrierGetTelemetry()
{
	unsigned char a, b;
	signed int c;
	int d;

 	IFS0bits.T1IF = 0;	//clear interrupt flag ??

	REG_V5_POWER_GOOD = DeviceCarrierMux(V5_GOOD);
	REG_V12_POWER_GOOD = DeviceCarrierMux(V12_GOOD);
	REG_PAYLOAD_1_PRESENT = !DeviceCarrierMux(NOT_PAYLOAD1_PRESENT);
	REG_PAYLOAD_2_PRESENT = !DeviceCarrierMux(NOT_PAYLOAD2_PRESENT);
	REG_CAMERA_PRESENT = !DeviceCarrierMux(NOT_CAMERA_PRESENT);

	a = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x32 ); // Accel X-Axis Data 0
	b = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x33 ); // Accel X-Axis Data 1
	c = a | (b << 8);
	REG_ACCEL_X = (float)c * 16.0 / 512.0;

	a = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x34 ); // Accel Y-Axis Data 0
	b = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x35 ); // Accel Y-Axis Data 1
	c = a | (b << 8);
	REG_ACCEL_Y = (float)c * 16.0 / 512.0;

	a = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x36 ); // Accel Z-Axis Data 0
	b = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x37 ); // Accel Z-Axis Data 1
	c = a | (b << 8);
	REG_ACCEL_Z = -(float)c * 16.0 / 512.0;

	d = DeviceCarrierReadTmp112Register( TMP112_0_ADDRESS, 0x00 ); // Temp
	REG_TEMP_INT0 = (float)d / 16.0; // TODO: don't forget to scale

	d = DeviceCarrierReadTmp112Register( TMP112_1_ADDRESS, 0x00 ); // Temp
	REG_TEMP_INT1 = (float)d / 16.0; // TODO: don't forget to scale

	d = DeviceCarrierReadHmc5843Register( HMC5843_ADDRESS, 0x03 ); // Magnet X-Axis Data
	REG_MAGNETIC_X = (float)d;

	d = DeviceCarrierReadHmc5843Register( HMC5843_ADDRESS, 0x05 ); // Magnet Y-Axis Data 0
	REG_MAGNETIC_Y = (float)d;

	d = DeviceCarrierReadHmc5843Register( HMC5843_ADDRESS, 0x07 ); // Magnet Z-Axis Data 0
	REG_MAGNETIC_Z = (float)d;

	// Sample Analog input devices
	_CH0SA = EXTERNAL_TEMP_SENSOR;
	_SAMP = 1;

	while (!_DONE) { }
	_DONE = 0;
	REG_TEMP_EXT0 = ADC1BUF0 * ADC_REF_VOLTAGE / ADC_SAMPLE_COUNT;
	
	_CH0SA = HUMIDITY_SENSOR;
	_SAMP = 1;

	while (!_DONE) { }
	_DONE = 0;
	REG_ROBOT_HUMIDITY = ADC1BUF0 * ADC_REF_VOLTAGE / ADC_SAMPLE_COUNT;


	REG_TELEMETRY_COUNT++;
}

void DeviceBothLedsOn()
{
	PWM_LED_ON(1);
	PWM_IRLED_ON(1);
}

void DeviceBothLedsOff()
{
	PWM_LED_ON(0);
	PWM_IRLED_ON(0);
}

void DeviceFlashLeds(unsigned n)
{
	unsigned i;
	for (i = 0; i < n; i++)
	{
		DeviceBothLedsOn();
		__delay_ms(500);
		DeviceBothLedsOff();
		__delay_ms(500);
	}
}

void DeviceCarrierSetTelemetry()
{

}

void DevicePushPowerButton()
{
	PWR_BTN_EN(1); // turn on power button
	DelayMs(100);
	PWR_BTN_EN(0); // turn off power button
}


void DeviceCarrierWait()
{
	if (EXT_PWR_BUTTON() == 1) gRegCarrierState = CARRIER_BOOT;
}


void DeviceCarrierBoot()
{
	unsigned char a;
	unsigned int i;
	unsigned int v3v3_error = 0;

	/* 5V_SBY is not connected on ComExpress so the recommended power up
	   sequence is defined in the Robin Z5xx Datasheet Section 5.3.2
	   to power 12V, wait for S5, deliver PWR_OK, turn on S3 Power Rails,
	   wait for S3, turn on 5V_CB, turn on S0 power rails, turn on
	   3.3V_CB, look for CB_RESET#. */

	/* turn on 3.3V but wait 10ms before checking it since the mux that
	   reads the V3V3_GOOD signal requires 3.3V */
	V3V3_ON(1);
	//block_ms(10);
	// wait 1s for power good signal otherwise flash error and try anyways
	i = 1000 /*ms*/;
	while(!DeviceCarrierMux(V3V3_GOOD))
	{
		if (i-- == 0) /* timeout */
		{
			v3v3_error = 1; // we can't flash the lights yet so do it later
			break;
		}
		//block_ms(1);
	}
	SMBUS_ON(1); // 3.3V SMBus power on
	RADIO_ON(1); // 3.3V to PCIE radio connector
	CODEC_PD_ON(1);            // pull audio codec SHTDWN# pin high
	CAMERA_BOARD_ON(1); // 3.3V & 12V on camera board
	COM_EXPRESS_ON(1); // COMExpres 3.3V on
	//block_ms(1); // wait for pull-ups to stabilize


	// turn on COMExpress 12V main
	V12_ON(1);

	// if there had been a 3.3V error now notify user
	if (v3v3_error)	DeviceFlashLeds(7 /*times*/);

	// wait 1s for power good signal otherwise flash error and try anyways
	i = 1000 /*ms*/;
	while(!DeviceCarrierMux(V12_GOOD))
	{
		if (i-- == 0) /* timeout */
		{
			DeviceFlashLeds(6 /*times*/);
			//block_ms(1000);
			break;
		}
		//block_ms(1);
	}

	COM_EXPRESS_PG_ON(1);

	/* Wait 1s for deassertion of S5 otherwise flash error try to start the
	   ComExpress by pressing the power button. */
	i = 1000 /*ms*/;
	while (SUS_S5)
	{
		if (i-- == 0) /* timeout */
		{
			DeviceFlashLeds(5 /*times*/);
			DevicePushPowerButton();
			//block_ms(1000);
			break;
		}
		//block_ms(1);
	}

	/* Wait 1s for deassertion of S3 otherwise flash error try to start the
	   ComExpress by pressing the power button. */
	i = 1000 /*ms*/;
	while (SUS_S3)
	{
		if (i-- == 0) /* timeout */
		{
			DeviceFlashLeds(4 /*times*/);
			DevicePushPowerButton();
			//block_ms(1000);
			break;
		}
		//block_ms(1);
	}

	V5_ON(1); // 5V regulator on
	// wait 1s for power good signal otherwise flash error and try anyways
	i = 1000 /*ms*/;
	while(!DeviceCarrierMux(V5_GOOD))
	{
		if (i-- == 0) /* timeout */
		{
			DeviceFlashLeds(3 /*times*/);
			//block_ms(1000);
			break;
		}
		//block_ms(1);
	}

	MOTOR_CONTROLLER_ON(1);    // enable 5V on motor controller PIC

	// power other USB devices (like hubs etc) - future

	// configure accelerometer
	writeI2CReg(ADXL345_ADDRESS, 0x2d, 0x08);// Accel (address 0x53), measure = 1, sleep  = 0
	a = DeviceCarrierReadAdxl345Register( ADXL345_ADDRESS, 0x31 );
	writeI2CReg(ADXL345_ADDRESS, 0x31, a | 0x03); // set range to +/- 16g

	// configure magnetometer for active mode
	writeI2CReg(HMC5843_ADDRESS, 0x02, 0x00);

	// Start logging telemetry
	DelayMs(100);
	IEC0bits.T1IE=1;

	DeviceFlashLeds(2 /*times*/);

	gRegCarrierState = CARRIER_RUN;
}

void DeviceCarrierRun()
{
	// TURN PAYLOAD POWER ON IF PAYLOAD DETECTED
	// TODO: add non blocking delay when payload inserted to extend life of connectors
	int p1 = DeviceCarrierMux(NOT_PAYLOAD1_PRESENT);
	int p2 = DeviceCarrierMux(NOT_PAYLOAD2_PRESENT);
	if ((p1 == 0) || (p2 == 0)) ALL_PAYLOAD_POWER_ON(1);
	else ALL_PAYLOAD_POWER_ON(0);
	if (p1 == 0) PAYLOAD1_PWR_ON(1);
	if (p2 == 0) PAYLOAD2_PWR_ON(1);
}

void DeviceCarrierShutdown()
{
	DelayMs(10000); // received a shutdown command, power off in 10 seconds

	V3V3_ON(0);
	COM_EXPRESS_ON(0);
	V5_ON(0);
	SMBUS_ON(0);
	CAMERA_BOARD_ON(0);
	RADIO_ON(0);

	CODEC_PD_ON(0);
	MOTOR_CONTROLLER_ON(0);
	V12_ON(0);
	COM_EXPRESS_PG_ON(0);

	gRegCarrierState = CARRIER_WAIT;
}

void DeviceCarrierSuspend()
{
	// SUSPEND IS NOT IMPLEMENTED YET
	gRegCarrierState = CARRIER_SUSPENDED;
}

void DeviceCarrierSuspended()
{
	// SUSPEND IS NOT IMPLEMENTED YET
}

void DeviceCarrierProcessIO()
{
	switch (gRegCarrierState)
	{
		case CARRIER_WAIT:
			DeviceCarrierWait();
			break;
		case CARRIER_BOOT:
			DeviceCarrierBoot();
			break;
		case CARRIER_RUN:
			DeviceCarrierRun();
			break;
		case CARRIER_SHUTDOWN:
			DeviceCarrierShutdown();
			break;
		case CARRIER_SUSPEND:
			DeviceCarrierSuspend();
			break;
		case CARRIER_SUSPENDED:
			DeviceCarrierSuspended();
			break;
		case CARRIER_INIT:
		default:
			DeviceCarrierInit();
			break;
	}	
}
