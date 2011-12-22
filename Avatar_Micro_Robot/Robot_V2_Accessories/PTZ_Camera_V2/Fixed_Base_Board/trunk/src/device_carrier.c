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

#include "stdhdr.h"
#include "device_carrier.h"

#define V3V3_ON(a)               _LATC14  = a
#define V3V3_EN(a)               _TRISC14 = !a

#define V5_ON(a)                 _LATF5   = a
#define V5_EN(a)                 _TRISF5  = !a

#define V12_ON(a)                _LATG8   = !a // inverse logic
#define V12_EN(a)                _TRISG8  = !a

#define COM_EXPRESS_ON(a)        _LATF1   = a
#define COM_EXPRESS_EN(a)        _TRISF1  = !a

#define COM_EXPRESS_PG_ON(a)     _LATB10  = a // PWR_OK
#define COM_EXPRESS_PG_EN(a)     _TRISB10 = !a

#define GPS_ON(a)                _LATF4   = a
#define GPS_EN(a)                _TRISF4  = !a

#define RADIO_ON(a)              _LATG9   = a
#define RADIO_EN(a)              _TRISG9  = !a

#define CAMERA_BOARD_ON(a)       _LATE6   = a
#define CAMERA_BOARD_EN(a)       _TRISE6  = !a

#define SMBUS_ON(a)              _LATG6   = a
#define SMBUS_EN(a)              _TRISG6  = !a

#define AUDIO_ON(a)              _LATG7   = a
#define AUDIO_EN(a)              _TRISG7  = !a

#define NCOM_WAKE0_ON(a)         _LATB15  = a
#define NCOM_WAKE0_EN(a)         _TRISB15 = !a

#define NCOM_WAKE1_ON(a)         _LATF0   = !a
#define NCOM_WAKE1_EN(a)         _TRISF0  = !a

#define NCOM_SYSRST_ON(a)        _LATB9   = !a
#define NCOM_SYSRST_EN(a)        _TRISB9  = !a

#define PWR_BTN_ON(a)             _LATD8  = !a
#define PWR_BTN_EN(a)             _TRISD8 = !a

#define PAYLOAD1_PWR_ON(a)       _LATB12  = a
#define PAYLOAD1_PWR_EN(a)       _TRISB12 = !a

#define PAYLOAD2_PWR_ON(a)       _LATB11  = a
#define PAYLOAD2_PWR_EN(a)       _TRISB11 = !a

#define MOTOR_CONTROLLER_ON(a)   _LATB13  = a
#define MOTOR_CONTROLLER_EN(a)   _TRISB13 = !a

#define PWM_IRLED_ON(a)          _LATD3   = !a // digital PWM output 100% duty cycle
#define PWM_IRLED_EN(a)          _TRISD3  = !a

#define PWM_LED_ON(a)            _LATD4   = !a
#define PWM_LED_EN(a)            _TRISD4  = !a


#define PIC_WAKE0()              (!_RB14)
#define PAYLOAD1_PRESENT()       (!_RB8)
#define PAYLOAD2_PRESENT()       (!_RB3)
#define COMPASS_DRDY()           (!_RB2)
#define PWR_BTN()                (!_RB4)
#define EXT_PWR_BUTTON()         (!_RB4)
#define SUS_S5()                 (!_RB5)
#define SMB_ALERT()              (!_RC13)
#define V3V3_GOOD()              (_RD1)
#define V12_GOOD()               (_RD2)
#define V5_GOOD()                (_RD6)
#define SUS_S3()                 (!_RE5)
#define NC_THRMTRIP()            (!_RE7)


#define EXTERNAL_TEMP_SENSOR_CH  0 // CH0 : input wth 100k pull-up (not used)
#define HUMIDITY_SENSOR_CH       1 // CH1 : analog input (0 - 3V ?)
#define ADC_REF_VOLTAGE          3.3f
#define ADC_SAMPLE_COUNT         1024

#define ADC_CH_TO_BIT(a) (2^a)

// board ids

#define GPS_TXD // pin 45 (RP12)
#define GPS_RXD // pin 46 (RP11)
#define SMB_SDA // pin 43 (SDA1)
#define SMB_SCL // pin 44 (SCLK1)
#define SECURE_PIC // pin 53


#define ADXL345_ADDRESS           0x53
#define TMP112_0_ADDRESS          0x48
#define TMP112_1_ADDRESS          0x49
#define HMC5843_ADDRESS           0x1E
#define EEPROM_ADDRESS            0x50 // to 0x57 (0x50 to 0x53 address four 256-byte blocks)

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
extern int gNewData;


// FUNCTIONS

#pragma code

uint8_t gWhiteIntensity = 0;
uint8_t gIrIntensity = 0;
uint8_t gLedControlRemote = 1;

void DeviceFlashLeds(unsigned n)
{
   unsigned i;
   gLedControlRemote = 0; // block USB from updating
   for (i = 0; i < n; i++)
   {
      gWhiteIntensity = 255;
      gIrIntensity = 255;
      block_ms(500);
      gIrIntensity = 0;
      gWhiteIntensity = 0;
      block_ms(500);
   }
   gLedControlRemote = 1;
}

void DeviceCycleMotorController()
{
	MOTOR_CONTROLLER_ON(0);
	block_ms(500);
	MOTOR_CONTROLLER_ON(1);
}

void DeviceCarrierAdxl345Callback(unsigned char addr, unsigned char bus,
                                  unsigned char reg, unsigned char *data,
                                  unsigned int len)
{
   signed int c;
   switch (reg)
   {
      case 0x32:  // Accel Data
         c = data[0] | (data[1] << 8);
         REG_ACCEL_X = (float)c * 16.0 / 512.0;
         c = data[2] | (data[3] << 8);
         REG_ACCEL_Y = (float)c * 16.0 / 512.0;
         c = data[4] | (data[5] << 8);
         REG_ACCEL_Z = (float)c * 16.0 / 512.0;

         // read 6-bytes of magnetometer data
         readRegI2C(1, HMC5843_ADDRESS, 0x03, 6);
         break;

      default:
         break;
   }
}


void DeviceCarrierHmc5843Callback(unsigned char addr, unsigned char bus,
                                  unsigned char reg, unsigned char *data,
                                  unsigned int len)
{
   signed int c;
   switch (reg)
   {
      case 0x03:  // Mag Data
         c = data[0] | (data[1] << 8);
         REG_MAGNETIC_X = (float)c;
         c = data[2] | (data[3] << 8);
         REG_MAGNETIC_Y = (float)c;
         c = data[4] | (data[5] << 8);
         REG_MAGNETIC_Z = (float)c;

         // read 2-bytes of temp-0 data
         readRegI2C(1, TMP112_0_ADDRESS, 0x00, 2);
         break;

      default:
         break;
   }
}


void DeviceCarrierTmp112Callback(unsigned char addr, unsigned char bus,
                                 unsigned char reg, unsigned char *data,
                                 unsigned int len)
{
   signed int c;
   switch (reg)
   {
      case 0x00: // temperature register
         c = ((int)data[1] >> 4) | ((int)data[0] << 4);

        switch (addr)
         {
            case TMP112_0_ADDRESS:
               REG_TEMP_INT0 = (float)c / 16.0;

               // read 2-bytes of temp-1 data
               readRegI2C(1, TMP112_1_ADDRESS, 0x00, 2);
               break;

            case TMP112_1_ADDRESS:
               REG_TEMP_INT1 = (float)c / 16.0;
               break;
         }

         break;

      default:
         break;
   }
}

void DeviceCarrierLedInterrupt()
{
   static uint8_t counter;


   if (counter >= gWhiteIntensity) PWM_LED_ON(0);
   else PWM_LED_ON(1);

   if (counter >= gIrIntensity) PWM_IRLED_ON(0);
   else PWM_IRLED_ON(1);

   if (counter++ > 3) counter = 0;

   IFS0bits.T2IF = 0;
}


void DeviceCarrierInit()
{
   // turn off unused analog input
   AD1PCFGL = 0xFFFF & ~ADC_CH_TO_BIT(EXTERNAL_TEMP_SENSOR_CH) &
                        ~ADC_CH_TO_BIT(HUMIDITY_SENSOR_CH);

   // SET DEFAULT OUTPUTS ***************************************************
   SMBUS_ON(0);
   AUDIO_ON(0);
   V12_ON(0);
   RADIO_ON(0);
   NCOM_WAKE0_ON(0);
   NCOM_SYSRST_ON(1);   // default: depressed(ON) but don't enable PIC output
   COM_EXPRESS_ON(0);
   NCOM_WAKE1_ON(0);
   COM_EXPRESS_PG_ON(0);
   PAYLOAD1_PWR_ON(0);
   PAYLOAD2_PWR_ON(0);
   MOTOR_CONTROLLER_ON(0);
   V3V3_ON(0);
   V5_ON(0);
   GPS_ON(0);
   PWR_BTN_ON(1);       // default: depressed(ON) but don't enable PIC output
   PWM_IRLED_ON(0);
   PWM_LED_ON(0);
   CAMERA_BOARD_ON(0);

   // ENABLE PIC OUTPUTS *****************************************************
   /* TODO: be able to unwind this because low-Z outputs may cause leakage
             current in sleep state */
   SMBUS_EN(1);
   AUDIO_EN(1);
   V12_EN(1);
   RADIO_EN(1);
   NCOM_WAKE0_EN(1);
   NCOM_SYSRST_EN(0);   // default: depressed(ON) but don't enable PIC output
   COM_EXPRESS_EN(1);
   NCOM_WAKE1_EN(1);
   COM_EXPRESS_PG_EN(1);
   PAYLOAD2_PWR_EN(1);
   PAYLOAD1_PWR_EN(1);
   MOTOR_CONTROLLER_EN(1);
   V3V3_EN(1);
   V5_EN(1);
   GPS_EN(1);
   PWR_BTN_EN(0);       // default: depressed(ON) but don't enable PIC output
   PWM_IRLED_EN(1);
   PWM_LED_EN(1);
   CAMERA_BOARD_EN(1);

   // CONFIGURE I2C **********************************************************
   ODCGbits.ODG3 = 1;      // SDA1 is set to open drain
   ODCGbits.ODG2 = 1;      // SCL1 is set to open drain
   I2C1BRG = 0xFF;         // ~400kHz (0x25)
   I2C1CONbits.I2CEN = 1;  // enable I2C
   _MI2C1IE = 1; // enable interrupt
   registerI2CReadCallback(DeviceCarrierAdxl345Callback, 1, ADXL345_ADDRESS);
   registerI2CReadCallback(DeviceCarrierHmc5843Callback, 1, HMC5843_ADDRESS);
   registerI2CReadCallback(DeviceCarrierTmp112Callback,  1, TMP112_0_ADDRESS);
   registerI2CReadCallback(DeviceCarrierTmp112Callback,  1, TMP112_1_ADDRESS);

   // CONFIGURE TIMER 1 - TELEMETRY LOGGER ************************************
   T1CON=0x0000;    //clear register
   _TCKPS=0b11;     //timer stops,1:256 prescale,
   TMR1=0;          //clear timer1 register
   PR1=625;         //interrupt every 10ms(625), 1ms(62)
   T1InterruptUserFunction = DeviceCarrierGetTelemetry;

   // CONFIGURE TIMER 2 - LED DUTY CYCLE *************************************
   T2CON = 0x0000;
   T2CONbits.TCKPS = 0x03;
   TMR2  = 0;
   PR2   = 10;            // 16us(1)
   IPC1bits.T2IP = 0x7;  // set to highest priority otherwise LED flickers
   T2InterruptUserFunction = DeviceCarrierLedInterrupt;
   T2CONbits.TON=1;
   IEC0bits.T2IE=1;

   gRegCarrierState = CARRIER_WAIT;
}

typedef unsigned int TIMER;
TIMER __device_carrier_timer_payload1 = 0;
TIMER __device_carrier_timer_payload2 = 0;
TIMER __device_carrier_timer_off_button = 0;

int DeviceCarrierHysteresis(int condition, unsigned int *timer, unsigned int expiration)
{
   if( condition )
   {
      if( ((*timer) & 0x7FFF) < expiration )
      {
         (*timer)++;
      }
      else
      {
         (*timer) |= 0x8000; // turn on
      }
   }
   else
   {
      if( ((*timer) & 0x7FFF) > 0 )
      {
         (*timer)--;
      }
      else
      {
         *timer = 0x0000; // turn off
      }
   }

   return ((*timer) >> 15) & 1;
}

int DeviceCarrierDelay(int condition, unsigned int *timer, unsigned int expiration)
{
   if( condition )
   {
      if( ((*timer) & 0x7FFF) < expiration )
      {
         (*timer)++;
      }
      else
      {
         (*timer) |= 0x8000; // turn on
      }
   }
   else
   {
      *timer = 0x0000; // turn off
   }

   return ((*timer) >> 15) & 1;
}

void DeviceCarrierGetTelemetry()
{
   // IF NEW CONNECTION, CYCLE MOTOR CONTROLLER
   // THIS IS TEMPORARY, REMOVE WHEN FIXED
   static int new_data;
   static uint16_t last_update_count;

   if (new_data != gNewData)
   {
      new_data = gNewData;
      if ((REG_TELEMETRY_COUNT - last_update_count) > 200 /* 2seconds */)
      {
         DeviceCycleMotorController();
		 DeviceFlashLeds(1);
      }
      last_update_count = REG_TELEMETRY_COUNT;
   }

   // TURN LIGHTS ON
   if( gLedControlRemote )
   {
      gIrIntensity = REG_IR_LED;
      gWhiteIntensity = REG_WHITE_LED;
   }

   // GET POWER CONDITION
   REG_V5_POWER_GOOD = V5_GOOD();
   REG_V12_POWER_GOOD = V12_GOOD();

   // TURN PAYLOAD POWER ON IF PAYLOAD DETECTED
   REG_PAYLOAD_1_PRESENT = DeviceCarrierHysteresis(PAYLOAD1_PRESENT(),
                                        &__device_carrier_timer_payload1, 500);
   PAYLOAD1_PWR_ON(REG_PAYLOAD_1_PRESENT);
   REG_PAYLOAD_2_PRESENT = DeviceCarrierHysteresis(PAYLOAD2_PRESENT(),
                                        &__device_carrier_timer_payload2, 500);
   PAYLOAD2_PWR_ON(REG_PAYLOAD_2_PRESENT);

   if( DeviceCarrierDelay(EXT_PWR_BUTTON(),
                                   &__device_carrier_timer_off_button, 10000) )
   {
      gRegCarrierState = CARRIER_SHUTDOWN;
   }


   readRegI2C(1, ADXL345_ADDRESS, 0x32, 6); // read 6-bytes of accelerometer data

   // Sample Analog input devices
   // TODO: get rid of blocking code here
   _CH0SA = EXTERNAL_TEMP_SENSOR_CH;
   _SAMP = 1;

   while (!_DONE) { }
   _DONE = 0;
   REG_TEMP_EXT0 = ADC1BUF0 * ADC_REF_VOLTAGE / ADC_SAMPLE_COUNT;
   
   _CH0SA = HUMIDITY_SENSOR_CH;
   _SAMP = 1;

   while (!_DONE) { }
   _DONE = 0;
   REG_ROBOT_HUMIDITY = ADC1BUF0 * ADC_REF_VOLTAGE / ADC_SAMPLE_COUNT;

   REG_TELEMETRY_COUNT++;
   IFS0bits.T1IF = 0;	//clear interrupt flag ??
}


void DevicePushPowerButton()
{
   PWR_BTN_EN(1); // depress(ON) power button
   block_ms(100);
   PWR_BTN_EN(0); // release(OFF) power button
}


void DeviceCarrierWait()
{
   gRegCarrierState = CARRIER_BOOT; // temporary for carrier
   if (EXT_PWR_BUTTON() == 1) gRegCarrierState = CARRIER_BOOT;
}


void DeviceCarrierBoot()
{
   unsigned int i;

   /* 5V_SBY is not connected on ComExpress so the recommended power up
      sequence is defined in the Robin Z5xx Datasheet Section 5.3.2
      to power 12V, wait for S5, deliver PWR_OK, turn on S3 Power Rails,
      wait for S3, turn on 5V_CB, turn on S0 power rails, turn on
      3.3V_CB, look for CB_RESET#. */

   /* turn on 3.3V but wait 10ms before checking it since the mux that
      reads the V3V3_GOOD signal requires 3.3V */
   V3V3_ON(1);
   NCOM_SYSRST_EN(1);
   block_ms(10);
   // wait 1s for power good signal otherwise flash error and try anyways
   i = 1000 /*ms*/;
   while(!V3V3_GOOD())
   {
      if (i-- == 0) /* timeout */
      {
         DeviceFlashLeds(7 /*times*/);
         break;
      }
      block_ms(1);
   }
   SMBUS_ON(1);
   RADIO_ON(1);
   AUDIO_ON(1);
   GPS_ON(0); // for now turn this off
   CAMERA_BOARD_ON(1);
   COM_EXPRESS_ON(1);
   block_ms(1); // wait for pull-ups to stabilize

   // turn on COMExpress 12V main
   V12_ON(1);

   // wait 1s for power good signal otherwise flash error and try anyways
   i = 1000 /*ms*/;
   while(!V12_GOOD())
   {
      if (i-- == 0) /* timeout */
      {
         DeviceFlashLeds(6 /*times*/);
         block_ms(1000);
         break;
      }
      block_ms(1);
   }

   block_ms(1);
   COM_EXPRESS_PG_ON(1);

   /* Wait 1s for deassertion of S5 otherwise flash error try to start the
      ComExpress by pressing the power button. */
   i = 1000 /*ms*/;
   while (SUS_S5())
   {
      if (i-- == 0) /* timeout */
      {
         DeviceFlashLeds(5 /*times*/);
         DevicePushPowerButton();
         block_ms(1000);
         break;
      }
      block_ms(1);
   }

   /* Wait 1s for deassertion of S3 otherwise flash error try to start the
      ComExpress by pressing the power button. */
   i = 1000 /*ms*/;
   while (SUS_S3())
   {
      if (i-- == 0) /* timeout */
      {
         DeviceFlashLeds(4 /*times*/);
         DevicePushPowerButton();
         block_ms(1000);
         break;
      }
      block_ms(1);
   }

   V5_ON(1); // 5V regulator on
   // wait 1s for power good signal otherwise flash error and try anyways
   i = 1000 /*ms*/;
   while(!V5_GOOD())
   {
      if (i-- == 0) /* timeout */
      {
         DeviceFlashLeds(3 /*times*/);
         block_ms(1000);
         break;
      }
      block_ms(1);
   }

   MOTOR_CONTROLLER_ON(1);    // enable 5V on motor controller PIC

   block_ms(110);
   NCOM_SYSRST_EN(0);

   // power other USB devices (like hubs etc) - future

   // Accel (address 0x53), measure = 1, sleep  = 0
   writeRegByteI2CBlocking(1, ADXL345_ADDRESS, 0x2d, 0x08);

   // set range to +/- 16g
   writeRegByteI2CBlocking(1, ADXL345_ADDRESS, 0x31, 0x03);

   // configure magnetometer for active mode
   writeRegByteI2CBlocking(1, HMC5843_ADDRESS, 0x02, 0x00);

   // enable data acquisition 
   T1CONbits.TON=1;

   // enable A/D Converter
   _SSRC = 0x07; // auto-convert
   _SAMC = 0; // holding (enable this to sample)
   _ADON = 1;

   // Start logging telemetry
   IEC0bits.T1IE=1;
   DeviceFlashLeds(2 /*times*/);

   gRegCarrierState = CARRIER_RUN;
}

void DeviceCarrierRun()
{
}

void DeviceCarrierShutdown()
{
   block_ms(10000); // received a shutdown command, power off in 10 seconds

   V3V3_ON(0);
   COM_EXPRESS_ON(0);
   V5_ON(0);
   SMBUS_ON(0);
   CAMERA_BOARD_ON(0);
   RADIO_ON(0);

   AUDIO_ON(0);
   MOTOR_CONTROLLER_ON(0);
   V12_ON(0);
   COM_EXPRESS_PG_ON(0);

    // stop logging telemetry
    IEC0bits.T1IE=0;

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
   gRegCarrierState = CARRIER_RUN;
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
