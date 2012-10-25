/*==============================================================================
File: InternalCharger.c

Description: Application-level file for the internal battery charger.  This is
  the quick, first-prototype version.

Notes:
  - N/A
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#define TEST_INTERNALCHARGER
/*---------------------------Dependencies-------------------------------------*/
#include "./core/StandardHeader.h"
#include "./core/Timers.h"          // for timers
#include "./core/ADC.h"             // for A/D conversions
#include "./core/TWI.h"             // to configure charging IC
#include "./core/TWISlave.h"        // to send data requested by powerboard
#include "./SMBusDeviceHeaders/BQ24745.h"

/*---------------------------Macros-------------------------------------------*/
// indicator(s)
#define HEARTBEAT_EN(a)       (_TRISE5 = !(a))
#define HEARTBEAT_PIN         _RE5

// charging IC interface pins
#define BQ24745_ACOK_EN(a)    (_TRISB9 = (a)); AD1PCFGL |= (1 << 9) // BUG ALERT: RB9 is analog by default
#define BQ24745_ACOK          _RB9
#define CHARGER_CONNECT_EN(a) (_TRISB10 = !(a)); Nop()
#define CHARGER_CONNECT       _LATB10
#define CELL_A_CONNECT_EN(a)  (_TRISB11 = !(a)); Nop()
#define CELL_A_CONNECT        _LATB11
#define CELL_B_CONNECT_EN(a)  (_TRISB12 = !(a)); Nop()
#define CELL_B_CONNECT        _LATB12
#define _5V_EN_EN(a)          (_TRISB13 = !(a)); Nop()
#define _5V_EN                _LATB13
#define BQ24745_EN_EN(a)      (_TRISB14 = !(a)); Nop()
#define BQ24745_EN            _LATB14

// timer(s)
#define _10ms                 10
#define _100ms                100
#define HEARTBEAT_TIMER       0
#define HEARTBEAT_TIME        (5*_100ms)
#define AWAKENING_TIMER       1
#define AWAKENING_TIME        (10*_100ms)
#define CHARGER_REFRESH_TIMER 2
#define CHARGER_REFRESH_TIME  (20*_100ms)

// powerboard protocol
#define MY_SLAVE_ADDRESS      0x0C
#define REQ_CHARGER_ALIVE     0xCA
#define RESP_ALIVE            0xDA
#define ERROR_RESPONSE        0xFF
  
/*---------------------------Type Definitions---------------------------------*/
// possible states
typedef enum {
	kAwakening = 0,
	kCharging,
} kChargerState;

/*---------------------------Helper Function Prototypes-----------------------*/
static void InitCharger(void);
static void InitPins(void);
static void ConfigureChargingIC(void);
static void RunChargerSM(void);
static uint8_t MyI2C1Response(const uint8_t command, 
                              const uint8_t response_index);
                              
/*---------------------------Module Variables---------------------------------*/
static uint8_t bqdata[2] = {0};
static TWIDevice bq24745 = {
  .address = BQ24745_SLAVE_ADDRESS,
  .subaddress = 0x00,
  .n_data_bytes = 2,  // number of data bytes the device will send us
  .data = bqdata
};

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_INTERNALCHARGER
#include "./core/ConfigurationBits.h"
int main(void) {
  InitCharger();
  
  while (1) RunChargerSM();
  
  return 0;
}
#endif

/*---------------------------Helper Function Definitions----------------------*/
static void RunChargerSM(void) {
  static kChargerState state = kAwakening;
  
  // toggle a pin to indicate normal operation
  if (TMRS_IsTimerExpired(HEARTBEAT_TIMER)) {
    TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
    HEARTBEAT_PIN ^= 1;
  }
  
  /*
  if (TMRS_IsTimerExpired(CHARGER_REFRESH_TIMER)) {
    TMRS_StartTimer(CHARGER_REFRESH_TIMER, CHARGER_REFRESH_TIME);  
    ConfigureChargingIC();
  }
  */

  switch (state) {
    case kAwakening:
      if (TMRS_IsTimerExpired(AWAKENING_TIMER)) {
        // TODO: update our status to 'on dock and alive'
        BQ24745_EN = 1;
        CHARGER_CONNECT = 1;
        CELL_A_CONNECT = 1;
        CELL_B_CONNECT = 1;
        ConfigureChargingIC();
        TMRS_StartTimer(CHARGER_REFRESH_TIMER, CHARGER_REFRESH_TIME);
        state = kCharging;
      }
      break;
    case kCharging:
      if (TMRS_IsTimerExpired(CHARGER_REFRESH_TIMER)) {
        TMRS_StartTimer(CHARGER_REFRESH_TIMER, CHARGER_REFRESH_TIME);
        
        ConfigureChargingIC();
        if (TWI_ErrorHasOccurred(kTWI02)) {
          TWI_Refresh(kTWI02);
        }  
      }
      break;
  }
}


/*
Description: Configures the upper limits for the input current, 
  output charging voltage and output charging current through 
  an SMBus interface.
Notes:
  - see p.23 of battery charger datasheet 'Enable and Disable Charging'
  - this must be done periodically to avoid a timeout (170s) within the bq24745
*/
static void ConfigureChargingIC(void) {
  uint8_t temp_data[2] = {0};
  
  // configure the maximum input current to ~8A (8.192A, 3/cell*2cells + ~2 for robot)
  temp_data[0] = 0x10;
  temp_data[1] = 0x00;
  bq24745.subaddress = BQ24745_INPUT_CURRENT;
  if (TWI_IsBusIdle(kTWI02)) TWI_WriteData(kTWI02, &bq24745, temp_data);
  Delay(5);
  
  // configure the maximum charging voltage to ~16.5V 
  temp_data[0] = 0x40;  // transmit 0x4070 (p.20-21 of bq24745 datasheet)
  temp_data[1] = 0x70;
  bq24745.subaddress = BQ24745_CHARGE_VOLTAGE;
  if (TWI_IsBusIdle(kTWI02)) TWI_WriteData(kTWI02, &bq24745, temp_data);
  Delay(5);
  
  // configure the maximum charging current to ~3A
  temp_data[0] = 0x0B;  // (p.21-22 of bq24745 datasheet)
  temp_data[1] = 0xB8;
  bq24745.subaddress = BQ24745_CHARGE_CURRENT;
  if (TWI_IsBusIdle(kTWI02)) TWI_WriteData(kTWI02, &bq24745, temp_data);
  Delay(5);
}


/*
Description: Maps commands to internal variables.  This function
  essentially implements the I2C-based protocol between the devices.
Parameters:
  uint8_t command,        the subaddress of whose contents are requested
  uint8_t response_index, which byte (0th or 1st) of the response
*/
static uint8_t MyI2C1Response(const uint8_t command, 
                              const uint8_t response_index) {
  // NB: this reponse is actually sent twice
  switch (command) {
    case REQ_CHARGER_ALIVE: return RESP_ALIVE;
    default: return ERROR_RESPONSE;
  }
}


static void InitPins(void) {
  // charging IC interface pin(s)
  BQ24745_ACOK_EN(1);
  CHARGER_CONNECT_EN(1); CHARGER_CONNECT = 1;
  CELL_A_CONNECT_EN(1); CELL_A_CONNECT = 1;
  CELL_B_CONNECT_EN(1); CELL_B_CONNECT = 1;
  _5V_EN_EN(1); _5V_EN = 1; // NB: TRIS must be set to high-Z to turn off!
  BQ24745_EN_EN(1); BQ24745_EN = 1;
  
  // configure and initialize indicator(s)
  HEARTBEAT_EN(1); HEARTBEAT_PIN = 0;
}


static void InitCharger(void) {
  InitPins();
  
  // initialize any dependent modules
  I2C1_UserProtocol = MyI2C1Response;
  TWISlave_Init(kTWISlave01, MY_SLAVE_ADDRESS, I2C);
  TWI_Init(kTWI02, kTWIBaudRate100kHz, SMBUS);
  TMRS_Init();
  
  // prime any timers that require it
  TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
  TMRS_StartTimer(AWAKENING_TIMER, AWAKENING_TIME);
}
