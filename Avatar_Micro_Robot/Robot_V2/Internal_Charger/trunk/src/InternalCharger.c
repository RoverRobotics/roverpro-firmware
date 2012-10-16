/*==============================================================================
File: InternalCharger.c

Description: This is the overarching file that encapsulates the 
  application-level firmware for the internal charger within the robot.
  
Features: (TODO: write unit test for each one of these)
  - charges the battery while providing power to the robot
  
Notes:
  - N/A
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
==============================================================================*/
#define TEST_INTERNALCHARGER
/*---------------------------Dependencies-------------------------------------*/
#include "./core/StandardHeader.h"
#include "./core/Timers.h"          // for timers
#include "./core/ADC.h"             // for A/D conversions
#include "./core/TWI.h"             // for I2C/SMBus library
#include "./SMBusDeviceHeaders/BQ24745.h"

/*---------------------------Macros-------------------------------------------*/
// power management threshold(s)
#define MAX_CELL_CURRENT    465   // maximum allowable current to an individual cell
                                  // [au], 0.01S*(3A*0.05Ohm)*1k ~= 1.5V, (1.5 / 3.30) * 1023=
#define MIN_CELL_CURRENT    8     // [au], minimum current to an individual cell
                                  // after which charging should terminate
                                  // 0.01S*(100mA*0.05Ohm)*1k ~= 1.5V, (1.5 / 3.30) * 1023=
                                  
#define MIN_TOPPING_V       705   // topping charge voltage threshold (when to charge a little more)
                                  // (1.6 / (10 + 1.6) * 16.5) / 3.3 * 1023=

// charging IC interface pins
#define BQ24745_ACOK_EN(a)    (_TRISB9 = (a)); AD1PCFGL |= (1 << 9) // BUG ALERT: RB9 is analog by default
#define BQ24745_ACOK          _RB9
#define CHARGER_CONNECT_EN(a) (_TRISB10 = !(a))
#define CHARGER_CONNECT       _LATB10
#define CELL_A_CONNECT_EN(a)  (_TRISB11 = !(a))
#define CELL_A_CONNECT        _LATB11
#define CELL_B_CONNECT_EN(a)  (_TRISB12 = !(a))
#define CELL_B_CONNECT        _LATB12
#define _5V_EN_EN(a)          (_TRISB13 = !(a))
#define _5V_EN                _LATB13
#define BQ24745_EN_EN(a)      (_TRISB14 = !(a))
#define BQ24745_EN            _LATB14

// indicator(s)
#define HEARTBEAT_EN(a)     (_TRISE5 = !(a))
#define HEARTBEAT_PIN       _RE5
#define RED_LED_EN(a)       (_TRISD2 = !(a))
#define RED_LED             _RD2
#define GREEN_LED_EN(a)     (_TRISD3 = !(a))
#define GREEN_LED           _RD3

// analog sensing pins (ANx)
// TODO: FIX THIS!!! #define V_WALLWART          
#define VCELLA_CURR_PIN     4     // cell A current sensing
#define VCELLB_CURR_PIN     6     // cell B current sensing
#define VCELLA_PIN          5     // cell A voltage sensing
#define VCELLB_PIN          7     // cell B voltage sensing

// timer(s)
#define _10ms                 10
#define _100ms                100
#define HEARTBEAT_TIMER       0
#define HEARTBEAT_TIME        (5*_100ms)
#define CHARGER_REFRESH_TIMER 1           // to update charging IC
#define CHARGER_REFRESH_TIME  (20*_100ms) // must be less than 170s
#define COOLDOWN_TIMER        2
#define COOLDOWN_TIME         (_100ms)

//#define UART1_TX_RPN        25
//#define UART1_RX_RPN        20

#define MAX_NUM_FILTERS       8 // for IIR filter

/*---------------------------Type Definitions---------------------------------*/
// states in which the master can be
typedef enum {
  kSleeping = 0,  // standing by after charging has finished
  kCharging
} kChargerState;

/*---------------------------Helper Function Prototypes-----------------------*/
static void InitCharger(void);
static void InitPins(void);
static void ConfigureChargingIC(void);
static void ProcessChargerIO(void);
static void RunChargerStateMachine(void);

/*---------------------------Module Variables---------------------------------*/
//static kInternalChargerState state = kWaiting;

static unsigned char bqdata[2] = {0};
static TWIDevice bq24745 = {
  .address = BQ24745_SLAVE_ADDRESS,
  .subaddress = 0x00,
  .numDataBytes = 2,  // number of data bytes the device will send us
  .data = bqdata
};

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_INTERNALCHARGER
#include "./core/ConfigurationBits.h"
int main(void) {
  //_SWDTEN = 1; // enable the watchdog timer
  InitCharger();
  
  while (1) {
    //ClrWdt();
    ProcessChargerIO();
  }
  
  return 0;
}
#endif

/*---------------------------Helper Function Definitions----------------------*/
static void InitCharger(void) {
  InitPins();
  
  // initialize any dependent module(s)
  uint16_t ad_bitmask = ((1 << VCELLA_CURR_PIN) |
                         (1 << VCELLA_PIN)      |
                         (1 << VCELLB_CURR_PIN) |
                         (1 << VCELLB_PIN));
  ADC_Init(ad_bitmask);
  TMRS_Init();
  TWI_Init(kTWI02, kTWIBaudRate100kHz, SMBUS);
  
  // initialize any peripheral devices
  ConfigureChargingIC();
  
 	// prime any timers that require it
  TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
  TMRS_StartTimer(CHARGER_REFRESH_TIMER, CHARGER_REFRESH_TIME);
}


static void InitPins(void) {
  // charging IC interface pin(s)
  BQ24745_ACOK_EN(1);
  CHARGER_CONNECT_EN(1); CHARGER_CONNECT = 1;
  CELL_A_CONNECT_EN(1); CELL_A_CONNECT = 1;
  CELL_B_CONNECT_EN(1); CELL_B_CONNECT = 1;
  _5V_EN_EN(1); _5V_EN = 1; // NB: this must be set to high-impedance to turn off!
  BQ24745_EN_EN(1); BQ24745_EN = 1;
  
  // indicator(s)
  HEARTBEAT_EN(1); HEARTBEAT_PIN = 0;
  RED_LED_EN(1); RED_LED = 0;
  GREEN_LED_EN(1); GREEN_LED = 0;
}


static void ProcessChargerIO(void) {
  // NEVER exceed the current limit of the battery
  if ((MAX_CELL_CURRENT < ADC_GetConversion(VCELLA_CURR_PIN)) ||
      (MAX_CELL_CURRENT < ADC_GetConversion(VCELLB_CURR_PIN))) {
    // disconnect the battery cells and wait a while
    CELL_A_CONNECT = 0;
    CELL_B_CONNECT = 0;
    TMRS_StartTimer(COOLDOWN_TIMER, COOLDOWN_TIME);
  }
  
  // toggle a pin to indicate normal operation
  if (TMRS_IsTimerExpired(HEARTBEAT_TIMER)) {
    TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
    HEARTBEAT_PIN ^= 1;
  }
  
  // run the state machine as long as something unexpected did NOT happen
  if (TMRS_IsTimerExpired(COOLDOWN_TIMER)) {
    RunChargerStateMachine();
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
  
  // configure the input current to ~8A (8.192A, 3/cell*2cells + ~2 for robot)
  temp_data[0] = 0x10;
  temp_data[1] = 0x00;
  bq24745.subaddress = BQ24745_INPUT_CURRENT;
  TWI_WriteData(kTWI02, &bq24745, temp_data); Delay(5);
  
  // configure the charging voltage to ~16.5V 
  temp_data[0] = 0x40;  // transmit 0x4070 (p.20-21 of bq24745 datasheet)
  temp_data[1] = 0x70;
  bq24745.subaddress = BQ24745_CHARGE_VOLTAGE;
  TWI_WriteData(kTWI02, &bq24745, temp_data); Delay(5);
  
  // configure the charging current limit to ~6A (5.888A)
  temp_data[0] = 0x17;  // (p.21-22 of bq24745 datasheet)
  temp_data[1] = 0x00;
  bq24745.subaddress = BQ24745_CHARGE_CURRENT;
  TWI_WriteData(kTWI02, &bq24745, temp_data); Delay(5);
}


/*
Description: This function executes the state machine.
Notes:
  - Lets the battery voltage drop to 4.00V/cell and recharge to only 
    4.05V/cell instead of the full 4.20V/cell. This reduces voltage-related
    stress and prolongs battery life.
*/
static void RunChargerStateMachine(void) {
  static kChargerState state = kSleeping;
  
  switch (state) {
    case kSleeping:
      // if we sense that the battery voltage is too low
      if (ADC_GetConversion(VCELLA_PIN) < MIN_TOPPING_V ||
          ADC_GetConversion(VCELLB_PIN) < MIN_TOPPING_V) {
        // turn back on the charger and ensure the cells are connected
        BQ24745_EN = 1; 
        CELL_A_CONNECT = 1;
        CELL_B_CONNECT = 1;
        state = kCharging;
      }
      break;
    case kCharging:
      // do NOT let the charger time out
      if (TMRS_IsTimerExpired(CHARGER_REFRESH_TIMER)) {
        TMRS_StartTimer(CHARGER_REFRESH_TIMER, CHARGER_REFRESH_TIME);
        ConfigureChargingIC();
      }
      
      // disconnect each cell individually 
      if (ADC_GetConversion(VCELLA_CURR_PIN) < MIN_CELL_CURRENT) {
        CELL_A_CONNECT = 0;
      }
      
      if (ADC_GetConversion(VCELLB_CURR_PIN) < MIN_CELL_CURRENT) {
        CELL_B_CONNECT = 0;
      }
      
      // if both batteries are accepting very little current
      if (ADC_GetConversion(VCELLA_CURR_PIN) < MIN_CELL_CURRENT &&  
          ADC_GetConversion(VCELLB_CURR_PIN) < MIN_CELL_CURRENT) {
        // disable the charger
        BQ24745_EN = 0;
        
        // re-connect each cell so we can monitor their SOC's
        // via their voltages
        CELL_A_CONNECT = 1;
        CELL_B_CONNECT = 1;
        state = kSleeping;
      }
      break;
  }
}
  

/*
Function: IIRFilter
Description: Passes the input through an Infinite-Impulse-Response filter
  characterized by the parameter alpha.
Paramters:
  uint8_t i,      filter index
  float x,        the current sample to be filtered
  float alpha,    the knob on how much to filter, [0,1)
                  alpha = t / (t + dT)
                  where t = the low-pass filter's time-constant
                    dT = the sample rate
Notes:
  - see also: http://dsp.stackexchange.com/questions/1004/low-pass-filter-in-non-ee-software-api-contexts
  - aka a 'leaky integrator'
*/
float IIRFilter(const uint8_t i, const float x, const float alpha) {
  static float y_lasts[MAX_NUM_FILTERS] = {0};
  float y = alpha * y_lasts[i] + (1.0 - alpha) * x;
  y_lasts[i] = y;
  
  return y;
}
