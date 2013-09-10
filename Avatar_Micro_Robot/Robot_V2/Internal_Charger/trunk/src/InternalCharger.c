/*==============================================================================
File: InternalCharger.c

Description: Application-level file for the internal battery charger.  This is
  the quick, first-prototype version.

Notes:
  - intended for BB-2590
    two (2) cells in parallel each comprised of four (4) modules in series
  
  
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

// analog sensing pins (ANx)
// TODO: FIX THIS!!! #define V_WALLWART          
#define SIDEA_I_PIN           4     // side A current sensing
#define SIDEA_V_PIN           5     // side A voltage sensing
#define SIDEB_I_PIN           6     // side B current sensing
#define SIDEB_V_PIN           7     // side B voltage sensing

// power management threshold(s)
// maximum allowable current to an individual cell
// [au], 0.01S*(3A*0.05Ohm)*1k ~= 1.5V, (1.5 / 3.30) * 1023=
// [au], minimum current to an individual cell
const kADCReading kMaxCellCurrent = 465;

// minimum cell current draw after
// which charging should terminate
// (100mA/cell*4cells) => 400mA
// 0.01S*(*0.05Ohm)*1k ~= 1.5V, (1.5 / 3.30) * 1023=
//const kADCReading kMinCellCurrent = 8;

// topping charge voltage threshold (when to charge a little more)
// (1.6 / (10 + 1.6) * 16.5) / 3.3 * 1023=
//const kADCReading kToppingVoltage = 705;

// (0.1/h)*C, threshold defining a deeply discharged lithium battery
// (3V/cell)*4cells => 12V, (1.6 / (10 + 1.6) * 12) / 3.3 * 1023=
const kADCReading kReviveVoltage = 514;

// maximum anticipated noise on an A/D line
// (0.100V / 3.3V) * 1023=
const kADCReading kMaxADCNoise = 31;

// filters
#define ALPHA                 0.2
#define SIDEA_I_FILTER        0
#define SIDEA_V_FILTER        1
#define SIDEB_I_FILTER        2
#define SIDEB_V_FILTER        3

// charging IC interface pins
#define BQ24745_ACOK_EN(a)    (_TRISB9 = (a)); AD1PCFGL |= (1 << 9) // BUG ALERT: RB9 is analog by default
#define BQ24745_ACOK          _RB9
#define CHARGER_CONNECT_EN(a) (_TRISB10 = !(a)); Nop()
#define CHARGER_CONNECT       _LATB10
#define SIDEA_CONNECT_EN(a)   (_TRISB11 = !(a)); Nop()
#define SIDEA_CONNECT         _LATB11
#define SIDEB_CONNECT_EN(a)   (_TRISB12 = !(a)); Nop()
#define SIDEB_CONNECT         _LATB12
#define _5V_EN_EN(a)          (_TRISB13 = !(a)); Nop()
#define _5V_EN                _LATB13
#define BQ24745_EN_EN(a)      (_TRISB14 = !(a)); Nop()
#define BQ24745_EN            _LATB14

// timer(s)
#define _10ms                 10
#define _100ms                100
#define _1s                   1000
#define _1m                   60000
#define _1h                   3600000
#define HEARTBEAT_TIMER       0
#define HEARTBEAT_TIME        (5*_100ms)
#define AWAKENING_TIMER       1
#define AWAKENING_TIME        (_1s)
#define CHARGER_REFRESH_TIMER 2
#define CHARGER_REFRESH_TIME  (10*_1s)
#define CHARGE_TIMEOUT_TIMER  3
#define CHARGE_TIMEOUT_TIME   (2*_1h)
#define REVIVE_SAFETY_TIMER   4
#define REVIVE_SAFETY_TIME    (_1m)
#define COOLDOWN_TIMER        5
#define COOLDOWN_TIME         (_100ms)
#define BALANCE_TIMER         6
#define BALANCE_TIME          (10*_1s)
#define BALANCE_CHECK_TIMER   7
#define BALANCE_CHECK_TIME    (10*_1s)

// powerboard protocol
#define MY_SLAVE_ADDRESS      0x0C
#define REQ_CHARGER_ALIVE     0xCA
#define RESP_ALIVE            0xDA
#define ERROR_RESPONSE        0xFF

// battery characterstics
#define N_SIDES               2
//#define CAPACITY              6     // [A*h], for smallest of our batteries
#define I_REVIVE_MAX          2000    // [mA], typically ~(0.1/h)*C
#define I_SIDE_MAX            3000    // the maximum input current to cell
//#define I_CC_STAGE          3000    // [mA], (0.2/h)*C to (0.7/h)*C
#define I_IN_MAX              8000    // [mA], maximum input current to the
                                      // charging IC
#define V_MAX_OUT             16500   // [V], maximum output voltage to an
                                      // individual cell
#define I_CHARGE_DELTA        125     // [mA], overall charge current increment

/*---------------------------Type Definitions---------------------------------*/
// possible states
typedef enum {
	kAwakening = 0,
	kReviving,  // when reviving deeply-discharged batteries
	kCharging,  // both constant-current and constant-voltage stages
	kBalancing,
	kCheckingBalance,
	kInvestigatingOvercurrent,
} kChargerState;

/*---------------------------Helper Function Prototypes-----------------------*/
static void InitCharger(void);
static void InitPins(void);
static void ConfigureChargingIC(const uint16_t I_in_max,
                                const uint16_t V_out_max,
                                const uint16_t I_out_max);
static void RunChargerSM(void);
static uint8_t MyI2C1Response(const uint8_t command,
                              const uint8_t response_index);
float IIRFilter(const uint8_t i, const float x, const float alpha);

/*---------------------------Module Variables---------------------------------*/
static uint8_t bqdata[2] = {0};
static TWIDevice bq24745 = {
  .address = BQ24745_SLAVE_ADDRESS,
  .subaddress = 0x00,
  .n_data_bytes = 2,  // number of data bytes the device will send us
  .data = bqdata
};

static volatile uint16_t I_ch_max = I_REVIVE_MAX;

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_INTERNALCHARGER
#include "./core/ConfigurationBits.h"
int main(void) {
  InitCharger();
  _SWDTEN = 1;
  while (1) {
    ClrWdt();
    RunChargerSM();
  }
  
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
  
  // BUG ALERT: the internal charger stays powered after driving off the dock
  // if we drove off the dock
  if (!BQ24745_ACOK) {
    // disconnect what is keeping us powered
    SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
    
    // let the watch dog timer expire,
    // allowing a reset to indicate an immediately 
    // visible failure of this working
    while (1) {};
  }
  
  // TODO: suspend charging if the temperature is too extreme
  
  // keep the charging IC from timing out
  if (TMRS_IsTimerExpired(CHARGER_REFRESH_TIMER)) {
    TMRS_StartTimer(CHARGER_REFRESH_TIMER, CHARGER_REFRESH_TIME);
    I_ch_max += I_CHARGE_DELTA;
    if ((I_SIDE_MAX * N_SIDES) < I_ch_max) I_ch_max = I_SIDE_MAX * N_SIDES;
    if (TWI_ErrorHasOccurred(kTWI02)) TWI_Refresh(kTWI02);
    ConfigureChargingIC(I_IN_MAX, V_MAX_OUT, I_ch_max);
  }

  switch (state) {
    case kAwakening:
      if (TMRS_IsTimerExpired(AWAKENING_TIMER)) {
        BQ24745_EN = 1;
        CHARGER_CONNECT = 1;
        ConfigureChargingIC(I_IN_MAX, V_MAX_OUT, I_ch_max);
        Delay(50);    // allow the charging IC to begin regulating
        SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
        Delay(50);    // delay long enough to ensure a new A/D reading
        TMRS_StartTimer(REVIVE_SAFETY_TIMER, REVIVE_SAFETY_TIME);
        state = kReviving;
      }
      break;
    case kReviving:
      // if the battery is still dead-dead
      if (TMRS_IsTimerExpired(REVIVE_SAFETY_TIMER) &&
          (IIRFilter(SIDEA_V_FILTER, ADC_value(SIDEA_V_PIN), ALPHA) < kMaxADCNoise || 
           IIRFilter(SIDEB_V_FILTER, ADC_value(SIDEB_V_PIN), ALPHA) < kMaxADCNoise)) {
        // restart in case there is something wrong
        SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
        TMRS_StartTimer(AWAKENING_TIMER, _1m);
        state = kAwakening;
      }
      
      // if the battery is sufficiently revived
      if (kReviveVoltage < IIRFilter(SIDEA_V_FILTER, ADC_value(SIDEA_V_PIN), ALPHA) &&
          kReviveVoltage < IIRFilter(SIDEB_V_FILTER, ADC_value(SIDEB_V_PIN), ALPHA)) {
        // move on and allow higher-current charging
        I_ch_max = I_SIDE_MAX*N_SIDES;
        state = kCharging;
      }
      break;
    case kCharging:
      // prevent too much current from going to an individual cell
      if (kMaxCellCurrent < IIRFilter(SIDEA_I_FILTER, ADC_value(SIDEA_I_PIN), ALPHA) ||
          kMaxCellCurrent < IIRFilter(SIDEB_I_FILTER, ADC_value(SIDEB_I_PIN), ALPHA)) {
        SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
        TMRS_StartTimer(COOLDOWN_TIMER, COOLDOWN_TIME);
        state = kInvestigatingOvercurrent;
        return;
      }
      
      // prevent one cell from charging the other by
      // giving the less-charged cell time to catch up
      if (IIRFilter(SIDEA_I_FILTER, ADC_value(SIDEA_I_PIN), ALPHA) < kMaxADCNoise) {
        SIDEA_CONNECT = 0; SIDEB_CONNECT = 1;
        TMRS_StartTimer(BALANCE_TIMER, BALANCE_TIME);
        state = kBalancing;
        return;
      }
      if (IIRFilter(SIDEB_I_FILTER, ADC_value(SIDEB_I_PIN), ALPHA) < kMaxADCNoise) {
        SIDEA_CONNECT = 1; SIDEB_CONNECT = 0;
        TMRS_StartTimer(BALANCE_TIMER, BALANCE_TIME);
        state = kBalancing;
        return;
      }
      break;
    case kBalancing:
      // prevent too much current from going to an individual cell
      if (kMaxCellCurrent < IIRFilter(SIDEA_I_FILTER, ADC_value(SIDEA_I_PIN), ALPHA) ||
          kMaxCellCurrent < IIRFilter(SIDEB_I_FILTER, ADC_value(SIDEB_I_PIN), ALPHA)) {
        SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
        TMRS_StartTimer(COOLDOWN_TIMER, COOLDOWN_TIME);
        state = kInvestigatingOvercurrent;
        return;
      }
      
      // connect then wait to get a valid sample
      if (TMRS_IsTimerExpired(BALANCE_TIMER)) {
        TMRS_StartTimer(BALANCE_CHECK_TIMER, BALANCE_CHECK_TIME);
        SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
        state = kCheckingBalance;
        return;
      }
      break;
    case kCheckingBalance:
      if (TMRS_IsTimerExpired(BALANCE_CHECK_TIMER)) {
        // if one side is still charging the other, disconnect it
        if (IIRFilter(SIDEA_I_FILTER, ADC_value(SIDEA_I_PIN), ALPHA) < kMaxADCNoise) {
          SIDEA_CONNECT = 0; SIDEB_CONNECT = 1;
          TMRS_StartTimer(BALANCE_TIMER, BALANCE_TIME);
          state = kBalancing;
        } else if (IIRFilter(SIDEB_I_FILTER, ADC_value(SIDEB_I_PIN), ALPHA) < kMaxADCNoise) {
          SIDEA_CONNECT = 1; SIDEB_CONNECT = 0;
          TMRS_StartTimer(BALANCE_TIMER, BALANCE_TIME);
          state = kBalancing;
        } else {  // otherwise try charging both again
          state = kCharging;
        }
        return;
      }
      break;
    case kInvestigatingOvercurrent:
      if (TMRS_IsTimerExpired(COOLDOWN_TIMER)) {
        if (ADC_value(SIDEA_I_PIN) < kMaxADCNoise &&
            ADC_value(SIDEB_I_PIN) < kMaxADCNoise) {
          // decrement the overall current to the
          // limit that the robot will stay powered
          I_ch_max -= I_CHARGE_DELTA;
          if (I_ch_max < I_REVIVE_MAX) I_ch_max = I_REVIVE_MAX;
          ConfigureChargingIC(I_IN_MAX, V_MAX_OUT, I_ch_max);
          SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
          Delay(50);    // delay long enough to ensure a new A/D reading
          state = kCharging;
          return;
        } else {
          // otherwise something is terribly wrong so reset
          SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
          CHARGER_CONNECT = 0;
          BQ24745_EN = 0;
          state = kAwakening;
          return;
        }
      }
      break;
  }
}


/*
Description: Configures the upper limits for the input current, 
  output charging voltage and output charging current through 
  an SMBus interface.
Parameters:
  const float I_max_in,   the maximum input current in milliamps
  const float V_max_out,  the maximum output voltage in millivolts
  const float I_max_out,  the maximum output current in milliamps
Notes:
  - see p.23 of battery charger datasheet 'Enable and Disable Charging'
  - this must be done periodically to avoid a timeout (170s) within the bq24745
*/
static void ConfigureChargingIC(const uint16_t I_in_max,
                                const uint16_t V_out_max,
                                const uint16_t I_out_max) {
  uint16_t temp = 0;
  uint8_t message[2] = {0};
  const uint16_t kCurrentMask = 0x1F80;   // see p.20-23 of bq24645 datasheet
  const uint16_t kVoltageMask = 0x7FF0;
  
  // configure the maximum input current
  temp = ((I_in_max >> 1) & kCurrentMask);// mask away ingored bits
  message[0] = (temp >> 8);               // high-byte
  message[1] = (temp & 0xff);             // low-byte
  bq24745.subaddress = BQ24745_INPUT_CURRENT;
  if (TWI_IsBusIdle(kTWI02)) TWI_WriteData(kTWI02, &bq24745, message);
  Delay(5);
  
  // configure the maximum charging voltage
  temp = (V_out_max & kVoltageMask);
  message[0] = (temp >> 8);
  message[1] = (temp & 0xff);
  bq24745.subaddress = BQ24745_CHARGE_VOLTAGE;
  if (TWI_IsBusIdle(kTWI02)) TWI_WriteData(kTWI02, &bq24745, message);
  Delay(5);
  
  // configure the maximum charging current
  temp = (I_out_max & kCurrentMask);
  message[0] = (temp >> 8);
  message[1] = (I_out_max & 0xff);
  bq24745.subaddress = BQ24745_CHARGE_CURRENT;
  if (TWI_IsBusIdle(kTWI02)) TWI_WriteData(kTWI02, &bq24745, message);
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
  SIDEA_CONNECT_EN(1); SIDEA_CONNECT = 1;
  SIDEB_CONNECT_EN(1); SIDEB_CONNECT = 1;
  _5V_EN_EN(1); _5V_EN = 1; // NB: TRIS must be set to high-Z to turn off!
  BQ24745_EN_EN(1); BQ24745_EN = 1;
  
  // configure and initialize indicator(s)
  HEARTBEAT_EN(1); HEARTBEAT_PIN = 0;
  Delay(5); // wait for the ACOK pin to settle
}


static void InitCharger(void) {
  InitPins();
  
  // initialize any dependent modules
  uint16_t ad_bitmask = ((1 << SIDEA_I_PIN) |
                         (1 << SIDEA_V_PIN) |
                         (1 << SIDEB_I_PIN) |
                         (1 << SIDEB_V_PIN));
  ADC_Init(ad_bitmask);
  I2C1_UserProtocol = MyI2C1Response;
  TWISlave_Init(kTWISlave01, MY_SLAVE_ADDRESS, I2C);
  TWI_Init(kTWI02, kTWIBaudRate100kHz, SMBUS);
  TMRS_Init();
  
  // prime any timers that require it
  TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
  TMRS_StartTimer(AWAKENING_TIMER, AWAKENING_TIME);
  TMRS_StartTimer(CHARGER_REFRESH_TIMER, 0);
}


float IIRFilter(const uint8_t i, const float x, const float alpha) {
  // first-order infinite impulse response (IIR) filter
  // (aka a 'leaky integrator')
  #define MAX_NUM_FILTERS     8 // maximum number of filters
  static float yLasts[MAX_NUM_FILTERS] = {0};
  float y = alpha * yLasts[i] + (1.0 - alpha) * x;
  yLasts[i] = y;
  
  return y;
}
