/*******************************************************************************
File: Charger.c

Description: This is the overarching file that encompasses the application-level
  logic of the standalone Lithium-ion battery charger.
  
Notes:
  - intended for BB-2590
    two (2) cells in parallel each comprised of four (4) modules in series
  - TODO: works on custom battery as well?
  - consumes I2C1, I2C2, and I2C3 hardware modules
  - NOT non-blocking or most ideal in terms of clarity
  - note that the method of detecting a battery varies by state
   
  
Responsible Engineer: Stellios Leventis (sleventis@robotex.com)
*******************************************************************************/
#define TEST_CHARGER
//---------------------------Dependencies---------------------------------------
#include "./core/StandardHeader.h"
#include "./core/Timers.h"          // for timers
#include "./core/ADC.h"             // for A/D conversions
#include "./core/TWI.h"             // for communication with IC and battery
#include "./SMBusDevices/BQ24745.h"
#include "./Filters.h"
#include "./UI.h"                   // to update the user interface

//---------------------------Macros---------------------------------------------
// wiring macros
//---charging IC interface pins
#define BQ24745_ACOK_EN(a)    (_TRISB15 = (a)); AD1PCFGL |= (1 << 15) // BUG ALERT: RB9 is analog by default
#define BQ24745_ACOK          _RB15
#define CHARGER_CONNECT_EN(a) (_TRISB14 = !(a)); Nop()
#define CHARGER_CONNECT       _LATB14
#define SIDEA_CONNECT_EN(a)   (_TRISG7 = !(a)); Nop()
#define SIDEA_CONNECT         _LATG7
#define SIDEB_CONNECT_EN(a)   (_TRISG6 = !(a)); Nop()
#define SIDEB_CONNECT         _LATG6
#define BQ24745_EN_EN(a)      (_TRISF3 = !(a)); Nop()
#define BQ24745_EN            _LATF3
// NB: configured as open-drain output, must be on a pin
// that does NOT have analog sensing ability
#define BATTERY_CHARGE_EN_EN(a) (_TRISE4 = !(a)); (_ODE4 = (a))
#define BATTERY_CHARGE_EN(a)    (_LATE4 = !(a));  Nop(); Nop()

//---analog sensing pins (ANx)
#define SIDEA_I_PIN           7     // side A current sensing
#define SIDEA_V_PIN           6     // side A voltage sensing
#define SIDEB_I_PIN           4     // side B current sensing
#define SIDEB_V_PIN           5     // side B voltage sensing
#define WALLWART_V_PIN        13    // wallwart voltage sensing
#define THERMISTOR_V_PIN      3     // thermistor in battery (UNUSED RIGHT NOW)

// times and timers
#define _10ms                 10
#define _100ms                100
#define _1s                   1000
#define _1m                   60000
#define _1h                   3600000

#define CHARGER_REFRESH_TIMER 4
#define CHARGER_REFRESH_TIME  (10*_1s)
#define CHARGE_TIMEOUT_TIMER  5
#define CHARGE_TIMEOUT_TIME   (2*_1h)
#define REVIVE_SAFETY_TIMER   6
#define REVIVE_SAFETY_TIME    (_1m)
#define COOLDOWN_TIMER        7
#define COOLDOWN_TIME         (_100ms)
//#define SMB2_TIMEOUT_TIMER    8
//#define SMB2_TIMEOUT_TIME     (_10ms)

// filters
#define ALPHA                 0.8
#define SIDEA_I_FILTER        0
#define SIDEA_V_FILTER        1
#define SIDEB_I_FILTER        2
#define SIDEB_V_FILTER        3

// battery characterstics
//#define CAPACITY              6     // [A*h], for smallest of our batteries
#define I_REVIVE_MAX          350     // [mA], typically ~(0.1/h)*C
#define I_SIDE_MAX            3000    // the maximum input current to cell
//#define I_CC_STAGE          3000    // [mA], (0.2/h)*C to (0.7/h)*C
#define I_IN_MAX              8000    // [mA], maximum input current to the
                                      // charging IC
#define V_OUT_MAX             16500   // [V], maximum output voltage to an
                                      // individual cell
#define I_CHARGE_DELTA        125     // [mA], overall charge current increment
#define I_CHARGE_MAX          3000//4000    // [mV], the wall-wart can only deliver ~4.5A at 19V

// power management threshold(s)
// maximum allowable current to an individual cell
// [au], 0.01S*(3A*0.05Ohm)*1k ~= 1.5V, (1.5 / 3.30) * 1023=
// [au], minimum current to an individual cell
static const kADCReading kMaxSideCurrent = 465;

// topping charge voltage lower threshold (when to charge a little more)
// (1.6 / (10 + 1.6) * 16.0) / 3.3 * 1023=
static const kADCReading kToppingVoltage = 680;

// (0.1/h)*C, threshold defining a deeply discharged lithium battery
// (3V/cell)*4cells => 12V, (1.6 / (10 + 1.6) * 12) / 3.3 * 1023=
static const kADCReading kReviveVoltage = 514;

// maximum anticipated noise on an A/D line
// (0.050V / 3.3V) * 1023=
static const kADCReading kMaxADCNoise = 15; // NB: trickle current is ~25au

// thresholds to validate the power supply
// (7.5/(75 + 7.5) * 20V / 3.3) *1023=
// (7.5/(75 + 7.5) * 18V / 3.3) *1023= MATH DOESN'T WORK OUT
// NB: the leakage current on the zener diode is on the order of the
// overall current coming in and is affecting the measurement.  
static const kADCReading kMaxWallwartVoltage = 500;
static const kADCReading kMinWallwartVoltage = 400; // getting 455 at ~19V

// current into a battery side below which charging should terminate
// I_chargeTermination = 0.1/h * (C_overallBattery/2cellPerBattery)
// Manufacturer recommends 100mA termination on each side of the battery:
// 0.01S*(0.1A*0.05Ohm)*1k ~= 0.05V, (.05 / 3.30) * 1023= 15.5
static const kADCReading kChargeTerminationCurrent = 16;

//---------------------------Type Definitions-----------------------------------
// possible states
typedef enum {
  kValidatingWallwart = 0,
  kPursuingBattery,
	kAwakening,
	kReviving,  // when reviving deeply-discharged batteries
	kCharging,  // both constant-current and constant-voltage stages
	kInvestigatingOvercurrent,
	kWaiting,
	kHanging,
} kChargerState;

//---------------------------Public Function Prototypes (if we had a header)----
void InitCharger(void);
void RunChargerSM(void);

//---------------------------Helper Function Prototypes-------------------------
static void InitPins(void);
static void ConfigureChargingIC(const uint16_t I_in_max,
                                const uint16_t V_out_max,
                                const uint16_t I_out_max);
static bool IsWallwartValid(void);
static bool IsBatteryValid(void);
static bool IsBatteryVoltageDetected(void);
static bool IsNoCurrentFlowing(void);
static bool IsBatteryRevived(void);
static bool IsOvercurrentDetected(void);
static bool IsBatteryCharged(void);
static bool IsWallwartAttached(void);
static void ResetCharger(void);

//---------------------------Module Variables-----------------------------------
static uint8_t bqdata[2] = {0};
static TWIDevice bq24745 = {
  .address = BQ24745_SLAVE_ADDRESS,
  .subaddress = 0x00,
  .n_data_bytes = 2,  // number of data bytes the device will send us
  .data = bqdata
};

// TODO: limit scope after debugging
extern kChargerState state = kValidatingWallwart;
extern kADCReading temp1 = 0;
extern kADCReading temp2 = 0;
extern kADCReading temp3 = 0;
extern kADCReading temp4 = 0;

//---------------------------Test Harness---------------------------------------
#ifdef TEST_CHARGER
#include "./core/ConfigurationBits.h"
#include "./core/StandardHeader.h"

int main(void) {
  InitCharger();
  //SWDTEN = 1;
  while (1) {
    RunChargerSM();
  	//ClrWdt(); // clear the software WDT
  }
  
  return 0;
}
#endif

//---------------------------Public Function Definitions------------------------
void RunChargerSM(void) {
  static volatile uint16_t I_ch_max = I_REVIVE_MAX;
  
  UI_Run();
  
  // ensure the charger does NOT stay powered while
  // on the battery after the wall-wart is removed
  if (!IsWallwartAttached()) ResetCharger();
  
  // keep the charging IC from timing out
  if (TMRS_IsTimerExpired(CHARGER_REFRESH_TIMER)) {
    TMRS_StartTimer(CHARGER_REFRESH_TIMER, CHARGER_REFRESH_TIME);
    I_ch_max += I_CHARGE_DELTA;
    if (I_CHARGE_MAX < I_ch_max) I_ch_max = I_CHARGE_MAX;
    if (TWI_ErrorHasOccurred(kTWI02)) TWI_Refresh(kTWI02);
    ConfigureChargingIC(I_IN_MAX, V_OUT_MAX, I_ch_max);
  }
  
  switch (state) {
    case kValidatingWallwart:
      if (IsWallwartAttached() && IsWallwartValid()) {
        I_ch_max = I_REVIVE_MAX;
        ConfigureChargingIC(I_IN_MAX, V_OUT_MAX, I_REVIVE_MAX);
        state = kPursuingBattery;
        return;
      } else {
        UI_set_state(kUIStateErring);
        state = kHanging;
        return;
      }
      break;
    case kPursuingBattery:
      if (IsBatteryValid()) {
        UI_set_state(kUIStateCharging);
        BQ24745_EN = 1; // enable the charging IC
        Delay(250);     // BUG ALERT: give some time to ensure a new A/D reading, working regulator
        state = kReviving;
        return;
      }
      break;
    case kReviving:
      // if the battery is sufficiently revived
      if (IsBatteryRevived()) {
        // move on and allow higher-current charging
        I_ch_max = I_CHARGE_MAX;
        ConfigureChargingIC(I_IN_MAX, V_OUT_MAX, I_ch_max);
        Delay(50); // BUG ALERT: give some time to get new A/D reading and to begin regulating for sufficient current to battery?
        state = kCharging;
        return;
      }
      
      // if the battery was pulled, reset
      if (IsNoCurrentFlowing()) {
        Nop();
        Nop();
        ResetCharger();
      }
      break;
    case kCharging:
	  /*
      /*
      // if we detect a short on either side through the voltage sagging
      if ((IIRFilter(SIDEA_V_FILTER, ADC_value(SIDEA_V_PIN), ALPHA) < 
           kReviveVoltage) ||
          (IIRFilter(SIDEB_V_FILTER, ADC_value(SIDEB_V_PIN), ALPHA) < 
          kReviveVoltage)) {
         SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
         BQ24745_EN = 0;
         CHARGER_CONNECT = 0;
         UI_set_state(kUIStateErring);
         state = kHanging;
      }
      */
	  //*/
    
      // prevent too much current from going to an individual cell
      if (IsOvercurrentDetected()) {
        SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
        TMRS_StartTimer(COOLDOWN_TIMER, COOLDOWN_TIME);
        state = kInvestigatingOvercurrent;
        return;
      }
      
      if (IsBatteryCharged()) {
        // disable the charger, so we can sense the voltage on the battery
        BQ24745_EN = 0;
        Delay(1000);
        UI_set_state(kUIStateDoneCharging);
        state = kWaiting;
        return;
      }
      
      // if the battery was pulled, reset
      // NB: IMPORTANT THAT THIS IS LAST, MOST RESTRICTIVE CURRENT CHECK
	  /*
      if (IsNoCurrentFlowing()) {
        Nop();
        Nop();
        ResetCharger();
      }
	*/
      
      break;
    case kInvestigatingOvercurrent:
      Nop();
      Nop();
      if (TMRS_IsTimerExpired(COOLDOWN_TIMER)) {
        if (ADC_value(SIDEA_I_PIN) < kMaxADCNoise &&
            ADC_value(SIDEB_I_PIN) < kMaxADCNoise) {
          // decrement the overall current
          I_ch_max -= I_CHARGE_DELTA;
          if (I_ch_max < I_REVIVE_MAX) I_ch_max = I_REVIVE_MAX;
          ConfigureChargingIC(I_IN_MAX, V_OUT_MAX, I_ch_max);
          SIDEA_CONNECT = 1; SIDEB_CONNECT = 1;
          Delay(50);    // delay long enough to ensure a new A/D reading
          state = kCharging;
          return;
        } else {
          // otherwise something is terribly wrong so brick
          SIDEA_CONNECT = 0; SIDEB_CONNECT = 0;
          CHARGER_CONNECT = 0;
          BQ24745_EN = 0;
          UI_set_state(kUIStateErring);
          state = kHanging;
          return;
        }
      }
      break;
    case kWaiting:
      // if the battery was pulled, reset
      // NB: the only way to check here is to monitor the voltage
      if (!IsBatteryVoltageDetected()) {
        Nop();
        Nop();
        ResetCharger();
      }
      
      // if the battery has sufficiently self-discharged
      if ((ADC_value(SIDEA_V_PIN) < kToppingVoltage) &&
          (ADC_value(SIDEB_V_PIN) < kToppingVoltage)) {
        I_ch_max = I_REVIVE_MAX;
        BQ24745_EN = 1;
        ConfigureChargingIC(I_IN_MAX, V_OUT_MAX, I_ch_max);
        UI_set_state(kUIStateCharging);
        state = kCharging;
        return;
      }
      break;
    case kHanging:
      Nop();
      break;
    default:
      // indicate a design error
      while (1) {
        UI_set_state(kUIStateDoneCharging); UI_Run();
        Delay(1000);
        UI_set_state(kUIStateCharging); UI_Run();
        Delay(1000);
      }
      Nop();
      break;
  }
}

void InitCharger(void) {
  InitPins();
  
  // initialize any dependent modules
  ADC_Init((1 << SIDEA_I_PIN)     |
           (1 << SIDEA_V_PIN)     |
           (1 << SIDEB_I_PIN)     |
           (1 << SIDEB_V_PIN)     |
           (1 << WALLWART_V_PIN)  | 
           (1 << THERMISTOR_V_PIN));
  TWI_Init(kTWI02, kTWIBaudRate100kHz, SMBUS);
  TMRS_Init();
  UI_Init();
  
  TMRS_StartTimer(CHARGER_REFRESH_TIMER, CHARGER_REFRESH_TIME);
  Delay(50);  // allow the A/D conversions buffer to get populated, likely don't need this
  UI_set_state(kUIStateWaiting);
}

//---------------------------Private Function Definitions-----------------------
// Description: Configures the upper limits for the input current, 
//   output charging voltage and output charging current through 
//   an SMBus interface.
// Parameters:
//   const float I_max_in,   the maximum input current in milliamps
//   const float V_max_out,  the maximum output voltage in millivolts
//   const float I_max_out,  the maximum output current in milliamps
// Notes:
//   - see p.23 of battery charger datasheet 'Enable and Disable Charging'
//   - this must be done periodically to avoid a timeout (170s) within the 
//     bq24745
static void ConfigureChargingIC(const uint16_t I_in_max,
                                const uint16_t V_out_max,
                                const uint16_t I_out_max) {
  uint16_t temp = 0;
  uint8_t message[2] = {0};
  const uint16_t kCurrentMask = 0x1F80;   // see p.20-23 of bq24645 datasheet
  const uint16_t kVoltageMask = 0x7FF0;
  
  if (TWI_ErrorHasOccurred(kTWI02)) TWI_Init(kTWI02, kTWIBaudRate100kHz, SMBUS);
  
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

static void InitPins(void) {
  // configure and initialize charging IC interface pin(s)
  BQ24745_ACOK_EN(1);
  CHARGER_CONNECT_EN(1); CHARGER_CONNECT = 1;
  SIDEA_CONNECT_EN(1); SIDEA_CONNECT = 1;
  SIDEB_CONNECT_EN(1); SIDEB_CONNECT = 1;
  // NB: charger must be DISABLED to sense the voltage on the battery
  BQ24745_EN_EN(1); BQ24745_EN = 0;
  BATTERY_CHARGE_EN_EN(1); BATTERY_CHARGE_EN(1);
  Delay(5); // wait for the ACOK pin to settle
}

static bool IsWallwartValid(void) {
  uint16_t temp = ADC_value(WALLWART_V_PIN);
  return ((kMinWallwartVoltage < temp) && (temp < kMaxWallwartVoltage));
}

static bool IsNoCurrentFlowing(void) {
  static uint16_t n_times_here = 0;
  temp1 = IIRFilter(SIDEA_I_FILTER, ADC_value(SIDEA_I_PIN), ALPHA);
  temp2 = IIRFilter(SIDEB_I_FILTER, ADC_value(SIDEB_I_PIN), ALPHA);
  if (1000 < ++n_times_here) {
    n_times_here = 0;
    return ((temp1 < kMaxADCNoise) && (temp1 < kMaxADCNoise));
  }
  return NO;
}

// Description: Returns whether there is voltage at BOTH sides
//   Note that this is only valid if the charger is disabled, otherwise
//   we are just seeing the voltage produced by the buck subcircuit
static bool IsBatteryVoltageDetected(void) {
  return (((kMaxADCNoise < ADC_value(SIDEA_V_PIN)) &&
           (kMaxADCNoise < ADC_value(SIDEB_V_PIN))));          
}

static bool IsBatteryRevived(void) {
  return ((kReviveVoltage < IIRFilter(SIDEA_V_FILTER, ADC_value(SIDEA_V_PIN), ALPHA)) &&
          (kReviveVoltage < IIRFilter(SIDEB_V_FILTER, ADC_value(SIDEB_V_PIN), ALPHA)));
}

static bool IsOvercurrentDetected(void) {
  return ((kMaxSideCurrent < IIRFilter(SIDEA_I_FILTER, ADC_value(SIDEA_I_PIN), ALPHA)) ||
          (kMaxSideCurrent < IIRFilter(SIDEB_I_FILTER, ADC_value(SIDEB_I_PIN), ALPHA)));
}

// Description: Returns whether there is little current flowing on both sides.
//   Additionally filters the A/D sensing to ensure this check is not
//   spuriosly tripped. 
static bool IsBatteryCharged(void) {
  static uint16_t n_times_here = 0;
  temp3 = IIRFilter(SIDEA_I_FILTER, ADC_value(SIDEA_I_PIN), ALPHA);
  temp4 = IIRFilter(SIDEB_I_FILTER, ADC_value(SIDEB_I_PIN), ALPHA);
  if (1000 < ++n_times_here) {
    n_times_here = 0;
    return ((temp3 < kChargeTerminationCurrent) &&
            (temp4 < kChargeTerminationCurrent));
  }
  return NO;
}

static bool IsBatteryValid(void) {
  return (ADC_value(THERMISTOR_V_PIN) < kMaxADCNoise);
}

static bool IsWallwartAttached(void) {
  return BQ24745_ACOK;
}

static void ResetCharger(void) {
  UI_Deinit();
  CHARGER_CONNECT = 0;
  SIDEA_CONNECT = 0;
  SIDEB_CONNECT = 0;
  BQ24745_EN = 0;
  BATTERY_CHARGE_EN(0);
  //Delay(3000);  // make it obvious when we reset
  asm("reset");
}
