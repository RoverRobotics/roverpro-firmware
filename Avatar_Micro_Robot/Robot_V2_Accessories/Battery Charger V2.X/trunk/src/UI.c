// File: UI.h
//

//#define TEST_USERINTERFACE
//---------------------------Dependencies---------------------------------------
#include "./UI.h"
#include "./core/StandardHeader.h"    // for ON/OFF and YES/NO macros
#include "./core/Timers.h"

//---------------------------Macros---------------------------------------------
// indicator wiring
#define HEARTBEAT_EN(a)       (_TRISE5 = !(a))
#define HEARTBEAT_PIN         (_RE5)
#define RED_EN(a)             (_TRISD2 = !(a)); (_ODD2 = (a))
#define GREEN_EN(a)           (_TRISD3 = !(a)); (_ODD3 = (a))
#define TURN_RED(a)           (_LATD2 = !(a)); Nop(); Nop() // to give pin time to float up if needed
#define TURN_GREEN(a)         (_LATD3 = !(a)); Nop(); Nop()

// PWM
#define T_PWM                 10   // [ms]
#define RED_LED_RPN           23
#define GREEN_LED_RPN         22

// times and timers
#define HEARTBEAT_TIMER       0
#define HEARTBEAT_TIME        500  // [ms]
#define G_UPDATE_TIMER        1
#define G_UPDATE_TIME         500
#define R_UPDATE_TIMER        2
#define R_UPDATE_TIME         500

//---------------------------Helper Function Prototypes-------------------------
static void RunHeart(void);
static void RunWaitingRoutine(void);
static void RunErringRoutine(void);

//---------------------------Module Variables-----------------------------------
static volatile kUIState current_state = kUIStateWaiting;

//---------------------------Test Harness---------------------------------------
#ifdef TEST_USERINTERFACE
#include "./core/ConfigurationBits.h"

int main(void) {
  TMRS_Init();
  UI_Init();
  // comment in each state to ensure we get the desired behavior
  //UI_set_state(kUIStateWaiting);
  //UI_set_state(kUIStateCharging);
  //UI_set_state(kUIStateDoneCharging);
  UI_set_state(kUIStateErring);
  while (1) {
    UI_Run();
  }
  
  return 0;
}
#endif

//---------------------------Public Function Implementations--------------------
void UI_Init(void) {
  // configure and initialize any pins
  HEARTBEAT_EN(YES); HEARTBEAT_PIN = 0;
  RED_EN(YES); TURN_RED(OFF);
  GREEN_EN(YES); TURN_GREEN(OFF);
}

void UI_Run(void) {
  RunHeart();
  switch (current_state) {
    case kUIStateWaiting: RunWaitingRoutine(); break;
    case kUIStateCharging: break;
    case kUIStateDoneCharging: break;
    case kUIStateErring: RunErringRoutine(); break;
    default: while(1); break; // indicate an error
  }
}

void UI_set_state(const kUIState state) {
  switch (state) {
    case kUIStateWaiting:
      TURN_RED(OFF); TURN_GREEN(OFF);
      current_state = kUIStateWaiting;
      break;
    case kUIStateCharging:
      TURN_GREEN(OFF); TURN_RED(ON);
      current_state = kUIStateCharging;
      break;
    case kUIStateDoneCharging:
      TURN_RED(OFF); TURN_GREEN(ON);
      current_state = kUIStateDoneCharging;
      break;
    case kUIStateErring:
      TURN_GREEN(OFF); TURN_RED(OFF);
      current_state = kUIStateErring;
      break;
  }
}

kUIState UI_state(void) {
  return current_state;
}


void UI_Deinit(void) {
  TURN_RED(OFF);
  TURN_GREEN(OFF);
  HEARTBEAT_PIN = 0;
}

//---------------------------Helper Function Implementations--------------------
static void RunWaitingRoutine(void) {
  static uint8_t last_state = 0;
  // toggle the green LED
  if (TMRS_IsTimerExpired(G_UPDATE_TIMER)) {
    TMRS_StartTimer(G_UPDATE_TIMER, G_UPDATE_TIME);
    TURN_GREEN(last_state);
    last_state ^= 1;
  }
}

static void RunErringRoutine(void) {
  static uint8_t last_state = 0;
  // toggle the red LED
  if (TMRS_IsTimerExpired(R_UPDATE_TIMER)) {
    TMRS_StartTimer(R_UPDATE_TIMER, R_UPDATE_TIME);
    TURN_RED(last_state);
    last_state ^= 1;
  }
}

/*
static void RunGreenBreath(void) {
  static float duty_cycle = DC_MIN;
  static float delta = 0.008;
  
  if (TMRS_IsTimerExpired(G_UPDATE_TIMER)) {
    TMRS_StartTimer(G_UPDATE_TIMER, G_UPDATE_TIME);
    
    duty_cycle += delta;
    if (DC_MAX < duty_cycle) {
      duty_cycle = DC_MAX;
      delta *= -1.0;
    } else if (duty_cycle < DC_MIN) {
      duty_cycle = DC_MIN;
      delta *= -1.0;
    }
  }
  
  PWM_UpdateDutyCycle(kPWM02, duty_cycle);
}
*/

static void RunHeart(void) {
  // toggle a pin to indicate normal operation
  if (TMRS_IsTimerExpired(HEARTBEAT_TIMER)) {
    TMRS_StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
    HEARTBEAT_PIN ^= 1; Nop();
  }
}
