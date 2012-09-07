/*=============================================================================
 
File: ESC_Calibrator.ino

Description: This file calibrates a LOSI electronic speed controller (ESC) for
  our current setup.  It employs the PWM library to set neutral, high and low
  pulse-width thresholds (1.5ms, 2ms and 1ms pulse-widths respectively) at a 
  frequency of about 100Hz.

Responsible Engineer: Stellios Leventis  (sleventis@robotex.com)
=============================================================================*/
/*---------------------------Dependencies------------------------------------*/
#include <PWM.h>
#include <Timers.h>

/*---------------------------Macros------------------------------------------*/
#define MAX_DC               20    // 0.20 * 10ms = 2ms pulse width
#define NEUTRAL_DC           15
#define MIN_DC               10
#define PWM_PIN              11    // must be either 3 or 11

// times and timers
#define _100ms               (100)
#define HEARTBEAT_TIMER      0
#define HEARTBEAT_HALFPERIOD (500)
#define TEST_TIMER           1
#define CALIBRATION_TIME    (50*_100ms)

#define HEARTBEAT_LED        13    // the onboard LED

/*---------------------------Public Function Definitions---------------------*/
void setup() {
  pinMode(HEARTBEAT_LED, OUTPUT); digitalWrite(HEARTBEAT_LED, LOW);
  
  InitPWM(1250);                          // T = 1250 * 8us => 10ms => f = 100Hz
  UpdateDutyCycle(PWM_PIN, NEUTRAL_DC);  // step 0
  
  StartTimer(TEST_TIMER, (80 * _100ms));
}

void loop() {
  if (IsTimerExpired(TEST_TIMER)) {
    StartTimer(TEST_TIMER, CALIBRATION_TIME);
    static unsigned char procedure_step = 1;
    switch (procedure_step++) {
      case 1:
        UpdateDutyCycle(PWM_PIN, MAX_DC);
        break;
      case 2:
        UpdateDutyCycle(PWM_PIN, MIN_DC);
        break;
      case 3:
        UpdateDutyCycle(PWM_PIN, NEUTRAL_DC);
        digitalWrite(HEARTBEAT_LED, HIGH);
        while (1);  // stop here and wait to be unplugged
        break;
    }
  }
  
}

