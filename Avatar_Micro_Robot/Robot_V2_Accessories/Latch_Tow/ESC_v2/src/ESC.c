/*=============================================================================
File: ESC.c

Notes:
  - employs six-step trapezoidal (aka 120°) commutation
  - omega_mechanical = 2 / n_motor_poles * omega_electrical
  
  //- employs a symmetrical PWM driving scheme (PWM's both high and low-side
    transistors)
  // - employs a majority filter to help reject brief noise spikes
    (majority filter ?= a back-EMF sensing method based on a nonlinear digital filter)  
  - TODO: consider using an IIR filter
  - takes precaution against "shoot-through" current possible during the 
    transition between transistor states via the control sequence.  In addition, 
    during a transition, the proper amount of time for the high-side driver to 
    turn off MUST be allowed to elapse before the low-side driver is activated. 
  
  - the samples acquired by the ADC may be affected by the resonant transition 
    voltages caused by the PWM switching frequency as well as kickback currents
    produced by winding de-energizations.  To help mitigate these two main 
    sources of noise, we simultaneously sample the back-EMF signals at a sampling
    rate equal to the PWM reload frequency??
  
  - samplePointInTime = f(omega_motor)??? but how?
    - the BEMF waveform of the motor varies as both a function of position and speed
    - at low speeds, the device samples the BEMF signals at 50% of 
      the PWM ON time
    - at the highest speed of 100% duty cycle, the devices sample the BEMF 
      signals at 75% of the PWM ON time

Theory of Operation:
  During each commutation cycle, one phase is left undriven.  This undriven 
  phase can be inferred via the difference in back EMF sensed on that pin.
  
Characteristics:  
  startup ------------------ blind startup
  back-EMF sensing method -- comparing the BEMF Voltage to the Motor Neutral Point
  filter ------------------- digital filter (majority function)

    
See also: 
  - http://www.microchip.com/stellent/groups/SiteComm_sg/documents/DeviceDoc/en543044.pdf
  
=============================================================================*/
#define TEST_ESC
/*---------------------------Dependencies------------------------------------*/
#include "./ESC.h"
#include "./StandardHeader.h" // for MCU P/N header file, CCW/CW macros, etc
//#include "./PPS.h"            // to dynamically select PWM pins
//#include "./PWM.h"            // to provide speed control
#include "./ADC.h"            // to sense current

/*---------------------------Macros------------------------------------------*/
// A_SCALED, B_SCALED, C_SCALED (TODO: these should be comparator inputs)
#define A_SCALED_ANALOG_PIN     0
#define B_SCALED_ANALOG_PIN     1
#define C_SCALED_ANALOG_PIN     2

// V_STAR
#define V_STAR_ANALOG_PIN       3

// Commutation Transistors (assumed exclusive use of PORTD)

// Power Bus Current Sensing
#define CURRENT_ANALOG_PIN      15
#define MAX_NORMAL_CURRENT      800     // [au], 

// PWM Pin(s)
//#define A_HI_RP_PIN           11
//#define B_HI_RP_PIN           24
//#define C_HI_RP_PIN           23
//#define A_LO_RP_PIN           22
//#define B_LO_RP_PIN           25
//#define C_LO_RP_PIN           20
//#define T_PWM                 100     // [us], period of the PWM signal

// Other                              
#define MAX_COMMUTATION_DURATION  1023//60000 //1024//(1 << 10) // maximum time after which we can expect that rotor is no longer moving, in units of Timer ticks???
#define NUM_PHASES                6

#define TIMER4_PRESCALER          0b01//0b11  // configure the prescaler to divide-by-256 (see p.166)

// number of commutations performed during startup
#define NUM_STARTUP_COMMUTATIONS  8
// number of milliseconds to lock rotor in first commutation step before the timed startup sequence is initiated
#define STARTUP_LOCK_DELAY        10000       

/*---------------------------Helper Function Prototypes----------------------*/
static void ConfigureControlPins(void);
static void ConfigureComparators(void);
static void ConfigureTimer4(void);
static void Commutate(unsigned char *pCurrentPhase, 
                      unsigned int *pCommutationDuration);
static void UpdatePhaseDelayCounter(unsigned int commutationDuration);
static unsigned char IsZeroCrossingDetected(unsigned char currentPhase);
static void UpdateControlPins(unsigned char currentPhase);
static unsigned char IsMotorStalled(unsigned int commutationDuration);
static void PrimeMotor(void);
static void DelayForStartup(unsigned int dot1Milliseconds);

/*---------------------------Constants---------------------------------------*/
/*
Description: This lookup table is used to write the commutation state to 
  the control pins.  It assumes exclusive use of a port and that the high-side
  and low-side commutation pins are grouped as follows: 
              control pin: C_LO B_LO A_LO C_HI B_HI A_HI
              port bit:       5    4    3    2    1    0
              
Notes:
  - See ../SupportingFiles/CurrentFlowSequence.jpg
  - The high-side and low-side transistor sequences
    must be written in a manner that protects against "shoot-through."
    As we step through the states, this should NOT be an issue.  There are 
    never two contiguous states where both the high and low-side of the 
    same transistor pair are on.
*/
const unsigned char kDriveTable[NUM_PHASES] = { 
  0x00, 0x14, 0x0C, 0x0A, 0x22, 0x21, 0x11, 0x00
};

/*
Description: This array holds the intercommutation delays used during startup
  in units of 0.1milliseconds?.  Note the acceleration (borrowed from 
  AVR444 application note).
*/
const unsigned char kStartupDelays[NUM_STARTUP_COMMUTATIONS] = {
  200, 150, 100, 80, 70, 65, 60, 55
};
  
/*
// phase transistor pin assignments
#define UL    PB5 // phase U, low-side enable switch
#define UH    PB4 // phase U, high-side enable switch
#define VL    PB3 // phase V, low-side enable switch
#define VH    PB2 // phase V, high-side enable switch
#define WL    PB1 // phase W, low-side enable switch
#define WH    PB0 // phase W, high-side enable switch

// CCW rotation commutation step drive patterns
#define DRIVE_PATTERN_STEP1_CCW      ((1 << UL) | (1 << VH))
#define DRIVE_PATTERN_STEP2_CCW      ((1 << UL) | (1 << WH))
#define DRIVE_PATTERN_STEP3_CCW      ((1 << VL) | (1 << WH))
#define DRIVE_PATTERN_STEP4_CCW      ((1 << VL) | (1 << UH))
#define DRIVE_PATTERN_STEP5_CCW      ((1 << WL) | (1 << UH))
#define DRIVE_PATTERN_STEP6_CCW      ((1 << WL) | (1 << VH))

// CW rotation commutation step drive patterns
#define DRIVE_PATTERN_STEP1_CW      ((1 << VH) | (1 << WL))
#define DRIVE_PATTERN_STEP2_CW      ((1 << UH) | (1 << WL))
#define DRIVE_PATTERN_STEP3_CW      ((1 << UH) | (1 << VL))
#define DRIVE_PATTERN_STEP4_CW      ((1 << WH) | (1 << VL))
#define DRIVE_PATTERN_STEP5_CW      ((1 << WH) | (1 << UL))
#define DRIVE_PATTERN_STEP6_CW      ((1 << VH) | (1 << UL))
*/

/*
Description: This lookup table is used to implement a majority filter
  on the Back-EMF (BEMF) detection.  See Table 3, p.6 of Microchip
  Application Note AN1175.
*/
const unsigned char kBemfFilterTable[64] = {
  0,   2,  4,  6,  8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30,
  32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62,
   0,  2,  4,  6,  8, 10, 12, 14, 16, 18,  1, 22,  1, 26, 28, 30,
  32, 34, 36, 38,  1, 42, 44, 46,  1,  1,  1, 54, 56, 58, 60, 62
};

/*---------------------------Module Variables--------------------------------*/
// Spike-Rejection Filter variables
static signed int phaseDelayCounter;
static unsigned int phaseDelayFilter = 0; // TODO: what should this be initialized to???
static volatile unsigned char nextCommutationStep = 0;
static volatile unsigned char nextDrivePattern = 0;

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_ESC
#include "./Timers.h"
#include "./ConfigurationBits.h"

// Power Bus
#define CONFIG_POWER_BUS(a)   _TRISD9 = (a); Nop(); Nop()
#define TURN_POWER_BUS(a)     _LATD9 = (a); Nop(); Nop()

// Heartbeat Indicator
#define CONFIGURE_HEARTBEAT_PIN(a)   (_TRISE5 = (a))
#define HEARTBEAT_PIN         (_RE5)

// Times and Timers
#define _100ms                100
#define HEARTBEAT_TIMER       0
#define HEARTBEAT_TIME        (10*_100ms)

int main(void) {
	InitESC();

  InitTimers();
	StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
	
  while (1) {
    /*
    // test simple commutation: slowly step through phases ensuring we 
    // reach each phase in order
    // note: since slow, this will appear to short approx 0.6 Ohm coils.
    //   make sure power supply is current-limited
    if (IsTimerExpired(HEARTBEAT_TIMER)) {
      StartTimer(HEARTBEAT_TIMER, HEARTBEAT_TIME);
      HEARTBEAT_PIN ^= 1;
      
      // note: do NOT ConfigureTimer4() for this test
      
      static unsigned char dummyCurrentPhase = 1;
      static unsigned int dummyCommutationDuration = 0;
      Commutate(&dummyCurrentPhase, &dummyCommutationDuration);
    }
    */
    
    // test zero-crossing-based driving: prime the motor then let the ISR take over
    
    // test noise-filter: step through phases as fast as possible
    
    // test variable speed: speed up and slow down
    
    // test variable direction: move full CCW, then full speed CW

	}

	return 0;
}
#endif
/*---------------------------End Test Harness--------------------------------*/
/*---------------------------Interrupt Service Routines (ISRs)---------------*/
void  __attribute__((__interrupt__, auto_psv)) _T4Interrupt(void) {
  _T4IF = 0;                                  // clear the source of the interrupt  
  T4CONbits.TCKPS = 0b00;//0b01;		                  // begin scanning more quickly

  static unsigned char currentPhase = 1;
  static unsigned int commutationDuration = 0;
  
  commutationDuration++;
  
  if (IsZeroCrossingDetected(currentPhase)) {
    /*
    // count down the phase to commutation?, ensuring we commutate at an appropriate electrical angle? 
    if ((phaseDelayCounter--) == 0) {
      Commutate(&currentPhase, &commutationDuration);                            
      HEARTBEAT_PIN ^= 1;
    }
    */
    HEARTBEAT_PIN ^= 1;
    Commutate(&currentPhase, &commutationDuration);
  }
  
  // if the motor is stalled, commutate anyways to unlock
  if (IsMotorStalled(commutationDuration)) {
    Commutate(&currentPhase, &commutationDuration);
  }
  
}


/*---------------------------Public Function Definitions---------------------*/
void InitESC(void) {
  #ifdef TEST_ESC
    CONFIGURE_HEARTBEAT_PIN(OUTPUT); HEARTBEAT_PIN = 0;
  #endif
  
  unsigned int analogBitMask = ((1 << A_SCALED_ANALOG_PIN) |
                                (1 << B_SCALED_ANALOG_PIN) |
                                (1 << C_SCALED_ANALOG_PIN) |
                                (1 << V_STAR_ANALOG_PIN)   | 
                                (1 << CURRENT_ANALOG_PIN) );
  InitADC(analogBitMask);
  //InitPWM(BLDCM_PWM_PIN, T_PWM);
  
	ConfigureControlPins();
  //ConfigureComparators();
  
  #ifdef TEST_ESC
   CONFIG_POWER_BUS(OUTPUT); TURN_POWER_BUS(ON); // TODO: leave this for another module to implement
  #endif
  
  PrimeMotor();
  
  // begin sensorless commutation
  ConfigureTimer4();
  //---set filteredTimeSinceCommutation to the time to the next commutation
  filteredTimeSinceCommutation = kStartupDelays[NUM_STARTUP_COMMUTATIONS - 1] * (STARTUP_DELAY_MULTIPLIER / 2);
}


void TurnMotor(unsigned char direction, unsigned char speed) {
  if (speed == 0) TURN_POWER_BUS(OFF);
  
  /*  
  if (direction == CW) {
  } else if (direction == CCW) {
  }
  */
}


/*---------------------------Helper Function Definitions---------------------*/
static void Commutate(unsigned char *pCurrentPhase, 
                      unsigned int *pCommutationDuration) {
  T4CONbits.TCKPS = TIMER4_PRESCALER;            // go back to scanning at a slower rate
  
  // change timer prescale to divide-by-1??? blank filtering during commutation time?
  UpdateControlPins(*pCurrentPhase);             // lookup the relevant control pin configuration
  //UpdateDutyCycle(desired_speed_as_percent);   // lookup the current duty cycle
  
  // hook up the next comparator //CM2CON0 = kCM2CON0[current_phase];
  UpdatePhaseDelayCounter(*pCommutationDuration);
  
  // reset the commutation timeout timer
  (*pCommutationDuration) = 0;
  
  // advance to the next phase
  if (NUM_PHASES < ++(*pCurrentPhase)) (*pCurrentPhase) = 1;
 
  /*
  if (direction == CW) {
    // increment the current_phase
    if (NUM_PHASES < ++current_phase) current_phase = 1;
  } else if (direction == CCW) {
    // invert the polarity of the comparator if in reverse
    CM2CON0 ^= 0x10;                          
    // decrement the current_phase
    if ((--current_phase == 0)) current_phase = NUM_PHASES;
  }
  */
}


// TODO: understand this
static void UpdatePhaseDelayCounter(unsigned int commutationDuration) {
  static unsigned int phaseDelay = 0;       // (only a temporary variable)
  
  phaseDelayFilter += commutationDuration;  // perform a 32-point moving average of the commutation
  phaseDelay = phaseDelayFilter / 32;       // duration and set the phase_delay to the average
  phaseDelayFilter -= phaseDelay;
  phaseDelayCounter = phaseDelay / 2;       // set the commutation time to halfway between zero crossings
  // TODO: phaseDelayCounter actually = f(speed, position)
}

/*
Notes:
  - It takes approximately 464Ohm*50e-12F => 0.25ns for a pin on the MCU to change states
    (figure 29-2, p.321).  Our instruction cycle is (2 / f_osc) => (2 / (32MHz)) => 0.6ns
    so one Nop() after writing a 'clear' state to all the pins should suffice
  - The turn-off delay time (t_D(off)) for the low-side transistors (AO4468) can be as long as
    19ns (see datasheet p.2).
*/
static void UpdateControlPins(unsigned char currentPhase) {
  unsigned int temp = kControlTable[currentPhase];
  temp |= (1 << 9); // make sure we don't turn off the power bus.  TODO: should remove this
  LATD = temp;
  // configure the last PWM pin as digital output and initialize to OFF
  // configure the new PWM pin
  // PWM the current low pin
}  

/*
Function: ConfigureComparators()
Description: Configures the comparators to implement back-EMF sensing 
  used as required for the PCBA
*/
static void ConfigureComparators(void) {
  CVRCONbits.CVRSS = 1;   // configure the reference source as CVRSRC = (VREF+) – (VREF-) => V_STAR - 0
  // SHOULD NOT NEED THIS, but break out to a pad just in case CVRCONbits.CVROE = 1;   // output the CVREF on the CVREF pin, TODO: is this necessary?
  CVRCONbits.CVREN = 1;   // turn the CVREF circuit ON
  
  
  // output the result on the CxOUT pin
  CM1CONbits.COE = 1;
  CM2CONbits.COE = 1;
  CM3CONbits.COE = 1;
  
  // do NOT invert the output (this will programmatically change) 
  CM1CONbits.CPOL = 0; 
  CM2CONbits.CPOL = 0;
  CM3CONbits.CPOL = 0;
  
  // connect the non-inverting input to CVREF
  CM1CONbits.CREF = 1;
  CM2CONbits.CREF = 1;
  CM3CONbits.CREF = 1;
  
  // connect the inverting input to pin CxINB
  CM1CONbits.CCH = 0b00;
  CM2CONbits.CCH = 0b00;
  CM3CONbits.CCH = 0b00;
  
  // enable the comparator
  CM1CONbits.CEN = 1;
  CM2CONbits.CEN = 1;
  CM3CONbits.CEN = 1;
}

static void ConfigureTimer4(void) {
  // using as 16-bit timer
	T4CONbits.TCKPS = TIMER4_PRESCALER;		// configure the prescaler
	PR4 = 0xffff;         		// configure the timer period
	_T4IF = 0;	              // ensure we begin with the Interrupt Flag cleared
	_T4IE = 1;                // enable interrupts
	T4CONbits.TON = 1;				// turn on the timer
}

static void ConfigureControlPins(void) {
  // configure control pins and initialize to OFF
  TRISD &= ~((1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
  PORTD &= ~((1 << 5) | (1 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
}

/*
Description: Compares the BEMF voltage to the inferred motor neutral point.
  Sensing zero-crossings was chosen largely for simplicity and because
  closed-loop operation near zero speed is NOT required.
  
Notes:
  - provided the speed is greater than zero, there are only two positions
    per electrical cycle when the BEMF of a phase is zero.  These positions 
    can be distinguished by the slope of the BEMF through the zero crossings
*/
static unsigned char IsZeroCrossingDetected(unsigned char currentPhase) {
  // TODO: this should be removed once comparators are available
  // TODO: confirm slope > or <
  unsigned char comparatorResult = 0;
  switch (currentPhase) {
    case 1:
      // phase C and B should be energized, so look for phase A to cross V_star
      comparatorResult = (GetADC(V_STAR_ANALOG_PIN) > GetADC(A_SCALED_ANALOG_PIN));
      break;
    case 2:
      // phase C and A should be energized, so look for phase B to cross V_star
      comparatorResult = (GetADC(V_STAR_ANALOG_PIN) < GetADC(B_SCALED_ANALOG_PIN));
      break;
    case 3:
      // phase B and A should be energized, so look for phase C to cross V_star
      comparatorResult = (GetADC(V_STAR_ANALOG_PIN) > GetADC(C_SCALED_ANALOG_PIN));
      break;
    case 4:
      // phase B and C should be energized, so look for phase A to cross V_star
      comparatorResult = (GetADC(V_STAR_ANALOG_PIN) < GetADC(A_SCALED_ANALOG_PIN));
      break;
    case 5:
      // phase A and C should be energized, so look for phase B to cross V_star
      comparatorResult = (GetADC(V_STAR_ANALOG_PIN) > GetADC(B_SCALED_ANALOG_PIN));
      break;
    case 6:
      // phase A and B should be energized, so look for phase C to cross V_star
      comparatorResult = (GetADC(V_STAR_ANALOG_PIN) < GetADC(C_SCALED_ANALOG_PIN));
      break;
  }
  
  return comparatorResult;
  
  /*
  // put this potential zero-crossing through a filter to reject any false positives
  // add the sample bit to the contents at the previous index to determine the next table index
  static unsigned char bemfFilter = 0;
  //if (C2OUT) bemf_filter |= 0b1;              // copy the C2 output to the bemf_filter index LSB
  if (comparatorResult) bemfFilter |= 0b1;
  bemfFilter = kBemfFilterTable[bemfFilter];  // perform the filter table lookup
  
  return (bemfFilter & 0b1);                   // check for an ODD result?
  */
}

/*
Function: IsMotorStalled()
Description: Detects whether the motor is stalled by checking if we
  sense high-current AND we have no-zero crossings for some time
*/
static unsigned char IsMotorStalled(unsigned int commutationDuration) {
  return ( (MAX_COMMUTATION_DURATION < commutationDuration) );
           // || (MAX_NORMAL_CURRENT < GetADC(CURRENT_ANALOG_PIN) );
}


/*
Description: Performs an cycle to ensure the motor is moving so that we can 
  sense the back EMF.
  Locks the motor into a known position and fires off a commutation sequence 
  controlled by the Timer/counter1 overflow interrupt.
*/
static void PrimeMotor(void) {
  /*
  unsigned char i;
  unsigned char dummyPhase = 1;
  for (i = 0; i < NUM_PHASES; i++) {
    Commutate(&dummyPhase, 0);
    Delay(10);
  }
  */
  
  SET_PWM_COMPARE_VALUE(STARTUP_PWM_COMPARE_VALUE);

 
  // initialize to a known starting point
  nextCommutationStep = 0;
  UpdateControlPins(nextCommutationStep);
  DelayForStartup(STARTUP_LOCK_DELAY);
  nextCommutationStep++;
  
  unsigned char i;
  for (i = 0; i < NUM_STARTUP_COMMUTATIONS; i++) {
    UpdateControlPins(nextCommutationStep);
    DelayForStartup(kStartupDelays[i]);

    //ADMUX = ADMUXTable[nextCommutationStep];

    // use LSB of nextCommutationStep to determine zero crossing polarity
    zcPolarity = nextCommutationStep & 0b1;  // TODO: try to simplify this out

    // advance the commutation step
    if (NUM_PHASES <= ++nextCommutationStep) nextCommutationStep = 0;
  }
}


#define OPS_PER_DOT1MS            32// operations per 0.1milliseconds, for current 
                                    // oscillator choice and 
                                    // configuration-bit settings
static void DelayForStartup(unsigned int dot1Milliseconds) {
  unsigned int i,j;
	for (i = 0; i < OPS_PER_DOT1MS; i++) {
    for (j = 0; j < milliseconds; j++);
  }
}

/*---------------------------End of File-------------------------------------*/
