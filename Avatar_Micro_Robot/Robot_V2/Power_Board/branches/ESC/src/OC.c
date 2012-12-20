/*==============================================================================
File: OC.c
==============================================================================*/
//#define TEST_OC
//---------------------------Dependencies---------------------------------------
#include "./OC.h"
#include "./core/StandardHeader.h"
#include "./core/PPS.h"

//---------------------------Macros---------------------------------------------
// NB: the hall effect sensor inputs go to both input capture pins as well as digital inputs
// input-capture inputs
#define A_HI_RPN            11
#define A_LO_RPN            24
#define B_HI_RPN            23
#define B_LO_RPN            22
#define C_HI_RPN            25
#define C_LO_RPN            20

//#define TEST_HI_RPN         21
//#define TEST_LO_RPN         19

//---------------------------Helper Function Prototypes-------------------------
static void InitOutputCompareTimebase(void);
static void InitOC1(void);
static void InitOC2(void);
static void InitOC3(void);
static void InitOC4(void);
static void InitOC5(void);
static void InitOC6(void);

static void DisableOCModules(void);
static void inline TurnOnA_HI(void);
static void inline TurnOnA_LO(void);
static void inline TurnOnB_HI(void);
static void inline TurnOnB_LO(void);
static void inline TurnOnC_HI(void);
static void inline TurnOnC_LO(void);

//---------------------------Module Variables-----------------------------------

//---------------------------Test Harness---------------------------------------
#ifdef TEST_OC
#include "./core/ConfigurationBits.h"
int main(void) {
  OC_Init();
  
  while (1) {
    static uint8_t next_position = 0;
    Energize(next_position);
    if (5 < ++next_position) next_position = 0;
    Delay(1000);
  }
  
  return 0;
}
#endif

//---------------------------Interrupt Service Routines (ISRs)------------------

//---------------------------Public Function Definitions------------------------
void Energize(const uint8_t hall_state) {
  DisableOCModules();
  
  switch (hall_state) {
    case 1: TurnOnB_HI(); TurnOnC_LO(); break;
    case 2: TurnOnC_HI(); TurnOnA_LO(); break;
    case 3: TurnOnB_HI(); TurnOnA_LO(); break;
    case 4: TurnOnA_HI(); TurnOnB_LO(); break;
    case 5: TurnOnA_HI(); TurnOnC_LO(); break;
    case 6: TurnOnC_HI(); TurnOnB_LO();  break;
    default: break; // NB: 0 and 7 are invalid hall states
  }
}  

void OC_Init(void) {
  InitOutputCompareTimebase();
  InitOC1(); InitOC2(); InitOC3();
  InitOC4(); InitOC5(); InitOC6();
}
  
//---------------------------Helper Function Definitions------------------------
static void InitOutputCompareTimebase(void) {
  //T4CONbits.TCKPS = 0b00;         // configure prescaler to divide-by-1
  //PR4 = 795;
  PR4 = 10000;
  T4CONbits.TON = 1;              // turn on the timer
}

static void inline TurnOnA_LO(void) {
  _LATD1 = 1;
}

static void inline TurnOnB_LO(void) {
  _LATD3 = 1;
}

static void inline TurnOnC_LO(void) {
  _LATD5 = 1;
}

static void inline TurnOnA_HI(void) {
  PPS_MapPeripheral(A_LO_RPN, OUTPUT, FN_OC1);
  PPS_MapPeripheral(A_HI_RPN, OUTPUT, FN_OC2);
}

static void inline TurnOnB_HI(void) {
  PPS_MapPeripheral(B_LO_RPN, OUTPUT, FN_OC3);
  PPS_MapPeripheral(B_HI_RPN, OUTPUT, FN_OC4);
}

static void inline TurnOnC_HI(void) {
  PPS_MapPeripheral(C_LO_RPN, OUTPUT, FN_OC5);
  PPS_MapPeripheral(C_HI_RPN, OUTPUT, FN_OC6);
}

static void InitOC1(void) {
  OC1CON1bits.OCTSEL = 0b010;     // use Timer4 as the timebase
  OC1CON1bits.OCM = 0b101;        // Double Compare Continuous Pulse mode (see p.5 of PIC24F Output Compare manual
  OC1CON2bits.SYNCSEL = 0b01110;  // trigger source is Timer4
  OC1CON2bits.OCINV = 0;
  //OC1R = PR4;                     // toggle when Timer4 ticks up to Timer4 maximum
  OC1R = 2000;               // toggle when Timer4 ticks up to ~1/8 of Timer4 maximum
  OC1RS = PR4;
}

static void InitOC2(void) {
  OC2CON1bits.OCTSEL = 0b010;
  OC2CON1bits.OCM = 0b101;        
  OC2CON2bits.SYNCSEL = 0b01110;
  OC2CON2bits.OCINV = 1;          // invert the output
  //OC2R = PR4;
  OC2R = 2000;
  OC2RS = PR4;
}

static void InitOC3(void) {
  OC3CON1bits.OCTSEL = 0b010;
  OC3CON1bits.OCM = 0b101;        
  OC3CON2bits.SYNCSEL = 0b01110;
  OC3CON2bits.OCINV = 0;
  //OC3R = PR4;
  OC3R = 2000;
  OC3RS = PR4;
}

static void InitOC4(void) {
  OC4CON1bits.OCTSEL = 0b010;
  OC4CON1bits.OCM = 0b101;        
  OC4CON2bits.SYNCSEL = 0b01110;
  OC4CON2bits.OCINV = 1;          // invert the output
  //OC4R = PR4;
  OC4R = 2000;
  OC4RS = PR4;
}

static void InitOC5(void) {
  OC5CON1bits.OCTSEL = 0b010;
  OC5CON1bits.OCM = 0b101;        
  OC5CON2bits.SYNCSEL = 0b01110;
  OC5CON2bits.OCINV = 0;
  //OC5R = PR4;
  OC5R = 2000;
  OC5RS = PR4;
}

static void InitOC6(void) {
  OC6CON1bits.OCTSEL = 0b010;
  OC6CON1bits.OCM = 0b101;        
  OC6CON2bits.SYNCSEL = 0b01110;
  OC6CON2bits.OCINV = 1;          // invert the output
  //OC6R = PR4;
  OC6R = 2000;
  OC6RS = PR4;
}

// Disables all output compare modules, configures them as digital outputs
//   and initializes them to logical low.
static void DisableOCModules(void) {
  LATD = 0x0000;
  PPS_MapPeripheral(A_HI_RPN, OUTPUT, FN_NULL);
  PPS_MapPeripheral(A_LO_RPN, OUTPUT, FN_NULL);
  PPS_MapPeripheral(B_HI_RPN, OUTPUT, FN_NULL);
  PPS_MapPeripheral(B_LO_RPN, OUTPUT, FN_NULL);
  PPS_MapPeripheral(C_HI_RPN, OUTPUT, FN_NULL);
  PPS_MapPeripheral(C_LO_RPN, OUTPUT, FN_NULL);
  TRISD = 0x0000; Nop(); Nop();
}
