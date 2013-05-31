/*==============================================================================
File: device_arm_link2.c
==============================================================================*/
/*---------------------------Dependencies-------------------------------------*/
#include "device_initiator.h"
#include "stdhdr.h"
#include "debug_uart.h"


//#define USB_TIMEOUT_ENABLED
/*---------------------------Macros-------------------------------------------*/

#define DISCHARGE_EN(a)     _TRISF3 = (!a)
#define DISCHARGE(a)        _LATF3 = (!a)
#define UNDERVOLTAGE        _RD5
#define GATE_XFRMR_EN(a)    _TRISD0 = (!a)
#define GATE_XFRMR_ON(a)    _LATD0 = a
#define MAIN_XFRMR_OR       _RP9R

#define INITIATOR_CHARGE_TIMEOUT 0x8000


/*---------------------------Macros-------------------------------------------*/
//static unsigned int REG_INITIATOR_FIRE = 0;
//static unsigned int REG_INITIATOR_CHARGE = 0;
//static unsigned int REG_INITIATOR_STATE = 0;
//static unsigned int REG_INITIATOR_FLAGS = 0;

/*---------------------------Type Definitions---------------------------------*/


/*---------------------------Helper Function Prototypes-----------------------*/
static void InitTimer(void);
static void charge_capacitor(unsigned int enabled);
static void Initiator_FSM(void);
void Timer2_ISR(void);
static void init_PWM(void);
static unsigned int check_USB_timeout(void);
                                       
/*---------------------------Module Variables---------------------------------*/

unsigned char USB_timeout_counter = 0;


static unsigned int millisecond_counter = 0;
static unsigned int ten_millisecond_counter = 0;
static unsigned int hundred_millisecond_counter = 0;
static unsigned int second_counter = 0;



/*---------------------------Public Function Definitions----------------------*/
void Initiator_Init(void) {

  AD1PCFGH = 0xffff;
  TRISB = 0xffff;
  TRISC = 0xffff;
  TRISD = 0xffff;
  TRISE = 0xffff;
  TRISF = 0xffff;

  DISCHARGE(1);
  GATE_XFRMR_ON(0);

  DISCHARGE_EN(1);
  GATE_XFRMR_EN(1);

  T2InterruptUserFunction = Timer2_ISR;

  InitTimer();
  init_PWM();


  

}


void Initiator_Process_IO(void) {

  if(ten_millisecond_counter)
  {  
    Initiator_FSM();
    ten_millisecond_counter = 0;
  }

 }


/*---------------------------Helper Function Definitions----------------------*/

static void InitTimer(void)
{

  //T2 frequency is Fosc/2 = 16 MHz
  //Max timer interrupt at 2^16/16MHz = 4.096ms
  //Let's set PR2 to 16000, to get interrupts every 1ms:
  PR2 = 16000;
  _T2IE = 1;
  T2CONbits.TON = 1;

}

void Timer2_ISR(void)
{
  _T2IF = 0;
  millisecond_counter++;
  
  //Make sure rollover is at a multiple of 10
  if(millisecond_counter == 10000)
    millisecond_counter = 0;


  if(millisecond_counter%10 == 0)
  {
    ten_millisecond_counter++;
    if(millisecond_counter%100 == 0)
    {
      hundred_millisecond_counter++;
      if(millisecond_counter%1000 == 0)
      {
        second_counter++;
      }
     
    }
  }
}

static void Initiator_FSM(void)
{
  static unsigned int thermal_recovery_counter = 0;
  static unsigned int charge_timeout_counter = 0;
  static unsigned int firing_pulse_counter = 0;
  static unsigned int charged_counter = 0;

  typedef enum {
    sWaiting = 0,
    sCharging,
    sCharged,
    sFiring,
    sDischarging,
    sThermalRecovery,
  } sInitiatorState;

  static sInitiatorState state = sWaiting;



  REG_INITIATOR_STATE = ( (REG_INITIATOR_STATE & 0xff00) | state);

  #ifdef USB_TIMEOUT_ENABLED
  if(check_USB_timeout())
  { 
    if( (state != sWaiting) && (state != sDischarging) && (state != sThermalRecovery) )
    {
      state = sDischarging;
    }
  }
  #endif

  if(REG_INITIATOR_CHARGE == 0)
  {
    if( (state != sWaiting) && (state != sDischarging) && (state != sThermalRecovery) )
    {  
      state = sDischarging;
    }
  }

  switch(state)
  {
    case sWaiting:
      thermal_recovery_counter = 0;
      charge_timeout_counter = 0;
      firing_pulse_counter = 0;
      charged_counter = 0;
      DISCHARGE(1);
      if((REG_INITIATOR_CHARGE==0x01)&&(REG_INITIATOR_FIRE==0))
        state = sCharging;
    break;

    case sCharging:
      DISCHARGE(0);
      REG_INITIATOR_STATE &= ~(INITIATOR_CHARGE_TIMEOUT);
      charge_capacitor(1);
      charge_timeout_counter++;
      if(UNDERVOLTAGE==0)
      {
        charged_counter++;
      }
      if(charged_counter > 3)
      {
        charged_counter = 0;
        state = sCharged;
      }
      if(charge_timeout_counter > 150)
      {
        REG_INITIATOR_STATE |= INITIATOR_CHARGE_TIMEOUT;
        state = sDischarging;
      }
      if(REG_INITIATOR_CHARGE == 0)
      {
        state = sDischarging;
      }    
    break;

    case sCharged:
      charge_capacitor(1);
      if(REG_INITIATOR_FIRE==0x01)
        state = sFiring;
      /*if(OC1R == 0)
      {
        state = sFiring;
        Nop();
        Nop();
        Nop();
        Nop();
      }*/
    break;

    case sFiring:
      GATE_XFRMR_ON(1);
      firing_pulse_counter++;
      if(firing_pulse_counter > 50)
      {
        firing_pulse_counter = 0;
        GATE_XFRMR_ON(0);
        state = sDischarging;
      }
    break;

    case sDischarging:
      charge_capacitor(0);
      DISCHARGE(1);
      state = sThermalRecovery;
    break;

    case sThermalRecovery:
      charge_capacitor(0);
      DISCHARGE(1);
      thermal_recovery_counter++;
      if(thermal_recovery_counter > 500)
      {
        thermal_recovery_counter = 0;
        state = sWaiting;
      }
    break;

    default:
      state = sDischarging;
    break;
  }

  

}

static void charge_capacitor(unsigned int enabled)
{

  if(enabled == 0)
  {
    OC1R = 0;
    return;
  }
  if(UNDERVOLTAGE)
  {
    OC1R = 80;
  }
  else
  {
    OC1R = 0;
  }

}

static void init_PWM(void)
{
  //set to OC1
  MAIN_XFRMR_OR = 18;

	OC1R=0;

  //50kHz: 32e6/2/50e3 = 320
	OC1RS=320;
	OC1CON2bits.SYNCSEL=0x1F;
	OC1CON2bits.OCTRIG=CLEAR;
	OC1CON1bits.OCTSEL=0b000;//Timer2
	OC1CON1bits.OCM=0b110;

}

static unsigned int check_USB_timeout(void)
{
  USB_timeout_counter++;
  if(USB_timeout_counter > 100)
  {
    USB_timeout_counter = 200;
    return 1;
  }

  return 0;
}
