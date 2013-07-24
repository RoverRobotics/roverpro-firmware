/*---------------------------Dependencies-------------------------------------*/
#include "device_repeater.h"
#include "stdhdr.h"
#include "debug_uart.h"
#include "timer.h"
#include "usb_communication.h"

//#define USB_TIMEOUT_ENABLED

/*---------------------------Hardware Configuration---------------------------*/
#define POWER_BUS_EN(a)     _TRISB2=(!a)
#define POWER_BUS_ON(a)     _LATB2=a
#define HALL_1              _RD3
#define HALL_2              _RD4
#define HALL_3              _RD5
#define HALL_4              _RD11
#define SERVO_L_OR          _RP24R
#define SERVO_R_OR          _RP23R
#define CURRENT_ADC_PIN     0
#define CURRENT_PIN_EN(a)   _PCFG0=(!a)

/*---------------------------Software Constants-------------------------------*/
#define LEFT_SERVO          1
#define RIGHT_SERVO         2

//1A
#define STALL_CURRENT       155  //0.01S*(1A*0.05Ohm)*1k * (1023/3.3) = 155

/*---------------------------Helper Function Prototypes-----------------------*/
static void InitPins(void);
static void InitPWM(void);
static void set_servo_position(unsigned char servo_number, int servo_position);
static void update_repeater_positions(void);
static void read_repeater_positions(void);
static void dispenser_open(void);
static void dispenser_close(void);
static void handle_overcurrent(void);
static void ADC_ISR(void);
static void init_ADC(void);
static void servos_off(void);
static void servos_on(void);
static void test_servo_positions(void);

/*---------------------------Module Variables---------------------------------*/


static unsigned char repeater_positions[4] = {0,0,0,0};
static unsigned int servo_current = 0;

/*---------------------------Public Function Definitions----------------------*/

void Repeater_Init()
{

  InitPins();
  init_ADC();
  InitTimer();

  POWER_BUS_EN(1);
  POWER_BUS_ON(1);

  dispenser_close();
  InitPWM();



}

void Repeater_Process_IO()
{

  static unsigned int close_counter = 0;

  #ifdef USB_TIMEOUT_ENABLED
    if(USB_timeout_counter > 100)
    {
      dispenser_close();
      return;
    }
  #endif

  if(ten_millisecond_counter)
  {  
    ten_millisecond_counter = 0;
    USB_timeout_counter++;
    test_servo_positions();
    if(REG_REPEATER_RELEASE)
    {
      //servos_on();
      //POWER_BUS_ON(1);
      dispenser_open();
      close_counter = 0;

    }
    else
    {
      close_counter++;
      dispenser_close();

      if(close_counter > 200)
      {
        close_counter = 300;
        //POWER_BUS_ON(0);
      }
    }
    read_repeater_positions();
    handle_overcurrent();

  }
  if(hundred_millisecond_counter >= 1)
  {
    update_repeater_positions();
    hundred_millisecond_counter = 0;

  }
  

}

static void InitPins(void)
{
  TRISB = 0xffff;
  TRISC = 0xffff;
  TRISD = 0xffff;
  TRISE = 0xffff;
  TRISF = 0xffff;
  AD1PCFGL = 0xffff;
  AD1PCFGH = 0xffff;

}

static void ADC_ISR(void)
{
  unsigned int test1, test2, test3, test4;
  _AD1IF = 0;
  test1 = ADC1BUF0;
  test2 = ADC1BUF1;
  test3 = ADC1BUF2;
  test4 = ADC1BUF3;
  Nop();
  Nop();
  


}

static void init_ADC(void)
{

  CURRENT_PIN_EN(1);

  ADC1InterruptUserFunction=ADC_ISR;


  //auto convert
  AD1CON1bits.SSRC = 0b111;

  //auto sample
  AD1CON1bits.ASAM = 1;

  //interrupt every 4 conversions
  AD1CON2bits.SMPI = 3;

  AD1CON3bits.SAMC = 0x1f;  //31*Tad
  AD1CON3bits.ADCS = 0x1f;  //31*Tcy

  AD1CHSbits.CH0SA = CURRENT_ADC_PIN;

  AD1CON1bits.SAMP = 1;


  //_AD1IE = 1;
  

  AD1CON1bits.ADON = 1;
}

static void InitPWM(void)
{
	SERVO_L_OR = 18;	//OC1
	SERVO_R_OR	= 19;	//OC2

  //PR2 is 16000 for timing, but we want a period of at least 20ms
  //20ms = 50 Hz, 32e6/2/2^16/50 = 4.88, so we'll need a prescaler of
  //at least 4.88.  
  
  //set 8:1 prescale
  T3CONbits.TCKPS = 0b01;

  //at 8:1, 20ms will be 32e6/2/8/50=40,000 ticks
  OC1RS = 40000;
  OC2RS = 40000;
  
	OC1CON2bits.SYNCSEL = 0x1f;	
	OC2CON2bits.SYNCSEL = 0x1f;	
	
	//use timer 3
	OC1CON1bits.OCTSEL = 1;
	OC2CON1bits.OCTSEL = 1;
	
	//edge-aligned pwm mode
	OC1CON1bits.OCM = 6;
	OC2CON1bits.OCM = 6;
	
	//turn on timer 2
	T3CONbits.TON = 1;

  _ODD1 = 1;
  _ODD2 = 1;


  

}

static void set_servo_position(unsigned char servo_number, int servo_position)
{

  unsigned int OCxR = 0;

  /*if(servo_position > 100) servo_position = 100;
  if(servo_position < -100) servo_position = -100;*/

  //each ms is 2,000 ticks
  //OCxR = 38000+servo_position*20;
  OCxR = 3000+servo_position*10;

  if(servo_number == 1) OC1R = OCxR;
  else OC2R = OCxR;

}

static void read_repeater_positions(void)
{
  if(HALL_1==0)
  {
    repeater_positions[3]++;
  }

  if(HALL_2==0)
  {
    repeater_positions[2]++;
  }

  if(HALL_3==0)
  {
    repeater_positions[1]++;
  }

  if(HALL_4==0)
  {
    repeater_positions[0]++;
  }

}

static void update_repeater_positions(void)
{
  unsigned int i;

  REG_REPEATER_POSITIONS = 0;

  for(i=0;i<4;i++)
  {
    if(repeater_positions[i] > 5)
      REG_REPEATER_POSITIONS |= (1<<i);
    repeater_positions[i] = 0;
  }

}

static void dispenser_close(void)
{
  set_servo_position(LEFT_SERVO,-100);
  set_servo_position(RIGHT_SERVO,100);
}

static void dispenser_open(void)
{
  set_servo_position(LEFT_SERVO,100);
  set_servo_position(RIGHT_SERVO,-100);
}

static void handle_overcurrent(void)
{

  typedef enum {
    NORMAL = 0,
    WAITING_AFTER_OVERCURRENT
  } repeater_current_state;

  static repeater_current_state state = NORMAL;
  static int overcurrent_counter = 0;
  static int wait_counter = 0;
  servo_current = ADC1BUF0;

  switch(state)
  {
    case NORMAL:

      if(servo_current >= STALL_CURRENT)
        overcurrent_counter++;
      else
        overcurrent_counter = 0;
    
      if(overcurrent_counter > 100)
      {
        POWER_BUS_ON(0);
        overcurrent_counter = 200;
        state = WAITING_AFTER_OVERCURRENT;
      }

    break;
    case WAITING_AFTER_OVERCURRENT:
      wait_counter++;
      if(wait_counter > 500)
      {
        overcurrent_counter = 0;
        wait_counter = 0;
        state = NORMAL;
        POWER_BUS_ON(1);
        
      }
    break;
  }


}

static void servos_off(void)
{
	SERVO_L_OR = 0;	//OC1
	SERVO_R_OR	= 0;	//OC2

  _TRISD1 = 0;
  _TRISD2 = 0;
  _RD1 = 0;
  _RD2 = 0;

  _ODD1 = 0;
  _ODD2 = 0;


}

static void servos_on(void)
{
	SERVO_L_OR = 18;	//OC1
	SERVO_R_OR	= 19;	//OC2

  _ODD1 = 1;
  _ODD2 = 1;

}

static void test_servo_positions(void)
{
  static unsigned int state_counter = 0;

  state_counter++;


  if (state_counter > 1000)
  {
   REG_REPEATER_RELEASE = 0;
    state_counter = 0;
  }
  else if(state_counter > 500)
   REG_REPEATER_RELEASE = 1;

}
