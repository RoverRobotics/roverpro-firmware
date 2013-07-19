/*---------------------------Dependencies-------------------------------------*/
#include "device_repeater.h"
#include "stdhdr.h"
#include "debug_uart.h"
#include "timer.h"
#include "usb_communication.h"

#define USB_TIMEOUT_ENABLED

/*---------------------------Macros-------------------------------------------*/
#define POWER_BUS_EN(a)     _TRISB2=(!a)
#define POWER_BUS_ON(a)     _LATB2=a
#define HALL_1              _RD3
#define HALL_2              _RD4
#define HALL_3              _RD5
#define HALL_4              _RD11
#define SERVO_L_OR          _RP24R
#define SERVO_R_OR          _RP23R

#define LEFT_SERVO          1
#define RIGHT_SERVO         2

/*---------------------------Helper Function Prototypes-----------------------*/
static void InitPins(void);
static void InitPWM(void);
static void set_servo_position(unsigned char servo_number, int servo_position);
static void update_repeater_positions(void);
static void read_repeater_positions(void);
static void dispenser_open(void);
static void dispenser_close(void);

/*---------------------------Module Variables---------------------------------*/


static unsigned char repeater_positions[4] = {0,0,0,0};

/*---------------------------Public Function Definitions----------------------*/

void Repeater_Init()
{

  InitPins();
  InitTimer();

  dispenser_close();
  InitPWM();



}

void Repeater_Process_IO()
{

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
    if(REG_REPEATER_RELEASE)
    {
      dispenser_open();

    }
    else
    {
      dispenser_close();
    }
    read_repeater_positions();

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
	OC1CON1bits.OCTSEL2 = 1;
	OC2CON1bits.OCTSEL2 = 1;
	
	//edge-aligned pwm mode
	OC1CON1bits.OCM = 6;
	OC2CON1bits.OCM = 6;
	
	//turn on timer 2
	T3CONbits.TON = 1;
  

}

static void set_servo_position(unsigned char servo_number, int servo_position)
{

  unsigned int OCxR = 0;

  if(servo_position > 100) servo_position = 100;
  if(servo_position < -100) servo_position = -100;

  //each ms is 2,000 ticks
  OCxR = 38000+servo_position*20;

  if(servo_number == 1) OC1R = OCxR;
  else OC2R = OCxR;

}

static void read_repeater_positions(void)
{
  if(HALL_1)
  {
    repeater_positions[0]++;
  }

  if(HALL_2)
  {
    repeater_positions[1]++;
  }

  if(HALL_3)
  {
    repeater_positions[2]++;
  }

  if(HALL_4)
  {
    repeater_positions[3]++;
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
    repeater_positions[0] = 0;
  }

}

static void dispenser_open(void)
{
  set_servo_position(LEFT_SERVO,100);
  set_servo_position(RIGHT_SERVO,-100);
}

static void dispenser_close(void)
{
  set_servo_position(LEFT_SERVO,-100);
  set_servo_position(RIGHT_SERVO,100);
}
