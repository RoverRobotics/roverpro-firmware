#include "home_office.h"
#include "charging.h"
#include "./core/ConfigurationBits.h"
#include "motor_control.h"
#include "uart.h"


static void InitPins(void);
static void start_up(void);
static void ADC_Init(void);


static void InitTimer(void);
static unsigned int millisecond_counter = 0;
static unsigned int ten_millisecond_counter = 0;
static unsigned int hundred_millisecond_counter = 0;
static unsigned int second_counter = 0;



void main(void)
{
  InitPins();
  start_up();
  InitTimer();
  ESC_Init();
  ADC_Init();
  init_uart();
  i2c3_init();

  //motor_control_test_function();

  while(1)
  {
  if(ten_millisecond_counter)
  {  
    motor_control_FSM();
    //parse_UART_message();
    messaging_FSM();
    battery_FSM();
    ten_millisecond_counter = 0;
  }
  if(hundred_millisecond_counter >= 10)
  {
    hundred_millisecond_counter = 0;
    //send_battery_message();

  }
    ClrWdt();

  }

}


static void InitPins(void)
{
  TRISB = 0xffff;
  TRISC = 0xffff;
  TRISD = 0xffff;
  TRISE = 0xffff;
  TRISF = 0xffff;

 


  V5_ON(0);
  SYS_BUS_ON(0);
  BQ24745_ON(0);

  V5_EN(1);
  SYS_BUS_EN(1);
  BQ24745_EN(1); 


}

static void start_up(void)
{

  unsigned int i = 0;

  //if battery is too low and not on the dock, this will shut down the robot
  for(i=0;i<20;i++)
  {
    //battery_FSM();
  }
  SYS_BUS_ON(1);
  V5_ON(1);

}

static void ADC_Init(void)
{

  AD1PCFGL = 0xffff;
  AD1PCFGH = 0xffff;

  //auto-convert
  AD1CON1bits.SSRC = 0b111;

  AD1CHS0bits.CH0SA = 2;
  AD1CHS0bits.CH0SB = 2;


  AD1CON2bits.SMPI = 0x03;  //interrupts every 4th conversion
  AD1CON3bits.SAMC = 0x1f;    //31Tad
  AD1CON3bits.ADCS = 0x1f;    //31*Tcy

  AD1CHS0bits.CH0NA = 0;      //ADC negative input is GND
  //AD1CHS0bits.CH0SA = 


  AD1PCFGLbits.PCFG0 = 0;
  AD1PCFGLbits.PCFG1 = 0;
  AD1PCFGLbits.PCFG2 = 0;
  AD1PCFGLbits.PCFG3 = 0;

  AD1CSSLbits.CSSL0 = 1;
  AD1CSSLbits.CSSL1 = 1;
  AD1CSSLbits.CSSL2 = 1;
  AD1CSSLbits.CSSL3 = 1;

  AD1CON2bits.CSCNA = 1;      //scan inputs

  
  AD1CON1bits.ADON = 1;

  //_AD1IF = 0;
  //_AD1IE = 1;

  AD1CON1bits.ASAM = 1;       //begin auto sampling

  
}


void __attribute__((__interrupt__, auto_psv)) _ADC1Interrupt(void) {
  unsigned int test;
  _AD1IF = 0;
	Nop();
  test=ADC1BUF0;
  test=ADC1BUF1;
  test=ADC1BUF2;
  test=ADC1BUF3;
}


void __attribute__((__interrupt__, auto_psv)) _T2Interrupt(void)
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


static void InitTimer(void)
{

  //T2 frequency is Fosc/2 = 16 MHz
  //Max timer interrupt at 2^16/16MHz = 4.096ms
  //Let's set PR2 to 16000, to get interrupts every 1ms:
  PR2 = 16000;
  _T2IE = 1;
  T2CONbits.TON = 1;

}


