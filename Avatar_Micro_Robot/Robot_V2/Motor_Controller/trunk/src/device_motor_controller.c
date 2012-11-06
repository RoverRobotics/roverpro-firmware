
#include "stdhdr.h"
#include "device_motor_controller.h"
#include "testing.h"

#define AHI_1_EN(a) (_TRISD0 = !a)
#define ALO_1_EN(a) (_TRISD1 = !a)
#define BHI_1_EN(a) (_TRISD2 = !a)
#define BLO_1_EN(a) (_TRISD3 = !a)
#define CHI_1_EN(a) (_TRISD4 = !a)
#define CLO_1_EN(a) (_TRISD5 = !a)

#define AHI_1_ON(a) (_LATD0 = a)
#define ALO_1_ON(a) (_LATD1 = a)
#define BHI_1_ON(a) (_LATD2 = a)
#define BLO_1_ON(a) (_LATD3 = a)
#define CHI_1_ON(a) (_LATD4 = a)
#define CLO_1_ON(a) (_LATD5 = a)

#define HA_1()      (_RE5)
#define HB_1()      (_RE6)
#define HC_1()      (_RE7)

//RP23
#define BHI_1_OR _RP23R
#define CHI_1_OR _RP25R


static void init_pwm(void);
static void set_pwm_duty(unsigned int duty_cycle);
static void handle_commutation(void);

static void Timer2_ISR(void);
static unsigned int fifty_us_counter = 0;
static unsigned int millisecond_counter = 0;

void Device_Motor_Controller_Process_IO(void)
{

  unsigned int i;
/*
  //for some reason, low side must go high to charge back up
  //high side gate driver
  BLO_1_ON(0);
  //AHI_1_ON(1);
  set_pwm_duty(50);
  block_ms(2000);
  //AHI_1_ON(0);
//ALO_1_ON(1);
  set_pwm_duty(0);
  BLO_1_ON(1);
  block_ms(2000);
//ALO_1_ON(0);
  //BLO_1_ON(0);
*/
handle_commutation();



}

void Device_Motor_Controller_Init(void)
{

  AHI_1_ON(0);
  ALO_1_ON(0);
  BHI_1_ON(0);
  BLO_1_ON(0);
  CHI_1_ON(0);
  CLO_1_ON(0);

  AHI_1_EN(1);
  ALO_1_EN(1);
  BHI_1_EN(1);
  BLO_1_EN(1);
  CHI_1_EN(1);
  CLO_1_EN(1);

  //set_pwm_duty(0);


  T2InterruptUserFunction = Timer2_ISR;
  //T2 frequency is Fosc/2 = 16 MHz
  //Max timer interrupt at 2^16/16MHz = 4.096ms
  //Let's set PR2 to 800, to get interrupts every 50us:
  PR2 = 80;
  _T2IE = 1;
  T2CONbits.TON = 1;

  block_ms(5000);

  //init_pwm();


}

static void init_pwm(void)
{

	//Choose 400Hz (2.5ms period)
	//2.5ms = [PR2 + 1]*62.5ns*1
  //CHI_1_OR = 18; //OC1
  BHI_1_OR = 18;
	PR2 = 40001;	
	OC1RS = 4000;


	OC1CON2bits.SYNCSEL = 0x1f;	
	//use timer 2

	OC1CON1bits.OCTSEL2 = 0;

	//edge-aligned pwm mode
	OC1CON1bits.OCM = 6;

	//turn on timer 2
	T2CONbits.TON = 1;


}

static void set_pwm_duty(unsigned int duty_cycle)
{
	if(duty_cycle > 100)
		duty_cycle = 100;
  OC1R = duty_cycle*40;

}

static void handle_commutation(void)
{
  static unsigned char MOSFET_states_F[7];
  static unsigned char MOSFET_states_R[7];
  static char next_commutation_state[14];
  static unsigned char sensor_data = 0x00;
  static unsigned int last_millisecond_counter = 0;

  static unsigned char MOSFET_state = 0x00;
  static unsigned char dummy1, dummy2;

  dummy1 = (MOSFET_state & 0x010000);
  dummy2 = (MOSFET_state & 0x001000);

  //HA LA HB LB HC LC
  MOSFET_states_F[0b101] = 0b100001;
  MOSFET_states_F[0b100] = 0b001001;
  MOSFET_states_F[0b110] = 0b011000;
  MOSFET_states_F[0b010] = 0b010010;
  MOSFET_states_F[0b011] = 0b000110;
  MOSFET_states_F[0b001] = 0b100100;
  MOSFET_states_F[0b000] = 0b000000;
  MOSFET_states_F[0b111] = 0b000000;

  MOSFET_states_R[0b101] = 0b010010;
  MOSFET_states_R[0b100] = 0b000110;
  MOSFET_states_R[0b110] = 0b100100;
  MOSFET_states_R[0b010] = 0b100001;
  MOSFET_states_R[0b011] = 0b001001;
  MOSFET_states_R[0b001] = 0b011000;
  MOSFET_states_R[0b000] = 0b000000;
  MOSFET_states_R[0b111] = 0b000000;



  if(fifty_us_counter >= 20)
  {

    fifty_us_counter = 0;
    last_millisecond_counter = millisecond_counter;

    sensor_data = ((HA_1()<<2)+(HB_1()<<1)+HC_1()) & 0b111;
    
    MOSFET_state = MOSFET_states_F[sensor_data];
  
    if(MOSFET_state & 0b010000)
    {
      AHI_1_ON(0);
      ALO_1_ON(1);
    }
    else if(MOSFET_state & 0b100000)
    {
      ALO_1_ON(0);
      AHI_1_ON(1);
    }
    else
    {
      AHI_1_ON(0);
      ALO_1_ON(0);
    }
  
    if(MOSFET_state & 0b000100)
    {
      BHI_1_ON(0);
      BLO_1_ON(1);
    }
    else if(MOSFET_state & 0b001000)
    {
      BLO_1_ON(0);
      BHI_1_ON(1);
    }
    else
    {
      BLO_1_ON(0);
      BHI_1_ON(0);
    }
    
    if(MOSFET_state & 0b000001)
    {
      CHI_1_ON(0);
      CLO_1_ON(1);
    }
    else if(MOSFET_state & 0b000010)
    {
      CLO_1_ON(0);
      CHI_1_ON(1);
    }
    else
    {
      CHI_1_ON(0);
      CLO_1_ON(0);
    }
  }
  //turn on low side if high side is on
  else if(fifty_us_counter >=19)
  {
    if(MOSFET_state & 0b10000)
    { 
      AHI_1_ON(0);
      ALO_1_ON(1);
    }
    if(MOSFET_state & 0b001000)
    {
      BHI_1_ON(0);
      BLO_1_ON(1);
    }
    if(MOSFET_state & 0b000010)
    {
      CHI_1_ON(0);
      CLO_1_ON(1);
    }

  }

}

static void Timer2_ISR(void)
{
  _T2IF = 0;
  //millisecond_counter++;
  fifty_us_counter++;

  if(fifty_us_counter >= 20)
  {
    millisecond_counter++;
  }
  if(fifty_us_counter > 20000)
    fifty_us_counter = 0;

  /*
  
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
  }*/
}
