
#include "stdhdr.h"
#include "device_motor_controller.h"
#include "testing.h"

#define BRUSHED

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

#define set_AHI_1_duty_cycle(a)  (OC1R = 2000-a*2)
#define set_BHI_1_duty_cycle(a)  (OC2R = 2000-a*2)
#define set_CHI_1_duty_cycle(a)  (OC3R = 2000-a*2)

//RP23
#define AHI_1_OR _RP11R
#define BHI_1_OR _RP23R
#define CHI_1_OR _RP25R


static void init_pwm(void);
static void set_pwm_duty(unsigned int duty_cycle);
static void handle_commutation(void);

static void Timer2_ISR(void);
static unsigned int fifty_us_counter = 0;
static unsigned int millisecond_counter = 0;

static void test_pwm(void);

static void brushed_control(int speed);
static void brushless_init(void);
static void brushed_init(void);
void init_pwm_brushed(void);

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
//test_pwm();
#ifdef BRUSHLESS
handle_commutation();
#endif

#ifdef BRUSHED
#endif



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


  #ifdef BRUSHLESS
    brushless_init();
  #endif

  #ifdef BRUSHED
    brushed_init();
  #endif




}

static void init_pwm(void)
{

	//Choose 400Hz (2.5ms period)
	//2.5ms = [PR2 + 1]*62.5ns*1
  //CHI_1_OR = 18; //OC1
  //Choose 8kHz (125us period)
  //125us = (PR2 + 1)*62.5ns
  //PR2 = 1999
  AHI_1_OR = 18;
  BHI_1_OR = 19;
  CHI_1_OR = 20;
	//PR2 = 40001;	

  //if both are 0, module doesn't stay low
  OC1R = 0;
	OC1RS = 2001;

  OC2R = 0;
  OC2RS = 2001;
  
  OC3R = 0;
  OC3RS = 2001;

  PR3 = 8000;


  //synchronize to Timer 3
	/*OC1CON2bits.SYNCSEL = 0b1101;	
	OC2CON2bits.SYNCSEL = 0b1101;	
	OC3CON2bits.SYNCSEL = 0b1101;	*/
OC1CON2bits.SYNCSEL = 0x1f;	
	OC2CON2bits.SYNCSEL = 0x1f;	
	OC3CON2bits.SYNCSEL = 0x1f;
	//use timer 3

  //Note:  We can't use PWM, as it is double-buffered
  //(to avoid shoot-through)
	//single-compare single-shot (starts high)
	OC1CON1bits.OCM = 5;
	OC2CON1bits.OCM = 5;
	OC3CON1bits.OCM = 5;

	OC1CON1bits.OCTSEL = 1;
	OC2CON1bits.OCTSEL = 1;
	OC3CON1bits.OCTSEL = 1;



	//turn on timer 3
	T3CONbits.TON = 1;


}

static void set_pwm_duty(unsigned int duty_cycle)
{
	if(duty_cycle > 100)
		duty_cycle = 100;
  OC1R = duty_cycle*40;

}

static void handle_commutation(void)
{
  static unsigned char MOSFET_states_F[8];
  static unsigned char MOSFET_states_F_ATMEL[8];
  static unsigned char MOSFET_states_R[8];
  static char next_commutation_state[14];
  static unsigned char sensor_data = 0x00;
  static unsigned int last_millisecond_counter = 0;

  static unsigned int speed = 0;

  static unsigned char MOSFET_state = 0x00;

  static unsigned char last_MOSFET_state = 0x00;
  static unsigned int locked_rotor_counter = 0;
  static unsigned int locked_rotor_recovery_counter = 0;
  static unsigned int locked_rotor_flag = 0;

  static unsigned int move_counter = 0;
  static unsigned int last_fifty_us_counter = 0;

  //HA LA HB LB HC LC
  MOSFET_states_F_ATMEL[0b101] = 0b100100;
  MOSFET_states_F_ATMEL[0b100] = 0b100001;
  MOSFET_states_F_ATMEL[0b110] = 0b001001;
  MOSFET_states_F_ATMEL[0b010] = 0b011000;
  MOSFET_states_F_ATMEL[0b011] = 0b010010;
  MOSFET_states_F_ATMEL[0b001] = 0b000110;
  MOSFET_states_F_ATMEL[0b000] = 0b000000;
  MOSFET_states_F_ATMEL[0b111] = 0b000000;
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

  speed = 100;

  //turn on low side if high side is on
  if(fifty_us_counter >=19)
  {
    if(MOSFET_state & 0b100000)
    { 
      //AHI_1_ON(0);
      set_AHI_1_duty_cycle(0);
      ALO_1_ON(1);
    }
    if(MOSFET_state & 0b001000)
    {
      //BHI_1_ON(0);
      set_BHI_1_duty_cycle(0);
      BLO_1_ON(1);
    }
    if(MOSFET_state & 0b000010)
    {
      //CHI_1_ON(0);
      set_CHI_1_duty_cycle(0);
      CLO_1_ON(1);
    }

  }
  //runs every 50us
  else if(fifty_us_counter != last_fifty_us_counter)
  {
    last_fifty_us_counter = fifty_us_counter;

    //fifty_us_counter = 0;

    if(locked_rotor_flag)
    {
      locked_rotor_recovery_counter++;
      //after 5s, try to restart
      if(locked_rotor_recovery_counter > 1000)
      {
        locked_rotor_flag = 0;
        locked_rotor_counter = 0;
        locked_rotor_recovery_counter = 0;
      }
      return;
    }

    sensor_data = ((HA_1()<<2)+(HB_1()<<1)+HC_1()) & 0b111;
    
    MOSFET_state = MOSFET_states_F[sensor_data];
    //sensor input, sensor read once stabilized
    //0b101 0b010
    //0b100 0b011
    //0b110 0b001
    //0b010 0b101
    //0b011 0b100
    //0b001 0b110
    /*MOSFET_state = MOSFET_states_F[0b001];
    move_counter++;

    if(move_counter > 2400)
    {
      move_counter = 0;
    }
    if(move_counter > 2000)
    {
    MOSFET_state = MOSFET_states_F[0b011];

    }
    else if(move_counter > 1600)
    {
      MOSFET_state = MOSFET_states_F[0b010];
  }
      else if(move_counter > 1200)
    {
      MOSFET_state = MOSFET_states_F[0b110];

    }
    else if(move_counter > 800)
    {
      MOSFET_state = MOSFET_states_F[0b100];

    }
    else if (move_counter > 400)
    {
      MOSFET_state = MOSFET_states_F[0b101];
    }*/

    if(MOSFET_state == last_MOSFET_state)
    {
      locked_rotor_counter++;
    }
    else
    {
      locked_rotor_counter = 0;
    }

    last_MOSFET_state = MOSFET_state;

    //rotors are locked after 200ms
    if(locked_rotor_counter > 200)
    {
      locked_rotor_flag = 1;
      MOSFET_state = 0x00;
    }
  
    if(MOSFET_state & 0b010000)
    {
      set_AHI_1_duty_cycle(0);;
      ALO_1_ON(1);
    }
    else if(MOSFET_state & 0b100000)
    {
      ALO_1_ON(0);
      //AHI_1_ON(1);
      set_AHI_1_duty_cycle(speed);
    }
    else
    {
      set_AHI_1_duty_cycle(0);
      ALO_1_ON(0);
    }
  
    if(MOSFET_state & 0b000100)
    {
      set_BHI_1_duty_cycle(0);
      BLO_1_ON(1);
    }
    else if(MOSFET_state & 0b001000)
    {
      BLO_1_ON(0);
      //BHI_1_ON(1);
      set_BHI_1_duty_cycle(speed);
    }
    else
    {
      BLO_1_ON(0);
      set_BHI_1_duty_cycle(0);;
    }
    
    if(MOSFET_state & 0b000001)
    {
      set_CHI_1_duty_cycle(0);
      CLO_1_ON(1);
    }
    else if(MOSFET_state & 0b000010)
    {
      CLO_1_ON(0);
      //CHI_1_ON(1);
      set_CHI_1_duty_cycle(speed);
    }
    else
    {
      set_CHI_1_duty_cycle(0);
      CLO_1_ON(0);
    }
  }

  if(fifty_us_counter > 20)
    fifty_us_counter = 0;


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

static void test_pwm(void)
{
  ALO_1_ON(0);
  BLO_1_ON(0);
  CLO_1_ON(0);

  while(1)
  {

  set_AHI_1_duty_cycle(500);
  set_BHI_1_duty_cycle(500);
  set_CHI_1_duty_cycle(500);

  block_ms(1000);

  set_AHI_1_duty_cycle(0);
  set_BHI_1_duty_cycle(0);
  set_CHI_1_duty_cycle(0);
  block_ms(1000);
  }

}

static void brushed_control(int speed)
{

  static unsigned int last_fifty_us_counter = 0;

  if(speed > 100)
    speed = 100;
  else if(speed < -100)
    speed = -100;  


  if(fifty_us_counter >=19)
  {
    //turn on low side if high side is on
    if(fifty_us_counter >=19)
    {
      if(speed < 0)
      { 
        set_AHI_1_duty_cycle(0);
        set_BHI_1_duty_cycle(0);
        ALO_1_ON(0);
        BLO_1_ON(1);
      }
      else
      {
        set_AHI_1_duty_cycle(0);
        set_BHI_1_duty_cycle(0);
        BLO_1_ON(0);
        ALO_1_ON(1);

      }
 
    }

  }

  //runs every 50us
  else if(fifty_us_counter != last_fifty_us_counter)
  {
    last_fifty_us_counter = fifty_us_counter;

    if(speed < 0)
    {
      BLO_1_ON(0);
      set_AHI_1_duty_cycle(0);
      ALO_1_ON(1);
      set_BHI_1_duty_cycle(speed*10);
    }
    else
    {
      ALO_1_ON(0);
      set_BHI_1_duty_cycle(0);
      BLO_1_ON(1);
      set_AHI_1_duty_cycle(speed*10);
    }
  }
  
  if(fifty_us_counter > 20)
    fifty_us_counter = 0;

}

static void brushless_init(void)
{
  T2InterruptUserFunction = Timer2_ISR;
  //T2 frequency is Fosc/2 = 16 MHz
  //Max timer interrupt at 2^16/16MHz = 4.096ms
  //Let's set PR2 to 800, to get interrupts every 50us:
  PR2 = 800;
  _T2IE = 1;
  T2CONbits.TON = 1;

  init_pwm();

  block_ms(5000);

}

static void brushed_init(void)
{
  T2InterruptUserFunction = Timer2_ISR;
  //T2 frequency is Fosc/2 = 16 MHz
  //Max timer interrupt at 2^16/16MHz = 4.096ms
  //Let's set PR2 to 800, to get interrupts every 50us:
  PR2 = 800;
  _T2IE = 1;
  T2CONbits.TON = 1;

  init_pwm_brushed();

  block_ms(5000);

}

void init_pwm_brushed(void)
{
	//Choose 400Hz (2.5ms period)
	//2.5ms = [PR2 + 1]*62.5ns*1
  //CHI_1_OR = 18; //OC1
  //Choose 8kHz (125us period)
  //125us = (PR2 + 1)*62.5ns
  //PR2 = 1999
  AHI_1_OR = 18;
  BHI_1_OR = 19;
  CHI_1_OR = 20;
	//PR2 = 40001;	

  //if both are 0, module doesn't stay low
  OC1R = 0;
	OC1RS = 2001;

  OC2R = 0;
  OC2RS = 2001;
  
  OC3R = 0;
  OC3RS = 2001;

  PR3 = 8000;


  //synchronize to Timer 3
	/*OC1CON2bits.SYNCSEL = 0b1101;	
	OC2CON2bits.SYNCSEL = 0b1101;	
	OC3CON2bits.SYNCSEL = 0b1101;	*/
OC1CON2bits.SYNCSEL = 0x1f;	
	OC2CON2bits.SYNCSEL = 0x1f;	
	OC3CON2bits.SYNCSEL = 0x1f;
	//use timer 3

  //Note:  We can't use PWM, as it is double-buffered
  //(to avoid shoot-through)
	//single-compare single-shot (starts high)
	OC1CON1bits.OCM = 5;
	OC2CON1bits.OCM = 5;
	OC3CON1bits.OCM = 5;

	OC1CON1bits.OCTSEL = 1;
	OC2CON1bits.OCTSEL = 1;
	OC3CON1bits.OCTSEL = 1;



	//turn on timer 3
	T3CONbits.TON = 1;


}

