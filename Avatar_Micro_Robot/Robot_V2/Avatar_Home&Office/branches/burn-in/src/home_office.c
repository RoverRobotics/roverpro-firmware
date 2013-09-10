#include "home_office.h"
#include "charging.h"
#include "./core/ConfigurationBits.h"
#include "motor_control.h"
#include "uart.h"

#define WAIT0_S     10
#define TURN_S      5
#define WAIT1_S     5
#define FORWARD_S   30
#define WAIT2_S     30
#define REVERSE_S   30

#define WAIT3_S     60


static int desired_motor_commands[2] = {0,0};
static void motor_accel_FSM(void);
static void InitPins(void);
static void start_up(void);
static void ADC_Init(void);
static void robot_burn_in(void);
static void wait_ten_ms(unsigned int ten_ms);
static int test_individual_motors(void);


static void InitTimer(void);
static unsigned int millisecond_counter = 0;
static unsigned int ten_millisecond_counter = 0;
static unsigned int hundred_millisecond_counter = 0;
static unsigned int second_counter = 0;



int main(void)
{

  static unsigned int start_burn_in_counter = 0;
  static unsigned int turn_off_counter = 0;

  //static unsigned int stop_burn_in = 0;

  InitPins();
  start_up();
  InitTimer();
  ESC_Init();
  ADC_Init();
  init_uart();
  i2c3_init();


  wait_ten_ms(10);
  while(PWR_BUTTON())
  {
    ClrWdt();
  }
  wait_ten_ms(10);

  hundred_millisecond_counter = 0;

  //motor_control_test_function();

  while(1)
  {
    if(ten_millisecond_counter)
    {  

      if(PWR_BUTTON())
      {
        turn_off_counter++;
      }
      else
      {
        turn_off_counter = 0;
      }
      if(turn_off_counter > 200)
      {
          SYS_BUS_ON(0);
          V5_ON(0);
          while(PWR_BUTTON())
            ClrWdt();
          while(1)
          {
            ClrWdt();
          }
      }


      motor_accel_FSM();
      motor_control_FSM();
      battery_FSM();
      ten_millisecond_counter = 0;
    }
    if(hundred_millisecond_counter >= 10)
    {
      if(PWR_BUTTON())
      {
        //only proceed if button was previously released
        if(start_burn_in_counter == 0)
        {
          //block until button is released
          wait_ten_ms(10);
          while(PWR_BUTTON())
            ClrWdt();
          //change state
          start_burn_in_counter = 1;
        }
      }

      hundred_millisecond_counter = 0;
      if(start_burn_in_counter)
      {
        robot_burn_in();
      }
  
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

 
  //If power button isn't pressed and robot isn't on the dock, don't continue.
  //This case happens if the robot is pushed on the ground, and the power from the motors
  //wakes up the robot
  if( (BQ24745_ACOK()==0) && (PWR_BUTTON()==0) )
  {
    while(1);
  }

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

static void robot_burn_in(void)
{

    typedef enum {
    sWait0 = 0,
    sForward,
    sWait1,
    sReverse,
    sWait2,
    sRightTurn,
    sLeftTurn,
    sWait3,
  } sBurninState;

  static sBurninState state = sWait0;

  static unsigned int state_timer = 0;
  static unsigned int cycle_counter = 0;

  state_timer++;

  if(cycle_counter > 10)
  {
    last_motor_commands[0] = 0;
    last_motor_commands[1] = 0;
    return;
  }

  switch(state)
  {
    case sWait0:
      desired_motor_commands[0] = 0;
      desired_motor_commands[1] = 0;
      if(state_timer >= WAIT0_S)
      {
        if(test_individual_motors())
        {
          state_timer = 0;
          state = sRightTurn;
        }
      }
    break;
    case sRightTurn:
      desired_motor_commands[0] = 100;
      desired_motor_commands[1] = -100;
      if(state_timer >= TURN_S)
      {
        state_timer = 0;
        state = sLeftTurn;
      }
    break;
    case sLeftTurn:
      desired_motor_commands[0] = -100;
      desired_motor_commands[1] = 100;
      if(state_timer >= TURN_S)
      {
        state_timer = 0;
        state = sWait1;
      }
    break;
    case sWait1:
      desired_motor_commands[0] = 0;
      desired_motor_commands[1] = 0;
      if(state_timer >= WAIT1_S)
      {
        state_timer = 0;
        state = sForward;
      }
    break;
    case sForward:
      desired_motor_commands[0] = 100;
      desired_motor_commands[1] = 100;
      if(state_timer >= FORWARD_S)
      {
        state_timer = 0;
        state = sWait2;
      }
    break;
    case sWait2:
      desired_motor_commands[0] = 0;
      desired_motor_commands[1] = 0;
      if(state_timer >= WAIT2_S)
      {
        state_timer = 0;
        state = sReverse;
      }
    break;
    case sReverse:
      desired_motor_commands[0] = -100;
      desired_motor_commands[1] = -100;
      if(state_timer >= REVERSE_S)
      {
        state_timer = 0;
        state = sWait3;
      }
    break;
    case sWait3:
      desired_motor_commands[0] = 0;
      desired_motor_commands[1] = 0;
      if(state_timer >= WAIT3_S)
      {
        cycle_counter++;
        state_timer = 0;
        state = sRightTurn;
      }
    break;


  }


}


static void motor_accel_FSM(void)
{
  unsigned int i;

  static int last_motor_commands_temp[2] = {0,0};

  for(i=0;i<2;i++)
  {

    if(last_motor_commands_temp[i] < desired_motor_commands[i])
      last_motor_commands_temp[i]+=1;
    else if(last_motor_commands_temp[i] > desired_motor_commands[i])
      last_motor_commands_temp[i]-=1;


      if(desired_motor_commands[i] == 0)
        last_motor_commands_temp[i] = 0;
  
    if( (last_motor_commands_temp[i] > 40) || (last_motor_commands_temp[i] < -40) )
      last_motor_commands[i] = last_motor_commands_temp[i];
    else
      last_motor_commands[i] = 0;

    //reverse left motor
    if(i==0)
      last_motor_commands[i]*=-1;
  }




}

static void wait_ten_ms(unsigned int ten_ms)
{
  ten_millisecond_counter = 0;
  while(ten_millisecond_counter < ten_ms)
  {
    ClrWdt();
  }

}

static int test_individual_motors(void)
{
  static unsigned int state_timer = 0;

    typedef enum {
    sLeftForward = 0,
    sIndWait1,
    sRightForward,
    sIndWait2,
  } sMotorTestState;

  static sMotorTestState state = sLeftForward;

  state_timer++;

  switch(state)
  {
    case sLeftForward:
      desired_motor_commands[0] = 100;
      desired_motor_commands[1] = 0;
      if(state_timer >= TURN_S)
      {
        state_timer = 0;
        state = sIndWait1;
      }
    break;
    case sIndWait1:
      desired_motor_commands[0] = 0;
      desired_motor_commands[1] = 0;
      if(state_timer >= TURN_S)
      {
        state_timer = 0;
        state = sRightForward;
      }
    break;
    case sRightForward:
      desired_motor_commands[0] = 0;
      desired_motor_commands[1] = 100;
      if(state_timer >= TURN_S)
      {
        state_timer = 0;
        state = sIndWait2;
      }
    break;
    case sIndWait2:
      desired_motor_commands[0] = 0;
      desired_motor_commands[1] = 0;
      if(state_timer >= TURN_S)
      {
        state_timer = 0;
        return 1;
      }
    break;
  }

  return 0;
  
}
