#include "timer.h"
#include "interrupt_switch.h"
#include "stdhdr.h"

unsigned int millisecond_counter = 0;
unsigned int ten_millisecond_counter = 0;
unsigned int hundred_millisecond_counter = 0;
unsigned int second_counter = 0;

static void T2_ISR(void)
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


void InitTimer(void)
{

  //T2 frequency is Fosc/2 = 16 MHz
  //Max timer interrupt at 2^16/16MHz = 4.096ms
  //Let's set PR2 to 16000, to get interrupts every 1ms:
  T2InterruptUserFunction=T2_ISR;
  
  PR2 = 16000;
  _T2IE = 1;
  T2CONbits.TON = 1;

}
