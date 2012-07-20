
#include "system.h"

/*****************************************************************************
 * Module Constants Definitions 
 *****************************************************************************/
#define ADC_FLIP_POS      	ADC_VOLT_CHAN	//ADC channels numbers, Flipper position sensor
#define ADC_FLIP_CUR		ADC_CUR1_CHAN	//Flipper current channel
#define ADC_L_CUR			ADC_CUR2_CHAN	//Left motor current draw
#define ADC_R_CUR			ADC_CUR3_CHAN	//Right motor current draw
#define AVCC                3300			//Reference voltage, mV

/*****************************************************************************
 * Module Level Variables 
 *****************************************************************************/
static unsigned int AD_Flipper_Pos;
static unsigned int AD_Flipper_Cur;
static unsigned int AD_L_Mot_Cur;
static unsigned int AD_R_Mot_Cur;

/*****************************************************************************
* Function: ADCInit
******************************************************************************/
void ADCInit(){
	AD1CON1 = 0b1000000011100100;	//Turn on, auto sample start, auto-convert, integer format (0000 00dd dddd dddd)
	AD1CON2 = 0b0000000000111100;	//AVdd, AVss, int every 16th conversion, MUXA only
	AD1CON3 = 0b0001111100000101;	//31 Tad auto-sample, Tad = 5*Tcy
	AD1CHS = ADC_FLIP_POS;			//AD Channel select bit, can only be set to sample one channel at a time
	AD1CSSL = 0;					//No scanned inputs
}

/*****************************************************************************
* Function: ADCProcessEvents
******************************************************************************/
void ADCProcessEvents()
{
static unsigned char Last_AD_State;

	if (AD1CON1bits.DONE){			//if conversion has finished
		Last_AD_State++;
		switch (Last_AD_State){
			case 1:							//Sample AN2, Flipper Position
				AD_Flipper_Pos = ADC1BUF0;
				AD1CHS = ADC_FLIP_CUR;		//Setup for case 2
  				break;
			case 2:							//Sample AN3, Flipper Current
				AD_Flipper_Cur = ADC1BUF0;
				AD1CHS = ADC_L_CUR;			//Setup for case 3
  				break;
			case 3:							//Sample AN4, Left Motor Current
				AD_L_Mot_Cur = ADC1BUF0;
				AD1CHS = ADC_R_CUR;			//Setup for case 4
				break;
			case 4:							//Sample AN5, Right Motor Current
				AD_R_Mot_Cur = ADC1BUF0;
				AD1CHS = ADC_FLIP_POS;		//Setup for case 1
				Last_AD_State = 0;			//Setup for case 1
				break;
			default:
				Last_AD_State = 0;
  				break;
		}//end switch
	}//end if
}//end function

/*****************************************************************************
* Function: Pull Flipper Position Value
******************************************************************************/
unsigned int Pull_Flip_Pos_AD(void){
	return AD_Flipper_Pos;
}

/*****************************************************************************
* Function: Pull Flipper Current Value
******************************************************************************/
unsigned int Pull_Flip_Cur_AD(void){
	return AD_Flipper_Cur;
}

/*****************************************************************************
* Function: Pull Left Motor Current Value
******************************************************************************/
unsigned int Pull_L_Mot_Cur_AD(void){
	return AD_L_Mot_Cur;
}

/*****************************************************************************
* Function: Pull Right Motor Current Value
******************************************************************************/
unsigned int Pull_R_Mot_Cur_AD(void){
	return AD_R_Mot_Cur;
}
