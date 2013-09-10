

/*****************************************************************************
* Function: ADCInit
*
* Preconditions: None.
*
* Overview: This function initiates ADC and state machine.
*
* Input: None.
*
* Output: None.
*
******************************************************************************/
extern void ADCInit();

/*****************************************************************************
* Function: ADCProcessEvents
*
* Preconditions: ADCInit must be called before.
*
* Overview: This is a state mashine to grab analog data from potentiometer
* and temperature sensor. Must be called periodically to refresh voltage
* and temperature strings.
*
* Input: None.
*
* Output: None.
*
******************************************************************************/
extern void ADCProcessEvents();

//return variable functions
unsigned int Pull_Flip_Pos_AD(void);
unsigned int Pull_Flip_Cur_AD(void);
unsigned int Pull_L_Mot_Cur_AD(void);
unsigned int Pull_R_Mot_Cur_AD(void);
