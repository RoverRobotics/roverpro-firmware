/*=============================================================================
File: ADC.c

Notes:
	1) requires (2 + num_AD_bits)*T_AD to complete conversion, where T_AD = 
     time required to convert 1 bit on the A/D.
	2) When the conversion time is complete, the result is loaded
     into one of 16 A/D result buffers and an interrupt fires
	3) t_total = t_sample + t_conversion


(see p.259 of datasheet)
T_AD = T_CY*(ADCS+1)
             ADCS = T_AD/T_CY - 1
where T_AD = A/D conversion clock period
      T_CY = 2/f_osc, instruction cycle period

=============================================================================*/
#define TEST_ADC
/*---------------------------Dependencies------------------------------------*/
#include "ADC.h"
#include "ConfigurationBits.h"

/*---------------------------Macros and Definitions--------------------------*/
#define MAX_NUM_AD_INPUTS     16 // maximum number of analog-to-digital inputs
                                 // this module can handle (16 counting 0)
                                
/*---------------------------Helper-Function Prototypes----------------------*/
static void ConfigurePins(void);
static void ConfigureInterrupt(void);
static inline void SelectAnalogPin(unsigned char analog_pin_index);
static void ExecuteADC_ISR();

/*---------------------------Module Variables--------------------------------*/
static unsigned int V_ADC[MAX_NUM_AD_INPUTS] = {0};

/*---------------------------Test Harness------------------------------------*/
#ifdef TEST_ADC

int main(void) {
  TRISEbits.TRISE5 = 0; PORTEbits.RE5 = 0; // initialize a debugging pin
  InitADC();

	while (1) {
		Nop();
    if (GetADC(0) >= 500) PORTEbits.RE5 = 1;
		else PORTEbits.RE5 = 0;
	}

	return 0;
}

#endif
/*---------------------------End Test Harness--------------------------------*/

/*---------------------------Public Function Definitions---------------------*/
/*
Function: InitADC()
Parameters:
	unsigned int bit_mask, a number interpreted as binary that describes which
  											 of the 0-though-15 analog pins are desired to be
                         analog inputs  
Description: Initializes the specified analog-to-digital-conversion (ADC)
	hardware module.
Usage: InitADC(0b0000 0000 000 0000)
*/
void InitADC(void) {
	ConfigurePins();
	ConfigureInterrupt();
	AD1CON1bits.ASAM = 1; // begin auto-sampling
}

/*
Function: GetADC()
Description: Returns the analog value in arbitrary units [au] on the given pin
Notes:
	1) results are placed into address ADCBUF0 and each sequential buffer 
     address on successive interrupts
*/
unsigned int GetADC(unsigned char analog_input_index) {
  return V_ADC[analog_input_index];
}

void _ISR _ADC1Interrupt(void) {
	ExecuteADC_ISR();
}

/*---------------------------Private Function Definitions--------------------*/
/*
Function: ExecuteADC_ISR()
Description: Cycles through each result as it it produced, updating a 
  software buffer (module-level array)
Notes:
	1) results are placed into address ADCBUF0 and each sequential buffer 
     address on successive interrupts
*/
static void ExecuteADC_ISR() {
	static unsigned char index = 0;

	V_ADC[index] = ADC1BUF0;
	if ((MAX_NUM_AD_INPUTS - 1) < ++index) index = 0;
	SelectAnalogPin(index);
	IFS0bits.AD1IF = 0;     // clear the source of the interrupt
}

/*
Function: ConfigurePins()
Description: Configures and initializes an pins associated with this 
  module.
*/
static void ConfigurePins(void) {
	// configure port pin(s) as analog input(s)
	TRISBbits.TRISB0 = 1; AD1PCFGLbits.PCFG0 = 0;
	TRISBbits.TRISB1 = 1; AD1PCFGLbits.PCFG1 = 0;
	TRISBbits.TRISB4 = 1; AD1PCFGLbits.PCFG4 = 0;
	TRISBbits.TRISB5 = 1; AD1PCFGLbits.PCFG5 = 0;

  // select the appropriate pin to sample
	AD1CHSbits.CH0NA = 0b000;		// configure the negative reference to ground (V_ss)
  SelectAnalogPin(0);					// select the first analog pin to sample
  AD1CON2bits.BUFM = 0;       // start filling buffer at address, ADCBUF0, and 
                              // each sequential address on successive interrupts    
	
	// select voltage reference source to match expected range on analog
	AD1CON2bits.VCFG = 0b000;		// configure V_ref+ = V_dd (positive rail)
	                            // and V_ref- = V_ss (ground)

	// select the analog conversion clock to match the desired data rate with the processor clock
	AD1CON3bits.ADRC = 0; 			// use the system clock for A/D conversion
	AD1CON3bits.ADCS = 0b00011111;// T_AD = T_CY*(ADCS + 1)

	// select the appropriate sample/conversion sequence
	AD1CON1bits.SSRC = 0b0111; 	// internal counter ends sampling and starts conversion?
	AD1CON3bits.SAMC = 0b11111;	// configure how often to auto-sample (11111 = 31 TAD)
	// select how conversion results are presented in the buffer
	AD1CON1bits.FORM = 0b00;		// configure the result as unsigned, right-justified decimal
}

/*
Function: ConfigureInterrupt()
Notes:
	1) interrupts to fire at the completion of every 32nd A/D conversion
*/
static void ConfigureInterrupt() {
	AD1CON2bits.SMPI = 0b01111;	// configure the interrupt rate
	AD1CON1bits.ADON = 1; 			// turn on A/D module
	IFS0bits.AD1IF = 0; 				// clear the AD1 interrupt flag
	// configure the A/D interrupt priority (leave as default)
	IEC0bits.AD1IE = 1; 				// enable the AD1 interrupt
}

static inline void SelectAnalogPin(unsigned char analog_pin_index) {
	AD1CHSbits.CH0SA = analog_pin_index;	// configure the positive reference
                                        // to the pin to sample	
}
/*---------------------------End of File-------------------------------------*/
