/*==============================================================================
File: ADC.c

Notes:
	- requires (2 + num_AD_bits)*T_AD to complete conversion, where T_AD = 
     time required to convert 1 bit on the A/D.
	- When the conversion time is complete, the result is loaded
     into one of 16 A/D result buffers and an interrupt fires
	- t_total = t_sample + t_conversion


(see p.259 of datasheet)
T_AD = T_CY*(ADCS+1)
             ADCS = T_AD/T_CY - 1
where T_AD = A/D conversion clock period
      T_CY = 2/f_osc, instruction cycle period

==============================================================================*/
//#define TEST_ADC
/*---------------------------Dependencies-------------------------------------*/
#include "./ADC.h"

/*---------------------------Macros and Definitions---------------------------*/
#define MAX_N_AD_INPUTS       16 // maximum number of analog-to-digital inputs
                                 // this module can handle
                                
/*---------------------------Helper-Function Prototypes-----------------------*/
static void ConfigurePins(uint16_t bit_mask);
static void ConfigureInterrupt(void);
static inline void SelectAnalogPin(uint8_t analog_pin_index);
static void ADC_ExecuteISR(void);

/*---------------------------Module Variables---------------------------------*/
static uint16_t consumed_pins = 0;		// bit mask of consumed pins
static uint16_t buffer[MAX_N_AD_INPUTS] = {0};

/*---------------------------Test Harness-------------------------------------*/
#ifdef TEST_ADC
#include "./ConfigurationBits.h"

int main(void) {
  _TRISE5 = 0; _RE5 = 0; // initialize a debugging pin
  ADC_Init(0b110011);

	while (1) {
    if (500 <= ADC_Get(0)) _RE5 = 1;
		else _RE5 = 0;
	}
	
	ADC_Deinit();
	
	return 0;
}

#endif
/*---------------------------Public Function Definitions----------------------*/
void ADC_Init(uint16_t bit_mask) {
	consumed_pins = bit_mask;
	uint8_t i;
	for (i = 0; i < MAX_N_AD_INPUTS; i++) buffer[i] = 0;
	ConfigurePins(consumed_pins);
	ConfigureInterrupt();
	AD1CON1bits.ASAM = 1; 			// begin auto-sampling
}


uint16_t ADC_value(uint8_t analog_input_index) {
  return buffer[analog_input_index];
}


void ADC_Deinit(void) {
	// turn OFF the A/D module and restore any registers to startup defaults
	// (EXCEPT AD1PCFGL -- don't make analog by default, just undo)
	AD1CON1 = 0x0000; AD1CON2 = 0x0000; AD1CON3 = 0x0000; AD1CHS = 0x0000;
	
	// restore any pins
	TRISB &= ~(consumed_pins);	// return any inputs to their default as outputs
	AD1PCFGL |= consumed_pins;	// 1 = pin in digital mode
	
	// clear any module-level variables
	consumed_pins = 0;
	uint8_t i;
	for (i = 0; i < MAX_N_AD_INPUTS; i++) buffer[i] = 0;	
}


void __attribute__((__interrupt__, auto_psv)) _ADC1Interrupt(void) {
	ADC_ExecuteISR();
}

/*---------------------------Private Function Definitions---------------------*/
/*
Function: ADC_ExecuteISR
Description: Cycles through each result as it is produced, updating a 
  software buffer (module-level array)
Notes:
	- results are placed into address ADCBUF0 and each sequential buffer 
    address on successive interrupts
*/
static void ADC_ExecuteISR(void) {
	static uint8_t index = 0;

	buffer[index] = ADC1BUF0;
	if ((MAX_N_AD_INPUTS - 1) < ++index) index = 0;
	SelectAnalogPin(index);
	IFS0bits.AD1IF = 0;     		// clear the source of the interrupt
}


static void ConfigurePins(uint16_t bit_mask) {
	// configure port pin(s) as analog input(s)
	TRISB |= bit_mask;
	AD1PCFGL &= ~(bit_mask);

  // select the appropriate pin to sample
	AD1CHSbits.CH0NA = 0b000;		// configure negative reference to ground (V_ss)
  SelectAnalogPin(0);					// select the first analog pin to sample
  AD1CON2bits.BUFM = 0;       // fill the buffer at address ADCBUF0 and each 
                              // sequential address on successive interrupts    
	
	// select voltage reference source to match expected range
	AD1CON2bits.VCFG = 0b000;		// configure V_ref+ = V_dd (positive rail)
	                            // and V_ref- = V_ss (ground)

	// select the analog conversion clock to match the desired data rate
	AD1CON3bits.ADRC = 0; 			// use the system clock for A/D conversion
	AD1CON3bits.ADCS = 0b00011111;// T_AD = T_CY*(ADCS + 1)

	// select the appropriate sample/conversion sequence
	AD1CON1bits.SSRC = 0b0111; 	// use an internal counter to end sampling and 
	                            // start the conversion
	AD1CON3bits.SAMC = 0b11111;	// configure how often to auto-sample (SAMC * TAD)	
}


static void ConfigureInterrupt(void) {
	AD1CON2bits.SMPI = 0b01111;	// configure the interrupt rate
	AD1CON1bits.ADON = 1; 			// turn on A/D module
	IFS0bits.AD1IF = 0; 				// clear the AD1 interrupt flag
	IEC0bits.AD1IE = 1; 				// enable the AD1 interrupt
}


static inline void SelectAnalogPin(uint8_t analog_pin_index) {
	AD1CHSbits.CH0SA = analog_pin_index;	// configure the positive reference
                                        // to the pin to sample	
}
