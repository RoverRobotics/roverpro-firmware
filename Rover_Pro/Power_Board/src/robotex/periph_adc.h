/**
 * @file periph_adc.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * API for ADC devices
 *
 *
 *
 * Firmware MUST configure/enable it's ADC hardware AND enable interrupts
 */


// callback prototype
typedef void (*ADC_FUNC)(unsigned char addr,
                         unsigned int  data);

// registers callback for ADC read operations
int registerADCReadCallback(ADC_FUNC func, unsigned char addr);

// registers callback for ADC channel range
int registerADCReadCallbackRange(ADC_FUNC func, unsigned char addr_low,
                                                unsigned char addr_high);


/* asyncronous ADC channel read - calls message handler
 returns -1 if busy, -2 invalid, 0 successful */
int readADC(unsigned char addr); /* ADC input address */


// syncronous version of the function above returns value
unsigned int readADCBlocking(unsigned char addr);


// returns 1 if ADC bus busy otherwise returns 0
int isBusyADC();


