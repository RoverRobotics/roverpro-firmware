
void Map_Pins(void);


//ADC input mapping		// an0, an1 reserved for ICSP
#define AN_VOLT_PIN  	AD1PCFGbits.PCFG2			//voltage input on AN3 (flipper pot)
#define ADC_VOLT_CHAN	2

#define AN_CUR1_PIN		AD1PCFGbits.PCFG3
#define ADC_CUR1_CHAN	3

#define AN_CUR2_PIN		AD1PCFGbits.PCFG4
#define ADC_CUR2_CHAN	4

#define AN_CUR3_PIN		AD1PCFGbits.PCFG5
#define ADC_CUR3_CHAN	5

//PPS Outputs
#define NULL_IO		0
#define C1OUT_IO	1
#define C2OUT_IO	2
#define U1TX_IO		3
#define U1RTS_IO	4
#define U2TX_IO		5
#define U2RTS_IO	6
#define SDO1_IO		7
#define SCK1OUT_IO	8
#define SS1OUT_IO	9
#define SDO2_IO		10
#define SCK2OUT_IO	11
#define SS2OUT_IO	12
#define OC1_IO		18
#define OC2_IO		19
#define OC3_IO		20
#define OC4_IO		21
#define OC5_IO		22
#define OC6_IO		23
#define OC7_IO		24
#define OC8_IO		25
#define U3TX_IO		28
#define U3RTS_IO	29
#define U4TX_IO		30
#define SDO3_IO		31
#define SCK3OUT_IO	33
#define SS3OUT_IO	34
#define OC9_IO		35


