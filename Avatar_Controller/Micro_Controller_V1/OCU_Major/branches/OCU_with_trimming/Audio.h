#define OUTPUT 0
#define INPUT 1
#define OPEN_DRAIN 1

#define POWER_DOWN_TRIS TRISDbits.TRISD8
#define SEL0_TRIS		TRISDbits.TRISD11
#define SEL1_TRIS		TRISDbits.TRISD0

#define POWER_DOWN_LAT	LATDbits.LATD8
#define SEL0_LAT		LATDbits.LATD11
#define SEL1_LAT		LATDbits.LATD0

#define MUTE_BUTTON_TRIS	TRISCbits.TRISC14
#define MUTE_BUTTON			PORTCbits.PORTC14

#define VOLUME_TRIS			TRISCbits.TRISC13
#define VOLUME_DRAIN		ODCCbits.ODC13
#define VOLUME				LATEbits.LATE5

//for OCU 2.00
//#define VOLUME				LATBbits.LATB6



//Modules
void Audio_Test(void);
void Init_Audio(void);
void Audio_Transmit(void);
void Audio_Receive(void);
void Audio_Mute(void);
void Audio_Unmute(void);
