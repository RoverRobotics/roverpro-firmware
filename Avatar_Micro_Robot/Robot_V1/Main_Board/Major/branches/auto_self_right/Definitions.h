// Pins
#define Audio_pin					PORTEbits.RE4		//PIC_AUDIO_PD_IO in schematic, for some reason
#define Amp_pin						LATEbits.LATE3		//sets volume to zero
//#define Led_pin						ODCEbits.ODE4
#define Led_pin						LATEbits.LATE5
#define Relay_Power_Control			LATBbits.LATB10		//Relay1 on schematic
#define Relay_Resistor_Control		LATBbits.LATB9		//Relay2 on schematic
#define Mic_pin 					LATDbits.LATD1
#define VIDEO_SEL0					LATBbits.LATB2
#define VIDEO_SEL1					LATBbits.LATB3

//CS for MAX7456 OSD
#define CS_LAT						LATBbits.LATB4

// Messaging 
#define CONTROLLER1 	0x00
#define CONTROLLER2		0x00
#define ROBOT1			0x00
#define ROBOT2			0x00

// Mechanical
#define CENTER 			127.5
#define EXPONENTIAL		1
#define STEER_MODE		1
#define	STEER_DEADZONE	5
#define MOTOR_SCALE_L	2
#define MOTOR_SCALE_R	2

// General defines used globally
#define OUTPUT 			0
#define INPUT 			1
#define OFF 			0
#define ON 				1
#define YES 			1
#define NO 				0
#define OPEN_DRAIN 		1

// Commands


// Arbitrary Variables
#define TRANSMIT		1
#define RECEIVE			2
#define PD				3



