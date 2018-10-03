void Init_Controller(void);
void Construct_Controller_Message(void);

#include "Datalink.h"


//#define BUTTON_1 PORTBbits.RB12
#define LIGHT_BUTTON PORTBbits.RB15
#define TALK_BUTTON PORTBbits.RB14
//#define BUTTON_4 PORTBbits.RB13
#define FLIPPER_UP PORTEbits.RE2
#define FLIPPER_DOWN PORTEbits.RE1
#define LEFT_TRIGGER PORTDbits.RD6
#define RIGHT_TRIGGER PORTDbits.RD5
#define SELECT_POS_1 PORTDbits.RD1
#define SELECT_POS_2 PORTDbits.RD2
#define PAYLOAD_BUTTON PORTBbits.RB13
//#define BUTTON_7 PORTEbits.RE3
//#define BUTTON_8 PORTEbits.RE0

/*#define BUTTON_1_MASK 0x80
#define LIGHT_BUTTON_MASK 0x10
#define TALK_BUTTON_MASK 0x20
#define BUTTON_4_MASK 0x02
#define TALK_MASK 0x40
#define LISTEN_MASK 0x20
#define FLIPPER_UP_MASK 0x08
#define FLIPPER_DOWN_MASK 0x04
//#define BUTTON_7_MASK 0x02
//#define BUTTON_8_MASK 0x01*/

#define LIGHT_ON_MASK 0x80
#define OCU_TO_ROBOT_TALK_MASK 0x40
#define PAYLOAD_BUTTON_MASK 0x20
#define FLIPPER_UP_MASK 0x10
#define FLIPPER_DOWN_MASK 0x08
#define LEFT_TRIGGER_MASK 0x04
#define RIGHT_TRIGGER_MASK 0x02

//#define TRIGGER_L PORTEbits.RE5

#define SELECT1_MASK 0x80
#define SELECT0_MASK 0x40

#define TRIGGER_R PORTEbits.RE4

#define AUDIO_TX 0
#define AUDIO_RX 1
#define AUDIO_DISABLE 2
#define AUDIO_ENABLE 3

#define LH_OFFSET -7
#define LV_OFFSET -2
#define RH_OFFSET -10
#define RV_OFFSET 11
#define LH_SCALE_POS 2.05
#define LV_SCALE_POS 2.19
#define RH_SCALE_POS 2.70
#define RV_SCALE_POS 2.44
#define LH_SCALE_NEG 3.53
#define LV_SCALE_NEG 3.63
#define RH_SCALE_NEG 2.49
#define RV_SCALE_NEG 3.02 


#define TRANSMIT 1
#define RECEIVE 0
/*#define LH_OFFSET 0
#define LV_OFFSET 0
#define RH_OFFSET 0
#define RV_OFFSET 0
#define LH_SCALE_POS 1
#define LV_SCALE_POS 1
#define RH_SCALE_POS 1
#define RV_SCALE_POS 1
#define LH_SCALE_NEG 1
#define LV_SCALE_NEG 1
#define RH_SCALE_NEG 1
#define RV_SCALE_NEG 1*/

#define ON 1
#define OFF 0


extern unsigned char OCU_ADDRESS_MSB;
extern unsigned char OCU_ADDRESS_LSB;
extern unsigned char ROBOT_ADDRESS_MSB;
extern unsigned char ROBOT_ADDRESS_LSB;
