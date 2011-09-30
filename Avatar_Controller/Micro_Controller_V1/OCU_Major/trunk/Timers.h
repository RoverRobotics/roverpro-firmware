void Init_Timer2(void);
void Init_Timer3(void);
void Init_Timer4(void);
void block_ms(unsigned int ms);

extern unsigned char event;
extern unsigned char osd_event;
extern unsigned char Controller_State;

#define NOT_CONNECTED 0x00
#define CONNECTED 0x01

#define NONE 0x00
#define SEND_TURNON_MESSAGE 0x01
#define CLEAR_LOGO 0x02
#define CLEAR_SCREEN 0x03
