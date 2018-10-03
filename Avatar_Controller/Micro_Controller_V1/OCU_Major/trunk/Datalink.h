void Datalink_Message_Send(void);
void Datalink_Init(void);
unsigned int return_crc(unsigned char* data, unsigned char length);

//used in testing -- I hate test harnesses
void Datalink_loop(void);
void Datalink_Test(void);
unsigned char Is_Robot_Powered_On(void);
void set_9xtend_channel(const ROM char *channel);

extern unsigned char Datalink_Buffer[20];
extern unsigned char Last_Datalink_Message[20];

extern unsigned char Datalink_Buffer_Index;
extern unsigned char Datalink_Buffer_Full;

extern unsigned char Datalink_Send[20];
extern unsigned char Datalink_Send_Buffer[20];
extern unsigned char Robot_Powered_On;


#define Datalink_Rx_Port 10
#define Datalink_Tx_POR RPOR8bits.RP17R
