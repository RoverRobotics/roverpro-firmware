

void RS485_Send_Byte(char byte);
void RS485_Init(void);
void RS485_Send(unsigned char* message,unsigned char length);
void RS485_Send_String(const ROM char *data,unsigned char length);

//used to test subsystem
void RS485_test(void);

extern unsigned char RS485_Message[40];
extern unsigned char RS485_Message_Length;
extern unsigned char RS485_Message_Index;

extern unsigned char RS485_RX_Message[10];
extern unsigned char RS485_RX_Length;
extern unsigned char RS485_Message_Ready;
