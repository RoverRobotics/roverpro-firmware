void Datalink_Init(void);
char Handle_Messages(void);
char Is_Message_Valid(void);
unsigned int return_crc(unsigned char* data, unsigned char length);
unsigned char Return_Last_Datalink_Message(unsigned char ldm);
unsigned char Set_Last_Datalink_Message(unsigned char byte, unsigned char index);
#define DATALINK_MESSAGE_LENGTH 14
