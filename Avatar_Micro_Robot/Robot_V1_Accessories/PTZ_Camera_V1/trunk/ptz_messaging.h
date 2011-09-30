extern unsigned char ptz_message[8];

void decode_message_from_robot(void);
void init_uart2(void);

#define PTZ_MESSAGE_LENGTH 8

void block_ms(unsigned int ms);
