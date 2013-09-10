#define MESSAGE_FROM_ROBOT_LENGTH 9
#define MESSAGE_TO_ROBOT_LENGTH 9

extern unsigned char message_to_robot[MESSAGE_TO_ROBOT_LENGTH];

extern unsigned char message_from_robot[MESSAGE_FROM_ROBOT_LENGTH];
extern unsigned char message_from_robot_ready;


char Is_CRC_valid(unsigned char* data, unsigned char length);
void send_message_to_robot(unsigned char* data_to_send);
void init_usart1(void);
