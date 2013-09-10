//#define MESSAGE_FROM_ROBOT_LENGTH 9
//#define MESSAGE_TO_ROBOT_LENGTH 9

#define PREV_PAYLOAD_RX_LENGTH 9
#define PREV_PAYLOAD_TX_LENGTH 9
#define NEXT_PAYLOAD_RX_LENGTH 9
#define NEXT_PAYLOAD_TX_LENGTH 9

//extern unsigned char message_to_robot[MESSAGE_TO_ROBOT_LENGTH];

//extern unsigned char message_from_robot[MESSAGE_FROM_ROBOT_LENGTH];
//extern unsigned char message_from_robot_ready;


extern unsigned char previous_payload_rx[PREV_PAYLOAD_RX_LENGTH];
extern unsigned char previous_payload_tx[PREV_PAYLOAD_TX_LENGTH];
extern unsigned char next_payload_rx[NEXT_PAYLOAD_RX_LENGTH];
extern unsigned char next_payload_tx[NEXT_PAYLOAD_TX_LENGTH];


extern unsigned char prev_payload_msg_ready;
extern unsigned char next_payload_msg_ready;


char Is_CRC_valid(unsigned char* data, unsigned char length);
void send_message_to_prev_payload(unsigned char* data_to_send);
void send_message_to_next_payload(unsigned char* data_to_send);
void init_usart1(void);
void init_usart2(void);

//used by main function to pass messages
extern unsigned char prev_payload_rx_last_good[PREV_PAYLOAD_RX_LENGTH];
extern unsigned char next_payload_rx_last_good[NEXT_PAYLOAD_RX_LENGTH];


void send_payload_ID(void);
