#define CODE_VERSION "RX v1.05"

#define MESSAGE_TO_PAYLOAD_LENGTH 9
#define MESSAGE_FROM_PAYLOAD_LENGTH 9

#define NONE 0x00
#define PTZ 0x01
#define ARM 0x02

void init_uart2(void);
void payload_robot_control(void);
void stop_payload_motion(void);
void return_version_info(void);
void handle_payload_message(void);
void send_message_to_payload(unsigned char* data_to_send);

char Is_payload_CRC_valid(unsigned char* data, unsigned char length);

void set_payload_type(unsigned char new_type);
void reset_payload(void);
unsigned char return_payload_type(void);
void arm_drive(unsigned char L_LR,unsigned char L_UD);

extern unsigned char message_from_payload_ready;
extern unsigned char message_from_payload[MESSAGE_FROM_PAYLOAD_LENGTH];

extern unsigned char arm_in_use;
