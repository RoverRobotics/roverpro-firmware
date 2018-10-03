extern unsigned char new_motor_control_message_flag;
void parse_UART_message(void);
void init_uart(void);
void send_battery_message(void);
void messaging_FSM(void);

extern char last_motor_commands[2];

