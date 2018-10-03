#define PGED2_RPn _RP7R
#define U2TX_NUM 5
void send_lcd_string(char* input_string, unsigned char len);
void print_loop_number(void);
void display_board_number(void);
void display_int_in_hex(char* description, unsigned int int_to_display);
void init_debug_uart(void);
void send_debug_uart_string(char* input_string, unsigned char len);

void display_int_in_dec(char* description, unsigned int int_to_display);
