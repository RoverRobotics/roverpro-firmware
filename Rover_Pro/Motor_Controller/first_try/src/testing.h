#define U1TX_RPn _RP7R
void send_lcd_string(char* input_string, unsigned char len);
void print_loop_number(void);
void display_board_number(void);
void display_int(char* description, unsigned int int_to_display);
void init_lcd_uart(void);




