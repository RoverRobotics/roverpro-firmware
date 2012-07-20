void select_video_channel(unsigned char channel);
void OSD_display_string(char *message, unsigned char col, unsigned char row, unsigned char length);
void video_test(void);
void SPI_Init(void);
void OSD_clear_screen(void);
void OSD_display_chars(unsigned char* message, unsigned char col, unsigned char row, unsigned char length);
unsigned char return_AD_value(unsigned char channel);
void display_voltage(void);
#define VM0 0x00
#define DMAH 0x05
#define DMAL 0x06
#define DMM 0x04

#define NORMAL_OSD 0x00
#define CLEAR_SCREEN 0x01
#define ENABLE_DISPLAY 0x02

