void SPI_Init(void);

void OSD_loop(void);

void OSD_display_string(const ROM char *message, unsigned char col, unsigned char row, unsigned char length);

void display_voltage(void);

void disable_video(unsigned char ms);

void MAX_Internal_Synch(void);
void MAX_External_Synch(void);

void write_logo(void);
void display_logo(void);
void clear_screen(void);
void ghetto_clear_screen(void);
void rewrite_space(void);
#define VM0_write 0x00
#define VM0_read 0x80
#define VM1_write 0x00
#define DMM_write 0x04

#define VM0 0x00
#define DMAH 0x05
#define DMAL 0x06
#define DMM 0x04
#define DMDI 0x07

#define CMM 0x08
#define CMAH 0x09
#define CMAL 0x0A
#define CMDI 0x0B


//for software spi routine
#define SPI_CS PORTBbits.RB14
//#define SPI_MOSI PORTFbits.RF4
#define SPI_MOSI PORTBbits.RB15
//#define SPI_MOSI LATFbits.LATF4
#define SPI_CK PORTFbits.RF5

#define SPI_MISO PORTDbits.RD5
//#define software_read_wait; block_ms(1);
#define software_read_wait Nop();



//variables used in interrupt
/*extern unsigned char OSD_message[20];
extern unsigned char OSD_DMAH;
extern unsigned char OSD_DMAL;
extern unsigned char OSD_message_length;
extern unsigned char OSD_byte_counter;*/


#define CS_LAT LATDbits.LATD1




//unsigned char* logo_reg[2916];

//logo_reg[0] = 0x55;
//logo_reg[1] = 0x55;

