#include "xc.h"
#include "config.h"
#include "bootloader.h"


#if MAX_PROG_SIZE % _FLASH_ROW != 0
#error "MAX_PROG_SIZE must be a multiple of _FLASH_ROW"
#endif

/* bootloader starting address (cannot write to addresses between
 * BOOTLOADER_START_ADDRESS and APPLICATION_START_ADDRESS) */
extern unsigned _CODE_BASE;
extern unsigned _CODE_LENGTH;

#define BOOTLOADER_START_ADDRESS (_CODE_BASE)
#define BOOTLOADER_END_ADDRESS (_CODE_BASE + _CODE_LENGTH)
#define UNCHANGED_WORD (0xFFFFFF)

static uint8_t message[RX_BUF_LEN] = {0};
static uint8_t f16_sum1 = 0, f16_sum2 = 0;


inline bool address_within_bootloader(uint32_t address){
    return BOOTLOADER_START_ADDRESS <= address && address < BOOTLOADER_END_ADDRESS;
}
inline bool address_within_reset_vector(uint32_t address){
    return __RESET_BASE <= address && address < __RESET_BASE + __RESET_LENGTH/2;
}

inline bool address_is_row_aligned(uint32_t address){ return address % (_FLASH_ROW * 2) == 0; }

inline bool address_is_page_aligned(uint32_t address){ return address % (_FLASH_PAGE * 2) == 0; }

int main(void) {
    pre_bootload_hook();

    while (1){
        ClrWdt();
        receiveBytes();
        bootload_loop_hook();
    }
}

typedef enum ParseState {
	STATE_WAIT_FOR_START, ///< Have not yet received a start byte.
	STATE_READ_ESCAPED,   ///< Have received an escape byte, next data byte must be decoded
	STATE_READ_VERBATIM,  ///< Have not received an escape byte, next data byte should be read as-is
	STATE_END_OF_MESSAGE,
} ParseState;

void receiveBytes(void){
	static uint16_t messageIndex = 0;
	static ParseState state = STATE_WAIT_FOR_START;
	// keep reading until one of the following:
	// 1. we don't have any data available in the uart buffer to read in which case we return
	//    and wait for this function to be called again
	// 2. we find an END OF MESSAGE, in which case we call processCommand to deal with the data
	// 3. we overflow the receive buffer, in which case we throw out the data
    while(true)
	{
		uint8_t a_byte;

		if (!tryRxByte(&a_byte))
			return;

		switch (state){
		case STATE_WAIT_FOR_START:
			if (a_byte == START_OF_FRAME){
				state = STATE_READ_VERBATIM;
			}
			// otherwise, ignore data until we see a start byte
			break;
		case STATE_READ_VERBATIM:
			if (a_byte == ESC){
				state = STATE_READ_ESCAPED;
			} else if (a_byte == END_OF_FRAME) {
				state = STATE_END_OF_MESSAGE;
			} else {
				message[messageIndex++] = a_byte;
			}
			break;
		case STATE_READ_ESCAPED:
			message[messageIndex++] = a_byte ^ ESC_XOR;
			state = STATE_READ_VERBATIM;
			break;
		default:
			break;
		}

		if (state == STATE_END_OF_MESSAGE) {
			uint16_t fletcher = (uint16_t)message[messageIndex - 2]
	                + ((uint16_t)message[messageIndex - 1] << 8);
	        if(fletcher == fletcher16(message, messageIndex - 2)){
	            processCommand(message);
	            // we got a valid message! reset the stall timer
				TMR3HLD = 0;
				TMR2 = 0;
	        }
		}
	    if (messageIndex >= RX_BUF_LEN || state == STATE_END_OF_MESSAGE) {
			uint16_t i;
		    for(i=0; i<RX_BUF_LEN; i++) message[i] = 0;
		    messageIndex = 0;
		    state = STATE_WAIT_FOR_START;
		    return;
		}
	}
}

/// Construct a uint32 from the little endian bytes starting at data
uint32_t from_lendian_uint32(const void * data){
	const unsigned char * bytes = data;
	return ((uint32_t)bytes[0] << 0)
         | ((uint32_t)bytes[1] << 8)
         | ((uint32_t)bytes[2] << 16)
         | ((uint32_t)bytes[3] << 24);
}

void processCommand(uint8_t* data){
    uint16_t i;

    /* length is the length of the data block only, not including the command */
    uint8_t cmd = data[2];
    uint32_t address;
    uint16_t word;
    uint32_t longWord;
    uint32_t progData[MAX_PROG_SIZE + 1] = {0};

    char strVersion[16] = VERSION_STRING;
    char strPlatform[20] = PLATFORM_STRING;

    switch(cmd){
        case CMD_READ_PLATFORM:
            txString(cmd, strPlatform);
            break;

        case CMD_READ_VERSION:
            txString(cmd, strVersion);
            break;

        case CMD_READ_ROW_LEN:
            word = _FLASH_ROW;
            txArray16bit(cmd, &word, 1);
            break;

        case CMD_READ_PAGE_LEN:
            word = _FLASH_PAGE;
            txArray16bit(cmd, &word, 1);
            break;

        case CMD_READ_PROG_LEN:
            longWord = __PROGRAM_LENGTH;
            txArray32bit(cmd, &longWord, 1);
            break;

        case CMD_READ_MAX_PROG_SIZE:
            word = MAX_PROG_SIZE;
            txArray16bit(cmd, &word, 1);
            break;

        case CMD_READ_APP_START_ADDR:
            word = APPLICATION_START_ADDRESS;
            txArray16bit(cmd, &word, 1);
            break;

        case CMD_READ_BOOT_START_ADDR:
            word = BOOTLOADER_START_ADDRESS;
            txArray16bit(cmd, &word, 1);
            break;

        case CMD_ERASE_PAGE:
            address = from_lendian_uint32(data + 3);

            if (!address_is_page_aligned(address))
                break;

            /* do not allow the bootloader to be erased */
            if(address_within_bootloader(address))
                break;

            if (address == __RESET_BASE) {
                read_words(progData, address, __RESET_LENGTH/2);
                erase_page(address);
                write_words(progData, address, __RESET_LENGTH/2);
            } else {
                erase_page(address);
            }

            break;

        case CMD_READ_ADDR:
            address = from_lendian_uint32(data + 3);
            progData[0] = address;
            read_words(progData+1, address, 1);

            txArray32bit(cmd, progData, 2);
            break;

        case CMD_READ_MAX:
            address = from_lendian_uint32(data + 3);

            progData[0] = address;
            read_words(progData+1, address, MAX_PROG_SIZE);

            txArray32bit(cmd, progData, MAX_PROG_SIZE + 1);

            break;

        case CMD_WRITE_ROW:
            address = from_lendian_uint32(data + 3);

            if (!address_is_row_aligned(address))
                break;

            for(i=0; i<_FLASH_ROW; i++){
                /* do not allow the bootloader or reset vector to be written here */
                if (address_within_bootloader(address + 2 * i)||
                    address_within_reset_vector(address + 2 * i)) {
                    progData[i] = UNCHANGED_WORD;
                } else {
                    progData[i] = from_lendian_uint32(data + 7 + i * 4);
                }
            }

            write_words(progData, address, _FLASH_ROW);
            break;

        case CMD_WRITE_MAX_PROG_SIZE:
            address = from_lendian_uint32(data + 3);

            if (!address_is_row_aligned(address))
                break;

            for(i=0; i<MAX_PROG_SIZE; i++){
                /* do not allow the bootloader or reset vector to be written here */
                if (address_within_bootloader(address + 2 * i)||
                    address_within_reset_vector(address + 2 * i)) {
                    progData[i] = UNCHANGED_WORD;
                } else {
                    progData[i] = from_lendian_uint32(data + 7 + i * 4);
                }
            }

            write_words(progData, address, MAX_PROG_SIZE);
            break;

        case CMD_START_APP:
            try_start_app_hook();
            break;

        default:
        	break;
    }
}

void txStart(void){
    f16_sum1 = f16_sum2 = 0;

    while(U1STAbits.UTXBF); /* wait for tx buffer to empty */
    U1TXREG = START_OF_FRAME;
}

void txByte(uint8_t byte){
    if((byte == START_OF_FRAME) || (byte == END_OF_FRAME) || (byte == ESC)){
        while(U1STAbits.UTXBF); /* wait for tx buffer to empty */
        U1TXREG = ESC;          /* send escape character */

        while(U1STAbits.UTXBF); /* wait */
        U1TXREG = ESC_XOR ^ byte;
    }else{
        while(U1STAbits.UTXBF); /* wait */
        U1TXREG = byte;
    }

    fletcher16Accum(byte);
}

void txEnd(void){
    /* append checksum */
    uint8_t sum1 = f16_sum1;
    uint8_t sum2 = f16_sum2;

    txByte(sum1);
    txByte(sum2);

    while(U1STAbits.UTXBF); /* wait for tx buffer to empty */
    U1TXREG = END_OF_FRAME;
}

void txBytes(uint8_t cmd, uint8_t* bytes, uint16_t len){
    uint16_t i;

    txStart();
    txByte((uint8_t)(len & 0x00ff));
    txByte((uint8_t)((len & 0xff00) >> 8));
    txByte(cmd);

    for(i=0; i<len; i++){
        txByte(bytes[i]);
    }

    txEnd();
}

void txArray16bit(uint8_t cmd, uint16_t* words, uint16_t len){
    uint16_t length = len << 1;
    txBytes(cmd, (uint8_t*) words, length);
}

void txArray32bit(uint8_t cmd, uint32_t* words, uint16_t len){
    uint16_t length = len << 2;
    txBytes(cmd, (uint8_t*) words, length);
}

void txString(uint8_t cmd, char* str){
    uint16_t i, length = 0;

    /* find the length of the version string */
    while(str[length] != 0)  length++;
    length++;       /* be sure to get the string terminator */

    txStart();

    /* begin transmitting */
    txByte((uint8_t)(length & 0xff));
    txByte((uint8_t)((length & 0xff00) >> 8));

    txByte(cmd);

    for(i=0; i<length; i++){
        txByte((uint8_t)str[i]);
    }

    txEnd();
}

uint16_t fletcher16Accum(uint8_t byte){
    f16_sum1 = (f16_sum1 + (uint16_t)byte) & 0xff;
    f16_sum2 = (f16_sum2 + f16_sum1) & 0xff;
    return (f16_sum2 << 8) | f16_sum1;
}

uint16_t fletcher16(uint8_t* data, uint16_t length){
	uint16_t sum1 = 0, sum2 = 0, checksum;

    uint16_t i = 0;
    while(i < length){
        sum1 = (sum1 + (uint16_t)data[i]) & 0xff;
        sum2 = (sum2 + sum1) & 0xff;
        i++;
    }

    checksum = (sum2 << 8) | sum1;

	return checksum;
}
