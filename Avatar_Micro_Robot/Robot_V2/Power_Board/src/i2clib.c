#include "i2clib.h"

#ifdef __PIC24FJ256GB106__
// begin PIC24 MCU
/** State of an I2C bus. */
typedef enum i2c_state_t {
  I2C_STARTED = 0, ///< I2C has issued a start condition and is idle.
  I2C_STOPPED, ///< I2C is idle and has either not yet issued a start condition or has issued a stop condition
  I2C_TRANSMITTING, ///< I2C is currently writing out data
  I2C_SEN, ///< I2C is Starting
  I2C_RSEN, ///< I2C is issuing a Start for a repeated start command
  I2C_PEN, ///< I2C is stopping
  I2C_RCEN, ///< I2C is receiving
  I2C_RBF, ///< I2C has received a byte
  I2C_ACKEN, ///< I2C is transmitting an ack bit
  I2C_NACKED, ///< A NACK was either sent or received, and I2C should stop
  I2C_DISABLED ///< The I2C module is not running
} i2c_state_t;

/** I2C Control Register, based on the PIC24 MCU documentation */
typedef struct i2c_con_t {
  unsigned SEN:1;
  unsigned RSEN:1;
  unsigned PEN:1;
  unsigned RCEN:1;
  unsigned ACKEN:1;
  unsigned ACKDT:1;
  unsigned STREN:1;
  unsigned GCEN:1;
  unsigned SMEN:1;
  unsigned DISSLW:1;
  unsigned A10M:1;
  unsigned IPMIEN:1;
  unsigned SCLREL:1;
  unsigned I2CSIDL:1;
  unsigned :1;
  unsigned I2CEN:1;
} i2c_con_t;

/** I2C status register, based on the PIC24 MCU documentation */
typedef struct i2c_stat_t {
  unsigned TBF:1;
  unsigned RBF:1;
  unsigned R_W:1;
  unsigned S:1;
  unsigned P:1;
  unsigned D_A:1;
  unsigned I2COV:1;
  unsigned IWCOL:1;
  unsigned ADD10:1;
  unsigned GCSTAT:1;
  unsigned BCL:1;
  unsigned :3;
  unsigned TRSTAT:1;
  unsigned ACKSTAT:1;
} i2c_stat_t;

/** I2C bus definition, based on the PIC24 MCU documentation
 * http://ww1.microchip.com/downloads/en/DeviceDoc/70000195f.pdf
*/
typedef struct i2c_busdef_t {
  volatile i2c_con_t *CON;    ///< Memory location of Control Register
  volatile i2c_stat_t *STAT;    ///< Memory location of Status Register
  volatile unsigned int *TRN; ///< Memory location of Transmit Data Register
  volatile unsigned int *RCV; ///< Memory location of Receive Data Register
  volatile unsigned int *RCV; ///< Memory location of Baud Rate Generator Reload Register
} i2c_busdef_t;


i2c_state_t i2c_state(i2c_bus_t bus) {
    i2c_state_t state = (
        bus->CON->SEN ? I2C_SEN
                      : bus->CON->RSEN ? I2C_RSEN
                                       : bus->CON->PEN ? I2C_PEN
                                                       : bus->CON->RCEN ? I2C_RCEN
                                                       : bus->STAT->RBF ? I2C_RBF
                                                                        : bus->CON->ACKEN ? I2C_ACKEN
                                                                                          : bus->STAT->TRSTAT
                                                                                            ? I2C_TRANSMITTING
      : bus->STAT->ACKSTAT==NACK? I2C_NACKED
                                                                                            : bus->STAT->S ? I2C_STARTED
                                                                                                           : bus->CON->I2CEN
                                                                                                             ? I2C_STOPPED
                                                                                                             : I2C_DISABLED
    );
    return state;
}

i2c_result_t i2c_enable(i2c_bus_t bus) {
    bus->CON->I2CEN = 1;
    return I2C_OKAY;
}
i2c_result_t i2c_start(i2c_bus_t bus) {
    switch (i2c_state(bus)) {
    case I2C_STOPPED: break;
    case I2C_PEN: return I2C_NOTYET;
    default: return I2C_ILLEGAL;
    }
    bus->CON->SEN = 1;
    return I2C_OKAY;
}

i2c_result_t i2c_stop(i2c_bus_t bus) {
    i2c_state_t state = i2c_state(bus);
    switch (state) {
      case I2C_NACKED:
    case I2C_STARTED: break;
    case I2C_ACKEN:
    case I2C_TRANSMITTING: return I2C_NOTYET;
    default: return I2C_ILLEGAL;
    }
    bus->CON->PEN = 1;
    return I2C_OKAY;
}

i2c_result_t i2c_restart(i2c_bus_t bus) {
    switch (i2c_state(bus)) {
    case I2C_TRANSMITTING: return I2C_NOTYET;
    case I2C_STARTED: break;
    default: return I2C_ILLEGAL;
    }
    bus->CON->RSEN = 1;
    return I2C_OKAY;
}

i2c_result_t i2c_request_byte(i2c_bus_t bus) {
    switch (i2c_state(bus)) {
    case I2C_STARTED: break;
    case I2C_ACKEN: // still acknowledging the previous byte
    case I2C_TRANSMITTING: // still sending data
        return I2C_NOTYET;
    default: return I2C_ILLEGAL;
    }
    bus->CON->RCEN = 1;
    return I2C_OKAY;
}

i2c_result_t i2c_address(i2c_bus_t bus, unsigned char addr, i2c_readwrite_t rw) {
    switch (i2c_state(bus)) {
    case I2C_STARTED: break;
    case I2C_SEN: // Writing address initially
    case I2C_RSEN: // Writing address after a repeated start condition
        return I2C_NOTYET;
    default: return I2C_ILLEGAL;
    }
    *(bus->TRN) = (addr << 1) + rw;
    if (bus->STAT->IWCOL) {
        bus->STAT->IWCOL = 0;
        return I2C_NOTYET;
    }
    return I2C_OKAY;
}

i2c_result_t i2c_transmit_byte(i2c_bus_t bus, unsigned char data) {
    switch (i2c_state(bus)) {
    case I2C_STARTED: break;
    case I2C_TRANSMITTING: return I2C_NOTYET;
    default: return I2C_ILLEGAL;
    }
    *(bus->TRN) = data;
    if (bus->STAT->IWCOL) {
        bus->STAT->IWCOL = 0;
        return I2C_NOTYET;
    }
    return I2C_OKAY;
}

i2c_ack_t i2c_get_ack(i2c_bus_t bus){
    return bus->STAT->ACKSTAT ? NACK : ACK;
}

i2c_result_t i2c_check_ack(i2c_bus_t bus, i2c_ack_t *ack) {
    switch (i2c_state(bus)) {
    case I2C_STARTED: break;
    case I2C_TRANSMITTING: return I2C_NOTYET;
    default: return I2C_ILLEGAL;
    }
    *ack = bus->STAT->ACKSTAT;
    return I2C_OKAY;
}

i2c_result_t i2c_receive_and_ack(i2c_bus_t bus, i2c_ack_t acknack, unsigned char *data) {
    switch (i2c_state(bus)) {
    case I2C_RBF: break;
    case I2C_RCEN: return I2C_NOTYET;
    default: return I2C_ILLEGAL;
    }

    *data = *bus->RCV;
    bus->CON->ACKDT = acknack;
    bus->CON->ACKEN = 1;
    return I2C_OKAY;
}
i2c_result_t i2c_receive_byte(i2c_bus_t bus,  unsigned char *data) {
    switch (i2c_state(bus)) {
    case I2C_RBF: break;
    case I2C_RCEN: return I2C_NOTYET;
    default: return I2C_ILLEGAL;
    }

    if (!(bus->STAT->RBF))
        return I2C_NOTYET;

    *data = *bus->RCV;
    return I2C_OKAY;
}
i2c_result_t i2c_ack(i2c_bus_t bus, i2c_ack_t ack){
    switch (i2c_state(bus)) {
        case I2C_STARTED: break;
        default: return I2C_ILLEGAL;
    }

    bus->CON->ACKDT = ack;
    bus->CON->ACKEN = 1;
    return I2C_OKAY;
}


const i2c_busdef_t I2C1_DEF = {(i2c_con_t *) &I2C1CON, (i2c_stat_t *) &I2C1STAT, &I2C1TRN, &I2C1RCV, &I2C1BRG}
const i2c_busdef_t I2C2_DEF = {(i2c_con_t *) &I2C2CON, (i2c_stat_t *) &I2C2STAT, &I2C2TRN, &I2C2RCV, &I2C2BRG}
const i2c_busdef_t I2C3_DEF = {(i2c_con_t *) &I2C3CON, (i2c_stat_t *) &I2C3STAT, &I2C3TRN, &I2C3RCV, &I2C3BRG}

const i2c_bus_t I2C1 = &I2C1_DEF;
const i2c_bus_t I2C2 = &I2C2_DEF;
const i2c_bus_t I2C3 = &I2C3_DEF;
#endif

// Begin device-independent I2C Stuff.


enum i2c_buffer_size {
  I2C_BYTE = 1,
  I2C_WORD = 2,
  I2C_UINT32 = 4,
  I2C_UINT64 = 8,
  I2C_VARIADIC = -1
};

typedef struct i2c_progress_t {
  int resume_at;
  int nbytes_written;
  int nbytes_write_len;
  int nbytes_read;
  int nbytes_read_len;
} i2c_progress_t;

int min(int x, int y) {
    return x < y ? x : y;
}

i2c_result_t i2c_do(i2c_bus_t bus, i2c_op_t op, i2c_progress_t *progress) {
    i2c_result_t result;
    i2c_ack_t ack;

#define STEP(cmd): progress->resume_at=(__LINE__);\
                    case __LINE__:\
                        result = (cmd); \
                        if (result == I2C_OKAY) {/*continue on to next step*/}\
                        else if (result == I2C_NOTYET) {return result;}\
                        else (BREAKPOINT);

#define STEP_ACK(cmd): progress->resume_at=(__LINE__);\
                    case __LINE__:\
                        result = (cmd); \
                        BREAKPOINTIF(result != I2C_OKAY && result != I2C_NOTYET)\
                        if (i2c_get_ack(bus)== NACK){ goto CASE_I2C_STOP;}\
                        else if (result == I2C_OKAY) { /*continue on to next step*/}\
                        else if (result == I2C_NOTYET) {return result;}

    switch (progress->resume_at) { // todo: we have to abort early if NACK
    default:BREAKPOINT;
    case 0:
        STEP(i2c_start(bus))
        if (op->size_writebuf || !op->no_command_byte) {
            STEP_ACK(i2c_address(bus, op->address, I2C_WRITE))

            if (op->has_command_byte) {
                STEP_ACK(i2c_transmit_byte(bus, op->command_byte))
            }
            if (op->write_starts_with_len) {
                progress->
                    nbytes_write_len = min(op->size_writebuf, op->writebuf[0] + 1);
            } else {
                progress->
                    nbytes_write_len = op->size_writebuf;
            }
            for (progress->nbytes_written = 0;
                 progress->nbytes_written < progress->nbytes_write_len;
                 progress->nbytes_written++) {
                STEP_ACK(i2c_transmit_byte(bus, op->writebuf[progress->nbytes_written]))
            }
        }

        if ((op->size_writebuf || !op->no_command_byte) && op->size_readbuf > 0) {
            STEP(i2c_restart(bus))
        }

        if (op->size_readbuf > 0) {
            STEP_ACK(i2c_address(bus, op->address, I2C_READ))

            progress->nbytes_read = 0
            // read the first data byte
            STEP(i2c_request_byte(bus))
            STEP(i2c_receive_byte(bus, op->readbuf))
            // the first time through, if the first byte we read is a length
            if (op->read_starts_with_len) {
                // then set the number of bytes to read based on that length or the read buffer size, whichever is smaller
                // remember, the readbuf byte count doesn't include itself, hence the +1
                progress->nbytes_read_len = min(op->size_readbuf, op->readbuf[0] + 1);
            } else {
                progress->nbytes_read_len = op->size_readbuf;
            }

            for (progress->nbytes_read = 1; // we have already read one byte.
                 progress->nbytes_read < progress->nbytes_read_len;
                 progress->nbytes_read++) {
                STEP(i2c_ack(bus, ACK));    // ACK since there's another byte to read
                STEP(i2c_request_byte(bus)) // let recipient know we're ready for another byte
                STEP(i2c_receive_byte(bus, op->readbuf + progress->nbytes_read)) // get that byte into the read buffer
            }
            STEP(i2c_ack(bus, NACK)) // NACK after reading all data bytes
        }

    CASE_I2C_STOP:
        STEP(i2c_stop(bus))
#undef STEP
    }
}

void i2c_await(i2c_bus_t bus, i2c_op_t op) {
    i2c_progress_t progress = {0};
    while (i2c_do(bus, op, &progress) == I2C_NOTYET) {
        // just block execution until this is done...
    };
}
