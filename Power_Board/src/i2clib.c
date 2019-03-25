#include "i2clib.h"
#include "stdhdr.h"
#include "stdlib.h"

/** I2C ack bit. Transmitted in response to any data received. */
typedef enum I2CAck {
    ACK = 0,  ///< ACKnowledge: the byte was successfully received and another byte may be sent
    NACK = 1, ///< Not ACKnowledge: either there is no receiver able to respond, the last command
    ///< wasn't understood, the receiver can't process any more data. Upon receiving a NACK, the
    ///< right thing to do is to stop.
} I2CAck;

/** I2C read/write bit. This always accompanies the address of a device and indicates whether we are
 * writing to or reading from the device*/
typedef enum I2CReadWrite {
    I2C_WRITE = 0, ///< The slave address is being opened in WRITE mode. We will transmit data.
    I2C_READ = 1,  ///< The slave address is being opened in READ mode. We will receive data.
} I2CReadWrite;

// Helper functions. Low-level I2C functions with platform-specific implementations
I2CResult i2c_start(I2CBus bus);
I2CResult i2c_stop(I2CBus bus);
I2CResult i2c_restart(I2CBus bus);
I2CResult i2c_request_byte(I2CBus bus);
I2CResult i2c_address(I2CBus bus, uint8_t addr, I2CReadWrite r);
I2CResult i2c_transmit_byte(I2CBus bus, uint8_t data);

/** State of an I2C bus. */
typedef enum I2CState {
    I2C_STARTING,            ///< I2C is starting
    I2C_IDLE_ACK,            ///< I2C has issued a start condition and is idle.
    I2C_IDLE_NACK,           ///< A NACK was either sent or received, and I2C should stop
    I2C_TRANSMITTING,        ///< I2C is currently writing out data
    I2C_RESTARTING,          ///< I2C is issuing a start for a repeated start command
    I2C_RECEIVING,           ///< I2C is receiving
    I2C_RECEIVE_BUFFER_FULL, ///< I2C has received a byte
    I2C_ACKING,              ///< ACK enabled - I2C is transmitting an ack bit
    I2C_STOPPING,            ///< I2C is stopping
    I2C_BUS_COLLISION,       ///< Attempted an action while the bus was busy. We should disable and
                             ///< re-enable the I2C module and issue a start.
    I2C_DISABLED,            ///< The I2C module is not running
} I2CState;

#ifdef __PIC24F__
#include <p24Fxxxx.h>

/** I2C Control Register, based on the PIC24 MCU documentation */
typedef struct I2CControlBits {
    unsigned SEN : 1;   ///< Start in progress
    unsigned RSEN : 1;  ///< ReStart in progress
    unsigned PEN : 1;   ///< stoP in progress
    unsigned RCEN : 1;  ///< ReCeive in progress
    unsigned ACKEN : 1; ///< transmitting ACK or NACK
    unsigned ACKDT : 1; ///< whether the transmitting ACK is an ACK or NACK
    unsigned STREN : 1;
    unsigned GCEN : 1;
    unsigned SMEN : 1;
    unsigned DISSLW : 1;
    unsigned A10M : 1;
    unsigned IPMIEN : 1;
    unsigned SCLREL : 1;
    unsigned I2CSIDL : 1;
    unsigned : 1;
    unsigned I2CEN : 1; ///> whether the I2C module is enabled
} I2CControlBits;

/** I2C status register, based on the PIC24 MCU documentation */
typedef struct I2CStatusBits {
    unsigned TBF : 1; ///< Transmit Buffer Full
    unsigned RBF : 1; ///< Receive Buffer Full
    unsigned R_W : 1;
    unsigned S : 1;
    unsigned P : 1;
    unsigned D_A : 1;
    unsigned I2COV : 1;
    unsigned IWCOL : 1;
    unsigned ADD10 : 1;
    unsigned GCSTAT : 1;
    unsigned BCL : 1; ///< Buffer CoLlision. If true, something else is using the bus.
    unsigned : 3;
    unsigned TRSTAT : 1; ///< TRansmit STATus. If true, I2C module is still writing transmit data to
                         ///< buffer.
    unsigned ACKSTAT : 1; ///< ACKnowledge STATus. If NACK, a NACK has been either sent or received.
} I2CStatusBits;

/** I2C bus definition, based on the PIC24 MCU documentation
 * http://ww1.microchip.com/downloads/en/DeviceDoc/70000195f.pdf
 */
typedef struct I2CBusDefinition {
    volatile I2CControlBits *CON; ///< Memory location of Control Register
    volatile I2CStatusBits *STAT; ///< Memory location of Status Register
    volatile unsigned int *TRN;   ///< Memory location of Transmit Data Register
    volatile unsigned int *RCV;   ///< Memory location of Receive Data Register
    volatile unsigned int *BRG;   ///< Memory location of Baud Rate Generator Reload Register
} I2CBusDefinition;

I2CState i2c_state(I2CBus bus) {
    I2CState state;
    I2CControlBits con = *bus->CON;
    I2CStatusBits stat = *bus->STAT;

    if (con.SEN) {
        state = I2C_STARTING;
    } else if (con.RSEN) {
        state = I2C_RESTARTING;
    } else if (con.PEN) {
        state = I2C_STOPPING;
    } else if (con.RCEN) {
        state = I2C_RECEIVING;
    } else if (con.ACKEN) {
        state = I2C_ACKING;
    } else if (!con.I2CEN) {
        state = I2C_DISABLED;

    } else if (stat.BCL) {
        state = I2C_BUS_COLLISION;
    } else if (stat.TRSTAT) {
        state = I2C_TRANSMITTING;
    } else if (stat.RBF) {
        state = I2C_RECEIVE_BUFFER_FULL;
    } else if (stat.ACKSTAT == NACK) {
        state = I2C_IDLE_NACK;
    } else {
        state = I2C_IDLE_ACK;
    }

    return state;
}

void i2c_enable(I2CBus bus) {
    I2CControlBits blank_con = {0};
    *(bus->CON) = blank_con;
    *(bus->BRG) = 0xc0;
    bus->CON->I2CEN = 1;
}
void i2c_disable(I2CBus bus) { bus->CON->I2CEN = 0; }

I2CResult i2c_start(I2CBus bus) {
    I2CStatusBits stat;
    // Clear bus collision flag. If there was a collision in a previous operation, we don't care
    // anymore.
    bus->STAT->BCL = 0;

    stat = *bus->STAT;
    I2CState state = i2c_state(bus);

    switch (state) {
    default:
        return I2C_ILLEGAL;
    case I2C_STOPPING:
        return I2C_NOTYET;
    case I2C_IDLE_ACK: // ACKSTAT will still be set from the last I2C operation, so we *do* need
                       // both
    case I2C_IDLE_NACK:
        break;
    }

    bus->CON->SEN = 1;
    return I2C_OKAY;
}

I2CResult i2c_stop(I2CBus bus) {
    // Clear bus collision flag. If there was a collision in a previous operation, we don't care
    // anymore.
    bus->STAT->BCL = 0;

    I2CState state = i2c_state(bus);

    switch (state) {
    default:
        return I2C_ILLEGAL;
    case I2C_ACKING:
    case I2C_TRANSMITTING:
        return I2C_NOTYET;
    case I2C_IDLE_NACK:
    case I2C_IDLE_ACK:
        break;
    }
    bus->CON->PEN = 1;
    return I2C_OKAY;
}

I2CResult i2c_restart(I2CBus bus) {
    I2CState state = i2c_state(bus);
    switch (state) {
    default:
        return I2C_ILLEGAL;

    case I2C_BUS_COLLISION:
        return I2C_ERROR_BUS_COLLISION;
    case I2C_TRANSMITTING:
        return I2C_NOTYET;
    case I2C_IDLE_NACK:
        return I2C_ERROR_NACK_RESTART;
    case I2C_IDLE_ACK:
        break;
    }
    bus->CON->RSEN = 1;
    return I2C_OKAY;
}

I2CResult i2c_request_byte(I2CBus bus) {
    I2CState state = i2c_state(bus);
    switch (state) {
    default:
        return I2C_ILLEGAL;
    case I2C_BUS_COLLISION:
        return I2C_ERROR_BUS_COLLISION;
    case I2C_ACKING:       // still acknowledging the previous byte
    case I2C_TRANSMITTING: // still sending data
        return I2C_NOTYET;
    case I2C_IDLE_NACK:
        return I2C_ERROR_NACK_READ;
    case I2C_IDLE_ACK:
        break;
    }
    bus->CON->RCEN = 1;
    return I2C_OKAY;
}

I2CResult i2c_address(I2CBus bus, uint8_t addr, I2CReadWrite rw) {
    I2CState state = i2c_state(bus);

    switch (state) {
    default:
        BREAKPOINT();
        return I2C_ILLEGAL;
    case I2C_BUS_COLLISION:
        return I2C_ERROR_BUS_COLLISION;
    case I2C_STARTING:   // Writing address initially
    case I2C_RESTARTING: // Writing address after a repeated start condition
        return I2C_NOTYET;
    case I2C_IDLE_ACK:
    case I2C_IDLE_NACK:
        break;
    }
    BREAKPOINT_IF(!bus->STAT->S);
    *(bus->TRN) = (addr << 1) + rw;
    if (bus->STAT->IWCOL) {
        bus->STAT->IWCOL = 0;
        return I2C_ILLEGAL;
    }
    return I2C_OKAY;
}

I2CResult i2c_transmit_byte(I2CBus bus, uint8_t data) {
    I2CState state = i2c_state(bus);
    switch (state) {
    default:
        return I2C_ILLEGAL;
    case I2C_BUS_COLLISION:
        return I2C_ERROR_BUS_COLLISION;
    case I2C_IDLE_NACK:
        return I2C_ERROR_NACK_WRITE;
    case I2C_TRANSMITTING:
        return I2C_NOTYET;
    case I2C_IDLE_ACK:
        break;
    }
    *(bus->TRN) = data;
    if (bus->STAT->IWCOL) {
        bus->STAT->IWCOL = 0;
        return I2C_NOTYET;
    }
    return I2C_OKAY;
}

I2CResult i2c_receive_byte(I2CBus bus, uint8_t *data) {
    I2CState state = i2c_state(bus);
    switch (state) {
    default:
        return I2C_ILLEGAL;
    case I2C_RECEIVING:
        return I2C_NOTYET;
    case I2C_RECEIVE_BUFFER_FULL:
        break;
    }

    if (!(bus->STAT->RBF))
        return I2C_NOTYET;

    *data = (uint8_t)(*bus->RCV);
    return I2C_OKAY;
}
I2CResult i2c_ack(I2CBus bus, I2CAck ack) {
    I2CState state = i2c_state(bus);
    switch (state) {
    default:
        return I2C_ILLEGAL;
    case I2C_IDLE_ACK:
        break;
    }

    bus->CON->ACKDT = ack;
    bus->CON->ACKEN = 1;
    return I2C_OKAY;
}

static const I2CBusDefinition I2C_BUS1_DEF = {
    (I2CControlBits *)&I2C1CON, (I2CStatusBits *)&I2C1STAT, &I2C1TRN, &I2C1RCV, &I2C1BRG};

static const I2CBusDefinition I2C_BUS2_DEF = {
    (I2CControlBits *)&I2C2CON, (I2CStatusBits *)&I2C2STAT, &I2C2TRN, &I2C2RCV, &I2C2BRG};

static const I2CBusDefinition I2C_BUS3_DEF = {
    (I2CControlBits *)&I2C3CON, (I2CStatusBits *)&I2C3STAT, &I2C3TRN, &I2C3RCV, &I2C3BRG};

const I2CBus I2C_BUS1 = &I2C_BUS1_DEF;
const I2CBus I2C_BUS2 = &I2C_BUS2_DEF;
const I2CBus I2C_BUS3 = &I2C_BUS3_DEF;
#endif

// Begin device-independent I2C Stuff.

I2CResult i2c_tick(I2CBus bus, const I2COperationDef *op, I2CProgress *progress) {
    I2CResult result = I2C_ILLEGAL;
    while (true) {
        switch (progress->resume_at) {

        case I2C_STEP_UNSTARTED:
            progress->nbytes_written = 0;
            progress->nbytes_read = 0;
            progress->nbytes_read_len = 0;
            progress->result_after_stop = I2C_NOTYET;
            progress->resume_at = I2C_STEP_START;
            // fallthrough to start

        case I2C_STEP_START:
            result = i2c_start(bus);
            if (result == I2C_OKAY) {
                if (!op->no_command_byte) {
                    progress->resume_at = I2C_STEP_ADDRESS_W;
                } else if (op->size_writebuf > 0) {
                    progress->resume_at = I2C_STEP_ADDRESS_W;
                } else if (op->size_readbuf > 0) {
                    progress->resume_at = I2C_STEP_ADDRESS_R;
                } else {
                    progress->resume_at = I2C_STEP_STOP;
                }
            }
            break;

        case I2C_STEP_ADDRESS_W:
            result = i2c_address(bus, op->address, I2C_WRITE);
            if (result == I2C_OKAY) {
                if (!op->no_command_byte) {
                    progress->resume_at = I2C_STEP_COMMAND;
                } else if (op->size_writebuf > 0) {
                    progress->resume_at = I2C_STEP_TRANSMIT;
                } else if (op->size_readbuf > 0) {
                    progress->resume_at = I2C_STEP_RESTART;
                } else {
                    progress->resume_at = I2C_STEP_STOP;
                }
            }
            break;

        case I2C_STEP_COMMAND:
            result = i2c_transmit_byte(bus, op->command_byte);
            if (result == I2C_OKAY) {
                if (op->write_starts_with_len) {
                    progress->resume_at = I2C_STEP_TRANSMIT_LENGTH;
                } else if (op->size_writebuf > 0) {
                    progress->resume_at = I2C_STEP_TRANSMIT;
                } else if (op->size_readbuf > 0) {
                    progress->resume_at = I2C_STEP_RESTART;
                } else {
                    progress->resume_at = I2C_STEP_STOP;
                }
            }
            break;

        case I2C_STEP_TRANSMIT_LENGTH:
            result = i2c_transmit_byte(bus, op->writebuf[op->size_writebuf]);
            if (result == I2C_OKAY) {
                if (op->size_writebuf > 0) {
                    progress->resume_at = I2C_STEP_TRANSMIT;
                } else if (op->size_readbuf > 0) {
                    progress->resume_at = I2C_STEP_RESTART;
                } else {
                    progress->resume_at = I2C_STEP_STOP;
                }
            }
            break;

        case I2C_STEP_TRANSMIT:
            result = i2c_transmit_byte(bus, op->writebuf[progress->nbytes_written]);
            if (result == I2C_OKAY) {
                progress->nbytes_written++;
                BREAKPOINT_IF(progress->nbytes_written > op->size_writebuf);
                if (progress->nbytes_written < op->size_writebuf) {
                    progress->resume_at = I2C_STEP_TRANSMIT;
                } else if (op->size_readbuf > 0) {
                    progress->resume_at = I2C_STEP_RESTART;
                } else {
                    progress->resume_at = I2C_STEP_STOP;
                }
            }
            break;

        case I2C_STEP_RESTART:
            result = i2c_restart(bus);
            if (result == I2C_OKAY) {
                progress->resume_at = I2C_STEP_ADDRESS_R;
            }
            break;

        case I2C_STEP_ADDRESS_R:
            result = i2c_address(bus, op->address, I2C_READ);
            if (result == I2C_OKAY) {
                if (op->read_starts_with_len) {
                    progress->resume_at = I2C_STEP_REQUEST_LENGTH;
                } else if (op->size_readbuf > 0) {
                    progress->nbytes_read_len = op->size_readbuf;
                    progress->resume_at = I2C_STEP_REQUEST;
                } else {
                    progress->resume_at = I2C_STEP_STOP;
                }
            }
            break;

        case I2C_STEP_REQUEST_LENGTH:
            result = i2c_request_byte(bus);
            if (result == I2C_OKAY) {
                progress->resume_at = I2C_STEP_RECEIVE_LENGTH;
            }
            break;

        case I2C_STEP_RECEIVE_LENGTH:
            result = i2c_receive_byte(bus, &(progress->nbytes_read_len));
            if (result == I2C_OKAY) {
                // Set the number of bytes to read based on that length or the read buffer
                // size, whichever is smaller
                progress->nbytes_read_len = min(op->size_readbuf, progress->nbytes_read_len);

                if (progress->nbytes_read_len > 1) {
                    progress->resume_at = I2C_STEP_SEND_ACK;
                } else {
                    progress->resume_at = I2C_STEP_SEND_NACK;
                }
            }
            break;

        case I2C_STEP_SEND_ACK:
            result = i2c_ack(bus, ACK);
            if (result == I2C_OKAY) {
                progress->resume_at = I2C_STEP_REQUEST;
            }
            break;

        case I2C_STEP_REQUEST:
            result = i2c_request_byte(bus);
            if (result == I2C_OKAY) {
                progress->resume_at = I2C_STEP_RECEIVE;
            }
            break;

        case I2C_STEP_RECEIVE:
            result = i2c_receive_byte(bus, op->readbuf + progress->nbytes_read);
            if (result == I2C_OKAY) {
                progress->nbytes_read++;
                BREAKPOINT_IF(progress->nbytes_read > progress->nbytes_read_len);
                if (progress->nbytes_read < progress->nbytes_read_len) {
                    progress->resume_at = I2C_STEP_SEND_ACK;
                } else {
                    progress->resume_at = I2C_STEP_SEND_NACK;
                }
            }
            break;

        case I2C_STEP_SEND_NACK:
            result = i2c_ack(bus, NACK); // NACK after reading all data bytes
            if (result == I2C_OKAY) {
                progress->resume_at = I2C_STEP_STOP;
            }
            break;

        case I2C_STEP_STOP:
            result = i2c_stop(bus);
            if (result == I2C_OKAY) {
                if (progress->result_after_stop == I2C_NOTYET) {
                    return I2C_OKAY;
                } else {
                    return progress->result_after_stop;
                }
            }
            break;
        }

        BREAKPOINT_IF(result == I2C_ILLEGAL);
        if (result == I2C_NOTYET) {
            // yield control back to the calling program
            return I2C_NOTYET;
        } else if (result == I2C_OKAY) {
            // return to the top and execute the next step
            continue;
        } else {
            progress->result_after_stop = result;
            progress->resume_at = I2C_STEP_STOP;
            // go to case I2C_STEP_STOP
            continue;
        }
    }
}

I2CResult i2c_synchronously_await(I2CBus bus, I2COperationDef op) {
    I2CProgress progress = I2C_PROGRESS_UNSTARTED;
    I2CResult result;
    do {
        result = i2c_tick(bus, &op, &progress);
    } while (result == I2C_NOTYET);
    return result;
}

I2COperationDef i2c_op_read_byte(uint8_t address, uint8_t command_byte, uint8_t *byte_to_read) {
    return (I2COperationDef){
        .address = address,
        .command_byte = command_byte,
        .size_readbuf = sizeof(*byte_to_read),
        .readbuf = byte_to_read,
    };
}

I2COperationDef i2c_op_read_word(uint8_t address, uint8_t command_byte, uint16_t *word_to_read) {
    return (I2COperationDef){
        .address = address,
        .command_byte = command_byte,
        .size_readbuf = sizeof(*word_to_read),
        .readbuf = (uint8_t *)word_to_read,
    };
}

I2COperationDef i2c_op_read_block(uint8_t address, uint8_t command_byte, void *block_to_read,
                                  uint8_t maxlen) {
    return (I2COperationDef){
        .address = address,
        .command_byte = command_byte,
        .size_readbuf = maxlen,
        .readbuf = block_to_read,
        .read_starts_with_len = true,
    };
}

I2COperationDef i2c_op_write_byte(uint8_t address, uint8_t command_byte,
                                  const uint8_t *byte_to_write) {
    return (I2COperationDef){
        .address = address,
        .command_byte = command_byte,
        .size_writebuf = sizeof(*byte_to_write),
        .writebuf = (uint8_t *)byte_to_write,
    };
}

I2COperationDef i2c_op_write_word(uint8_t address, uint8_t command_byte,
                                  const uint16_t *word_to_write) {
    return (I2COperationDef){
        .address = address,
        .command_byte = command_byte,
        .size_writebuf = sizeof(*word_to_write),
        .writebuf = (uint8_t *)word_to_write,
    };
}

I2COperationDef i2c_op_write_block(uint8_t address, uint8_t command_byte, void *block_to_write,
                                   uint8_t maxlen) {
    return (I2COperationDef){
        .address = address,
        .command_byte = command_byte,
        .size_writebuf = maxlen,
        .writebuf = block_to_write,
        .write_starts_with_len = true,
    };
}
