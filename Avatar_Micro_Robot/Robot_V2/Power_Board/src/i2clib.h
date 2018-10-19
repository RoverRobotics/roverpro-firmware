/** Dan's glorious I2C library.
 * This provides a wrapper around PIC24's I2C primitives
 */

#ifndef I2CLIB_H
#define I2CLIB_H

#include <stdint.h>
#include <stdbool.h>

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

/** The result of an I2C operation. */
typedef enum I2CResult {
    I2C_NOTYET,  ///< Bus is still busy with the last operation. Try again in a bit.
    I2C_OKAY,    ///< The operation completed successfully
    I2C_ILLEGAL, ///< Incorrect use of the I2C module, probably by calling functions in the wrong
                 ///< order. This should be fixed in code.

    // The following are runtime errors which may be encountered during an elementary I2C operation.
    // Any of them should be followed by a STOP condition:
    I2C_ERROR_BUS_COLLISION, ///< Operation was not completed because another device was sending
                             ///< data on the bus
    I2C_ERROR_NACK_RESTART,  ///< Slave NACKED when we expected to restart. It means the device did
                             ///< not recognize our command byte.
    I2C_ERROR_NACK_WRITE,    ///< Operation was not completed because the device NACKED when we
                          ///< expected to send data. This means either the device is not present or
                          ///< we sent it more data than it was expecting.
    I2C_ERROR_NACK_READ, ///< Device NACKED when were expecting it to send data.

} I2CResult;

/// Reference to the I2C bus definition. The caller should treat this as an opaque type
typedef const struct I2CBusDefinition *I2CBus;

void i2c_enable(I2CBus bus);
void i2c_disable(I2CBus bus);
I2CResult i2c_start(I2CBus bus);
I2CResult i2c_stop(I2CBus bus);
I2CResult i2c_restart(I2CBus bus);
I2CResult i2c_request_byte(I2CBus bus);
I2CResult i2c_address(I2CBus bus, uint8_t addr, I2CReadWrite r);
I2CResult i2c_transmit_byte(I2CBus bus, uint8_t data);
I2CResult i2c_check_ack(I2CBus bus, I2CAck *ack);

/** These are the I2C buses available on our hardware (PIC24). */
#ifdef __PIC24FJ256GB106__

extern const I2CBus I2C_BUS1;
extern const I2CBus I2C_BUS2;
extern const I2CBus I2C_BUS3;

#endif

/** Represents an asynchronous operation. */
typedef struct I2COperationDef {
    uint8_t address;
    bool no_command_byte;
    uint8_t command_byte;
    bool write_starts_with_len;
    uint8_t size_writebuf;
    uint8_t *writebuf;
    bool read_starts_with_len;
    uint8_t size_readbuf;
    uint8_t *readbuf;
} I2COperationDef;

I2COperationDef i2c_op_read_byte(uint8_t address, uint8_t command_byte, uint8_t *byte_to_read);
I2COperationDef i2c_op_read_word(uint8_t address, uint8_t command_byte, uint16_t *word_to_read);
I2COperationDef i2c_op_read_block(uint8_t address, uint8_t command_byte, uint8_t *block_to_read,
                                  uint8_t maxlen);

I2COperationDef i2c_op_write_byte(uint8_t address, uint8_t command_byte,
                                  const uint8_t *byte_to_write);
I2COperationDef i2c_op_write_word(uint8_t address, uint8_t command_byte,
                                  const uint16_t *word_to_write);
I2COperationDef i2c_op_write_block(uint8_t address, uint8_t command_byte, uint8_t *block_to_write,
                                   uint8_t maxlen);
/** A logical step of the I2C protocol. */
typedef enum I2CResumeAt {
    I2C_STEP_UNSTARTED = 0,
    I2C_STEP_START,
    I2C_STEP_ADDRESS_W,
    I2C_STEP_COMMAND,
    I2C_STEP_TRANSMIT_LENGTH,
    I2C_STEP_TRANSMIT,
    I2C_STEP_RESTART,
    I2C_STEP_ADDRESS_R,
    I2C_STEP_REQUEST_LENGTH,
    I2C_STEP_RECEIVE_LENGTH,
    I2C_STEP_SEND_ACK,
    I2C_STEP_REQUEST,
    I2C_STEP_RECEIVE,
    I2C_STEP_SEND_NACK,
    I2C_STEP_STOP,
} I2CResumeAt;

/// Represents the progress of the I2C operation, not including the actual physical data already
/// read
typedef struct I2CProgress {
    I2CResumeAt resume_at;   ///< The next logical step of the I2C process
    uint8_t nbytes_written;  ///< number of bytes we have already written. If the I2C operation ends
                             ///< early, this may be less than nbytes_write_len
    uint8_t nbytes_read_len; ///< number of bytes we intend to read. In the case of a block read,
                             ///< this does not include the length byte
    uint8_t nbytes_read;     ///< number of bytes we have already read.
    I2CResult result_after_stop; ///< The result that we will eventually return. If we terminate the
                                 ///< I2C operation early from an unexpected NACK or bus collision,
                                 ///< this will hold that value while we issue a STOP condition.
} I2CProgress;

static const I2CProgress I2C_PROGRESS_UNSTARTED = {.resume_at = I2C_STEP_UNSTARTED};

/// Asynchronously do the given operation. This function will continuously return I2C_NOTYET until
/// it's finished, then it will return either I2C_OKAY or I2C_ERROR
I2CResult i2c_tick(I2CBus bus, const I2COperationDef *op, I2CProgress *progress);

/// Synchronously force the operation to completion
I2CResult i2c_synchronously_await(I2CBus bus, I2COperationDef op);

#endif
