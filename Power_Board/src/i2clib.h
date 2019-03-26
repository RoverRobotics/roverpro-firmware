/// @file
/// Dan's glorious I2C library.
/// A high-level wrapper around PIC24's I2C primitives.

#ifndef I2CLIB_H
#define I2CLIB_H

#include <stdint.h>
#include <stdbool.h>

/// The result of an I2C operation.
/// The I2C_ERROR_... values are all runtime errors which may be encountered during an elementary
/// I2C operation. Any of them should be followed by a STOP condition:
typedef enum I2CResult {
    I2C_NOTYET,  ///< Bus is still busy with the last operation. Try again in a bit.
    I2C_OKAY,    ///< The operation completed successfully
    I2C_ILLEGAL, ///< Incorrect use of the I2C module, probably by calling functions in the wrong
                 ///< order. This should be fixed in code.
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

/// An I2C bus peripheral available on our hardware (PIC24)
extern const I2CBus I2C_BUS1;
/// An I2C bus peripheral available on our hardware (PIC24)
extern const I2CBus I2C_BUS2;
/// An I2C bus peripheral available on our hardware (PIC24)
extern const I2CBus I2C_BUS3;

/// Activate the given I2C peripheral
void i2c_enable(I2CBus bus);
/// Deactivate the given I2C peripheral
void i2c_disable(I2CBus bus);

/// The definition of a particular I2C operation
typedef struct I2COperationDef {
    /// Target device address, excluding the RW flag (7 bits)
    uint8_t address;
    /// If true, the operation does not require a command byte
    bool no_command_byte;
    /// Ignored if `no_command_byte == true`. The command byte usually specifies which data element
    /// we will read/write
    uint8_t command_byte;
    /// If true, we prepend the write data with its length
    bool write_starts_with_len;
    /// Size of the data we intend to write, in bytes. e.g. 2 for an I2C "Write Word"
    uint8_t size_writebuf;
    /// Pointer to the data to write to the device. Should be at least as big as `size_writebuf`
    uint8_t *writebuf;
    /// If true, we interpret the first received byte as the byte length of the inbound data.
    bool read_starts_with_len;
    /// Size of the readbuf in bytes. e.g. 2 for an I2C "Read Word"
    uint8_t size_readbuf;
    /// Pointer to a buffer to hold inbound data from the device. Should be at least as big as
    /// `size_readbuf`
    uint8_t *readbuf;
} I2COperationDef;

/// Declare (but don't start) an I2C Read Byte operation
I2COperationDef i2c_op_read_byte(uint8_t address, uint8_t command_byte, uint8_t *byte_to_read);
/// Declare (but don't start) an I2C Read Word operation
I2COperationDef i2c_op_read_word(uint8_t address, uint8_t command_byte, uint16_t *word_to_read);
/// Declare (but don't start) an I2C Read Block operation
I2COperationDef i2c_op_read_block(uint8_t address, uint8_t command_byte, void *block_to_read,
                                  uint8_t maxlen);
/// Declare (but don't start) an I2C Write Byte operation
I2COperationDef i2c_op_write_byte(uint8_t address, uint8_t command_byte,
                                  const uint8_t *byte_to_write);
/// Declare (but don't start) an I2C Write Word operation
I2COperationDef i2c_op_write_word(uint8_t address, uint8_t command_byte,
                                  const uint16_t *word_to_write);
/// Declare (but don't start) an I2C Write Block operation
I2COperationDef i2c_op_write_block(uint8_t address, uint8_t command_byte, void *block_to_write,
                                   uint8_t maxlen);

/// A logical step of an I2C protocol.
/// Not every stage is relevant to every I2C operation, and some stages may be executed multiple
/// times. e.g. I2C_STEP_TRANSMIT is executed once for every sent byte
typedef enum I2CProtocolStep {
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
} I2CProtocolStep;

/// The progress of the I2C operation.
typedef struct I2CProgress {
    /// The next logical step of the I2C process
    I2CProtocolStep resume_at;
    /// number of bytes we have already written. If the I2C operation ends early, this may be less
    /// than nbytes_write_len
    uint8_t nbytes_written;
    /// number of bytes we intend to read. In the case of a block read, this does not include the
    /// length byte
    uint8_t nbytes_read_len;
    /// number of bytes we have already read.
    uint8_t nbytes_read;
    /// The result that we will eventually return. If we terminate the I2C operation early from an
    /// unexpected NACK or bus collision, this will hold that value while we issue a STOP condition.
    I2CResult result_after_stop;
} I2CProgress;

/// The progress of an unstarted I2C transaction.
/// When calling i2c_tick, create a copy of this value and pass it, by reference, as the `progress`
/// argument
static const I2CProgress I2C_PROGRESS_UNSTARTED = {.resume_at = I2C_STEP_UNSTARTED};

/// Asynchronously do the given operation. This function will continuously return I2C_NOTYET until
/// it's finished, then it will return either I2C_OKAY or I2C_ERROR
/// @param bus Which I2C bus to perform the I2C operation. A bus is stateful and may NOT interleave
/// different asynchronous operations on the same bus.
/// @param op Operation definition to perform
/// @param progress (in/out) status of the current I2C operation. Repeated calls to i2c_tick with a
/// given bus should reuse the same progress struct until i2c_tick returns a value other than
/// I2C_NOTYET
/// @return * I2C_NOTYET if the operation is not yet complete.
///         * I2C_OKAY if the operation completed successfully
///         * other values if the operation failed
I2CResult i2c_tick(I2CBus bus, const I2COperationDef *op, I2CProgress *progress);

/// Synchronously force the operation to completion
/// @param bus Which I2C bus to perform the I2C operation.
/// @param op Operation definition to perform
/// @return I2C_NOTYET if the operation is not yet complete.
///          I2C_OKAY if the operation completed successfully
///          other values if the operation failed
I2CResult i2c_synchronously_await(I2CBus bus, I2COperationDef op);

#endif
