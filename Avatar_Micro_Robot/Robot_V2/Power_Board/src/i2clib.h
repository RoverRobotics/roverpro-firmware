#ifndef I2CLIB_H
#define I2CLIB_H

#include <stdint.h>
#include <stdbool.h>

/** I2C ack bit. Transmitted in response to any data received. */
typedef enum i2c_ack_t {
    ACK = 0,  ///< ACKnowledge: the byte was successfully received and another byte may be sent
    NACK = 1, ///< Not ACKnowledge: either there is no receiver able to respond, the last command
    ///< wasn't understood, the receiver can't process any more data. Upon receiving a NACK, the
    ///< right thing to do is to stop.
} i2c_ack_t;

/** I2C read/write bit. This always accompanies the address of a device and indicates whether we are
 * writing to or reading from the device*/
typedef enum i2c_readwrite_t {
    I2C_WRITE = 0, ///< The slave address is being opened in WRITE mode. We will transmit data.
    I2C_READ = 1,  ///< The slave address is being opened in READ mode. We will receive data.
} i2c_readwrite_t;

/** The result of an I2C operation. */
typedef enum i2c_result_t {
    I2C_NOTYET,  ///< Bus is still busy with the last operation. Try again in a bit.
    I2C_OKAY,    ///< The operation completed successfully
    I2C_ERROR,   ///< Operation was not completed due to a problem, e.g. bus contention with another
                 ///< master. The master should issue a STOP condition.
    I2C_ILLEGAL, ///< Incorrect use of the I2C module, probably by calling functions in the wrong
                 ///< order.
} i2c_result_t;

/// Reference to the I2C bus definition. The caller should treat this as an opaque type
typedef const struct i2c_busdef_t *i2c_bus_t;

i2c_result_t i2c_enable(i2c_bus_t bus);
i2c_result_t i2c_start(i2c_bus_t bus);
i2c_result_t i2c_stop(i2c_bus_t bus);
i2c_result_t i2c_restart(i2c_bus_t bus);
i2c_result_t i2c_request_byte(i2c_bus_t bus);
i2c_result_t i2c_address(i2c_bus_t bus, uint8_t addr, i2c_readwrite_t r);
i2c_result_t i2c_transmit_byte(i2c_bus_t bus, uint8_t data);
i2c_result_t i2c_check_ack(i2c_bus_t bus, i2c_ack_t *ack);
i2c_result_t i2c_receive_and_ack(i2c_bus_t bus, i2c_ack_t acknack, uint8_t *data);

/** These are the I2C buses available on our hardware (PIC24). */
#ifdef __PIC24FJ256GB106__

extern const i2c_bus_t I2C_BUS1;
extern const i2c_bus_t I2C_BUS2;
extern const i2c_bus_t I2C_BUS3;

#endif

/** Represents an asynchronous operation. */
typedef struct i2c_operationdef_t {
    uint8_t address;
    bool no_command_byte;
    uint8_t command_byte;
    bool write_starts_with_len;
    uint8_t size_writebuf;
    uint8_t *writebuf;
    bool read_starts_with_len;
    uint8_t size_readbuf;
    uint8_t *readbuf;
} i2c_operationdef_t;

i2c_operationdef_t i2c_op_read_byte(uint8_t address, uint8_t command_byte, uint8_t *byte_to_read);
i2c_operationdef_t i2c_op_read_word(uint8_t address, uint8_t command_byte, uint16_t *word_to_read);
i2c_operationdef_t i2c_op_read_block(uint8_t address, uint8_t command_byte, uint8_t *block_to_read,
                                     uint8_t maxlen);

i2c_operationdef_t i2c_op_write_byte(uint8_t address, uint8_t command_byte,
                                     const uint8_t *byte_to_write);
i2c_operationdef_t i2c_op_write_word(uint8_t address, uint8_t command_byte,
                                     const uint16_t *word_to_write);
i2c_operationdef_t i2c_op_write_block(uint8_t address, uint8_t command_byte,
                                      uint8_t *block_to_write, uint8_t maxlen);
/** A logical step of the I2C protocol. */
typedef enum i2c_resume_at_t {
    I2C_STEP_START = 0,
    I2C_STEP_ADDRESS_W,
    I2C_STEP_COMMAND,
    I2C_STEP_TRANSMIT,
    I2C_STEP_RESTART,
    I2C_STEP_ADDRESS_R,
    I2C_STEP_REQUEST_FIRST,
    I2C_STEP_RECEIVE_FIRST,
    I2C_STEP_SENDACK,
    I2C_STEP_REQUEST,
    I2C_STEP_RECEIVE,
    I2C_STEP_SENDNACK,
    I2C_STEP_STOP,
    I2C_STEP_ABORT,
} i2c_resume_at_t;

typedef struct i2c_progress_t {
    i2c_resume_at_t resume_at;
    int nbytes_written;
    int nbytes_write_len;
    int nbytes_read;
    int nbytes_read_len;
} i2c_progress_t;

static const i2c_progress_t I2C_PROGRESS_UNSTARTED = {I2C_STEP_START};

/** Asynchronously do the given operation.
This function will continuously return I2C_NOTYET until it's finished, then it will return either
I2C_OKAY or I2C_ERROR */
i2c_result_t i2c_tick(i2c_bus_t bus, i2c_operationdef_t *op, i2c_progress_t *progress);

/** Synchronously force the operation to completion */
i2c_result_t i2c_synchronously_await(i2c_bus_t bus, i2c_operationdef_t *op);
#endif
