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
    I2C_OKAY,    ///< The operation completed successfully
    I2C_NOTYET,  ///< Bus is still busy with the last operation. Try again in a bit.
    I2C_ILLEGAL, ///< Incorrect use of the I2C protocol, probably by calling functions in the wrong
                 ///< order.
} i2c_result_t;

/// Reference to the I2C bus definition. The caller should treat this as an opaque type
typedef const struct i2c_busdef_t *i2c_bus_t;

i2c_result_t i2c_enable(i2c_bus_t bus);
i2c_result_t i2c_start(i2c_bus_t bus);
i2c_result_t i2c_stop(i2c_bus_t bus);
i2c_result_t i2c_restart(i2c_bus_t bus);
i2c_result_t i2c_request_byte(i2c_bus_t bus);
i2c_result_t i2c_address(i2c_bus_t bus, unsigned char addr, i2c_readwrite_t r);
i2c_result_t i2c_transmit_byte(i2c_bus_t bus, unsigned char data);
i2c_result_t i2c_check_ack(i2c_bus_t bus, i2c_ack_t *ack);
i2c_result_t i2c_receive_and_ack(i2c_bus_t bus, i2c_ack_t acknack, unsigned char *data);

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

typedef const struct i2c_operationdef_t *i2c_op_t;
i2c_operationdef_t i2c_op_readbyte(uint8_t address, uint8_t command_byte, uint8_t *byte_to_read);
i2c_operationdef_t i2c_op_readword(uint8_t address, uint8_t command_byte, uint16_t *word_to_read);
i2c_operationdef_t i2c_op_readblock(uint8_t address, uint8_t command_byte, uint8_t *block_to_read,
                                    uint8_t maxlen);

i2c_operationdef_t i2c_op_writebyte(uint8_t address, uint8_t command_byte,
                                    const uint16_t *byte_to_write);
i2c_operationdef_t i2c_op_writeword(uint8_t address, uint8_t command_byte,
                                    const uint16_t *word_to_write);
i2c_operationdef_t i2c_op_writeblock(uint8_t address, uint8_t command_byte, uint8_t *block_to_write,
                                     uint8_t maxlen);

typedef struct i2c_progress_t {
    int resume_at;
    int nbytes_written;
    int nbytes_write_len;
    int nbytes_read;
    int nbytes_read_len;
} i2c_progress_t;
/** Asynchronously do the given operation */
i2c_result_t i2c_tick(i2c_bus_t bus, i2c_operationdef_t *op, i2c_progress_t *progress);

#endif