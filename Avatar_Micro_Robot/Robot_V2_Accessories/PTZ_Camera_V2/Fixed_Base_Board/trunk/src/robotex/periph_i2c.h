/**
 * @file periph_i2c.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * API for I2C devices
 *
 *
 *
 * Firmware MUST configure/enable it's I2C hardware AND enable interrupts
 */


// callback prototype
typedef void (*I2C_FUNC)(unsigned char addr,
                         unsigned char bus, /* 1-indexed Microchip I2C bus */
                         unsigned char reg, /* zero if I2C read_current_adr */
                         unsigned char *data,
                         unsigned int  len);

// registers callback for I2C read operations
int registerI2CReadCallback(I2C_FUNC, unsigned char bus, unsigned char addr);

// registers callback for I2C write operations
int registerI2CWriteCallback(I2C_FUNC, unsigned char bus, unsigned char addr);



// asyncronous I2C bus write returns -1 if busy, -2 invalid, 0 successful
int writeI2C(unsigned char bus,  /* 1-indexed Microchip I2C bus */
             unsigned char addr, /* I2C device address */
             unsigned char *data,
             unsigned int  len);

// syncronous I2C bus write returns -1 if busy, -2 invalid, 0 successful
int writeI2CBlocking(unsigned char bus,  /* 1-indexed Microchip I2C bus */
                     unsigned char addr, /* I2C device address */
                     unsigned char *data,
                     unsigned int  len);

// asyncronous I2C bus write returns -1 if busy, -2 invalid, 0 successful
int writeRegI2C(unsigned char bus,  /* 1-indexed Microchip I2C bus */
                unsigned char addr, /* I2C device address */
                unsigned char reg,
                unsigned char *data,
                unsigned int  len);

// syncronous I2C bus write returns -1 if busy, -2 invalid, 0 successful
int writeRegI2CBlocking(unsigned char bus,  /* 1-indexed Microchip I2C bus */
                        unsigned char addr, /* I2C device address */
                        unsigned char reg,
                        unsigned char *data,
                        unsigned int  len);

// asyncronous I2C bus write returns -1 if busy, -2 invalid, 0 successful
int writeRegByteI2C(unsigned char bus,
                    unsigned char addr,
                    unsigned char reg,
                    unsigned char data);

// syncronous I2C bus write, returns -1 if busy, -2 invalid, 0 if successful
int writeRegByteI2CBlocking(unsigned char bus,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char data);





/* asyncronous I2C bus read - calls message handler
 returns -1 if busy, -2 invalid, 0 successful */
int readI2C(unsigned char bus,  /* 1-indexed Microchip I2C bus */
            unsigned char addr, /* I2C device address */
            unsigned int  len);

// syncronous version of the function above returns pointer to packet data
unsigned char *readI2CBlocking(unsigned char bus,
                               unsigned char addr,
                               unsigned int  len);

/* asyncronous I2C bus read - calls message handler
   returns -1 if busy, -2 invalid, 0 successful */
int readRegI2C(unsigned char bus,  /* 1-indexed Microchip I2C bus */
               unsigned char addr, /* I2C device address */
               unsigned char reg,  /* I2C device register */
               unsigned int  len);

// syncronous version of the function above returns pointer to packet data
unsigned char *readRegI2CBlocking(unsigned char bus,
                                  unsigned char addr,
                                  unsigned char reg,
                                  unsigned int  len);







// returns 1 if I2C bus busy otherwise returns 0
int isBusyI2C(unsigned char bus); /* 1-index Microchip I2C bus */
