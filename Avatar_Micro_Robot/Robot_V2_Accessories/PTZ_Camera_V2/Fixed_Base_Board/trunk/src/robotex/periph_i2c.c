/**
 * @file periph_i2c.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * I2C bus firmware driver for Microchip PIC24F
 *
 *
 * Method writeI2CBus() sets the driver state to begin a process of I2C
 * transfers. If the driver is busy it returns -1. Each control sequence
 * sets the drivers state and returns immediately waiting for the hardware
 * interrupt to trigger the next control sequence:
 *
 * writeI2CBus()         : 1) check if device is free
 *                         2) initiate master START
 *                         3) set state to I2C_WRITE_START
 *                         4) return
 * _MI2C1Interrupt()     : 1) confirm master START
 *                         2) initiate master address WRITE
 *                         3) set state to I2C_WRITE_ADDR
 *                         4) return
 * _MI2C1Interrupt()     : 1) confirm master WRITE
 *                         2) initiate master data WRITE
 *                         3) set state to I2C_WRITE_DATA
 *                         4) return
 * _MI2C1Interrupt()     : 1) confirm master WRITE
 *                         2) initiate master data WRITE
 *                         3) set state to I2C_WRITE_DATA
 *                         4) return
 * _MI2C1Interrupt()     : 1) confirm master WRITE
 *                         2) initiate master STOP
 *                         3) set state to I2C_STOP
 *                         4) return
 * _MI2C1Interrupt()     : 1) confirm I2C STOP
 *                         2) set state to I2C_IDLE
 *                         3) return
 *
 */

#include <string.h>
#include "p24Fxxxx.h"

#include "periph_i2c.h"

// max number of I2C bus callbacks
#define I2C_MAX_CALLBACKS 10 /*functions*/
#define I2C_BUS_BUFFER 256 /*bytes*/

// I2C callback vector
struct I2C_CALLBACKS
{
   I2C_FUNC func;
   unsigned char bus;
   unsigned char addr;
} __i2c_read_callbacks[I2C_MAX_CALLBACKS] = { { 0 } },
  __i2c_write_callbacks[I2C_MAX_CALLBACKS] = { { 0 } };

unsigned int __i2c_read_callback_pos = 0;
unsigned int __i2c_write_callback_pos = 0;


// I2C driver state
typedef enum I2C_STATES_T
{
   I2C_IDLE = 0,

   I2C_WRITE_START,
   I2C_WRITE_ADDR,
   I2C_WRITE_DATA,
   I2C_WRITE_STOP,

   I2C_READ_START,
   I2C_READ_ADDR,
   I2C_READ_DATA,
   I2C_READ_NACK,
   I2C_READ_ACK,
   I2C_READ_STOP,

   I2C_READREG_START,
   I2C_READREG_ADDR,
   I2C_READREG_WRITE,
   I2C_READREG_RESTART,
} I2C_STATES;

// ****************************************************************************
// DRIVER
// ****************************************************************************

// I2C bus driver
struct I2C_DRIVER
{
   I2C_STATES state;
   unsigned int  pos;
   unsigned int  len;
   unsigned char addr;
   unsigned char reg;
   unsigned char buf[I2C_BUS_BUFFER];
} __i2c_driver[3] = { { 0 } };


// ****************************************************************************
// FUNCTIONS
// ****************************************************************************

I2C1CONBITS* __i2c_get_hardware_cntl(unsigned int bus)
{
   switch (bus)
   {
      case 1:
         return (I2C1CONBITS*)&I2C1CONbits;
      case 2:
         return (I2C1CONBITS*)&I2C2CONbits;
      case 3:
         return (I2C1CONBITS*)&I2C3CONbits;
   }
   return 0;
}

// find callback to handle incoming packet
void __i2c_read_callback(struct I2C_DRIVER *driver, unsigned char mbus,
                         unsigned char reg)
{
   int i;
   I2C_FUNC func = 0;

   for (i = 0; i < I2C_MAX_CALLBACKS; i++)
   {
      if ((mbus         == __i2c_read_callbacks[i].bus) &&
          (driver->addr == __i2c_read_callbacks[i].addr))
      {
         func = __i2c_read_callbacks[i].func;
      }
   }

   if( func != 0 )
   {
      func(driver->addr, mbus, reg, driver->buf, driver->len);
   }
}

// find callback to handle completion of outgoing
void __i2c_write_callback(struct I2C_DRIVER *driver, unsigned char mbus,
                         unsigned char reg)
{
   int i;
   I2C_FUNC func = 0;

   for (i = 0; i < I2C_MAX_CALLBACKS; i++)
   {
      if ((mbus         == __i2c_write_callbacks[i].bus) &&
          (driver->addr == __i2c_write_callbacks[i].addr))
      {
         func = __i2c_write_callbacks[i].func;
      }
   }

   if( func != 0 )
   {
      func(driver->addr, mbus, reg, driver->buf, driver->len);
   }
}

// handles generic i2c hardware routines
void __i2c_routine(struct I2C_DRIVER *driver, volatile I2C1CONBITS *I2CCONbits,
                   volatile unsigned int *I2CTRN, volatile unsigned int *I2CRCV,
                   unsigned char bus)
{
   switch (driver->state)
   {
      // after START interrupt TRANSMIT address
      case I2C_WRITE_START:
         driver->state = I2C_WRITE_ADDR;
         *I2CTRN = (driver->addr << 1) & ~0x01;
         break;

      case I2C_WRITE_ADDR:
      case I2C_WRITE_DATA:
         if( driver->pos >= driver->len ) {  // no more data to send
            driver->state = I2C_WRITE_STOP;
            I2CCONbits->PEN = 1; }
         else {  // data left in buffer
            driver->state = I2C_WRITE_DATA;
            *I2CTRN = driver->buf[driver->pos];
            driver->pos++; }
         break;

      case I2C_WRITE_STOP:
         driver->state = I2C_IDLE;
		__i2c_write_callback(driver, bus, driver->reg);
         break;


      // after START interrupt TRANSMIT address
      case I2C_READ_START:
         *I2CTRN = (driver->addr << 1) | 0x01;
         driver->state = I2C_READ_ADDR;
         break;

      // after address TRANSMIT acknowledgement RECEIVE data
      case I2C_READ_ADDR:
         driver->state = I2C_READ_DATA;
         I2CCONbits->RCEN = 1; // receive enable
         break;

      case I2C_READ_DATA:
         driver->buf[driver->pos] = *I2CRCV;
         driver->pos++;
         if( driver->pos < driver->len ) { // receive more data
            I2CCONbits->ACKDT = 0; // ACK vs. NACK
            I2CCONbits->ACKEN = 1; // send ACK
            driver->state = I2C_READ_ACK; }
         else { // done, send NACK and STOP
            I2CCONbits->ACKDT = 1; // NACK vs. ACK
            I2CCONbits->ACKEN = 1; // send ACK
            driver->state = I2C_READ_NACK; }
         break;

      case I2C_READ_ACK:
         driver->state = I2C_READ_DATA;
         I2CCONbits->RCEN = 1; // receive enable
         break;

      case I2C_READ_NACK:
         I2CCONbits->PEN = 1; // STOP
         driver->state = I2C_READ_STOP;
         break;

      case I2C_READ_STOP:
         driver->state = I2C_IDLE;
		__i2c_read_callback(driver, bus, driver->reg);
         break;


      // after START interrupt TRANSMIT address
      case I2C_READREG_START:
         *I2CTRN = (driver->addr << 1) & ~0x01;
         driver->state = I2C_READREG_ADDR;
         break;

      // after address TRANSMIT acknowledgement TRANSMIT reg
      case I2C_READREG_ADDR:
         driver->state = I2C_READREG_WRITE;
         *I2CTRN = driver->reg;
         break;

      case I2C_READREG_WRITE:
         driver->state = I2C_READREG_RESTART;
         I2CCONbits->RSEN = 1; // repeat start
         break;

      case I2C_READREG_RESTART:
         driver->state = I2C_READ_ADDR;
         *I2CTRN = (driver->addr << 1) | 0x01;
         break;


      default:
         return;
   }
   return;
}


// Master I2C interupt 1
void _ISR __attribute__((auto_psv)) _MI2C1Interrupt(void)
{
   _MI2C1IF=0;
   __i2c_routine(&__i2c_driver[0], (I2C1CONBITS*)&I2C1CONbits,
                 &I2C1TRN, &I2C1RCV, 1);
}


// Master I2C interrupt 2
void _ISR __attribute__((auto_psv)) _MI2C2Interrupt(void)
{
   _MI2C2IF=0;
   __i2c_routine(&__i2c_driver[1], (I2C1CONBITS*)&I2C2CONbits,
                 &I2C2TRN, &I2C2RCV, 2);
}


// Master I2C interrupt 3
void _ISR __attribute__((auto_psv)) _MI2C3Interrupt(void)
{
   _MI2C3IF=0;
   __i2c_routine(&__i2c_driver[2], (I2C1CONBITS*)&I2C3CONbits,
                 &I2C3TRN, &I2C3RCV, 3);
}


// ****************************************************************************
// API DEFINITIONS
// ****************************************************************************


// registers data handler for i2c messages
int registerI2CReadCallback(I2C_FUNC func, unsigned char bus, unsigned char addr)
{
   if (__i2c_read_callback_pos >= I2C_MAX_CALLBACKS) return -1;

   __i2c_read_callbacks[__i2c_read_callback_pos].func = func;
   __i2c_read_callbacks[__i2c_read_callback_pos].bus  = bus;
   __i2c_read_callbacks[__i2c_read_callback_pos].addr = addr;

   __i2c_read_callback_pos++;

   return 0;
}

// registers data handler for i2c messages
int registerI2CWriteCallback(I2C_FUNC func, unsigned char bus, unsigned char addr)
{
   if (__i2c_write_callback_pos >= I2C_MAX_CALLBACKS) return -1;

   __i2c_write_callbacks[__i2c_write_callback_pos].func = func;
   __i2c_write_callbacks[__i2c_write_callback_pos].bus  = bus;
   __i2c_write_callbacks[__i2c_write_callback_pos].addr = addr;

   __i2c_write_callback_pos++;

   return 0;
}

// returns 1 if I2C bus busy otherwise returns 0
int isBusyI2C(unsigned char bus)
{
   if( __i2c_driver[bus-1].state != I2C_IDLE ) return 1;
   else return 0;
};

// asyncronous I2C bus write, returns -1 if busy, -2 invalid, 0 if successful
int writeI2C(unsigned char bus,
             unsigned char addr,
             unsigned char *data,
             unsigned int len)
{
   if( isBusyI2C(bus) ) return -1; // busy

   if (len == 0) return -2;
   if (len > I2C_BUS_BUFFER) return -2;
   if ((bus > 3) || (bus < 1)) return -2;

   memcpy(__i2c_driver[bus-1].buf, data, len);
   __i2c_driver[bus-1].pos   = 0;
   __i2c_driver[bus-1].len   = len;
   __i2c_driver[bus-1].addr  = addr;
   __i2c_driver[bus-1].state = I2C_WRITE_START;
   // Best estimate for most devices. Only used in read callback.
   __i2c_driver[bus-1].reg   = data[0] + len - 1;
   __i2c_get_hardware_cntl(bus)->SEN = 1; // send start condition

   return 0;
}

// syncronous I2C bus write, returns -1 if busy, -2 invalid, 0 if successful
int writeI2CBlocking(unsigned char bus,
                     unsigned char addr,
                     unsigned char *data,
                     unsigned int  len)
{
   int r;

   r = writeI2C(bus, addr, data, len);

   while( isBusyI2C(bus) ) { }

   return r;
}

// asyncronous I2C bus write, returns -1 if busy, -2 invalid, 0 if successful
int writeRegI2C(unsigned char bus,
                unsigned char addr,
                unsigned char reg,
                unsigned char *data,
                unsigned int len)
{
   if( isBusyI2C(bus) ) return -1; // busy

   if (len == 0) return -2;
   if (len + 1 > I2C_BUS_BUFFER) return -2;
   if ((bus > 3) || (bus < 1)) return -2;

   memcpy(__i2c_driver[bus-1].buf + 1, data, len);
   __i2c_driver[bus-1].buf[0] = reg;
   __i2c_driver[bus-1].pos   = 0;
   __i2c_driver[bus-1].len   = len + 1;
   __i2c_driver[bus-1].addr  = addr;
   __i2c_driver[bus-1].state = I2C_WRITE_START;
   // Best estimate for most devices. Only used in read callback.
   __i2c_driver[bus-1].reg   = reg + len - 1;
   __i2c_get_hardware_cntl(bus)->SEN = 1; // send start condition

   return 0;
}

// syncronous I2C bus write, returns -1 if busy, -2 invalid, 0 if successful
int writeRegI2CBlocking(unsigned char bus,
                        unsigned char addr,
                        unsigned char reg,
                        unsigned char *data,
                        unsigned int  len)
{
   int r;

   r = writeRegI2C(bus, addr, reg, data, len);

   while( isBusyI2C(bus) ) { }

   return r;
}

// asyncronous I2C bus write, returns -1 if busy, -2 invalid, 0 if successful
int writeRegByteI2C(unsigned char bus,
                unsigned char addr,
                unsigned char reg,
                unsigned char data)
{
   if( isBusyI2C(bus) ) return -1; // busy

   if ((bus > 3) || (bus < 1)) return -2;

   __i2c_driver[bus-1].buf[0] = reg;
   __i2c_driver[bus-1].buf[1] = data;
   __i2c_driver[bus-1].pos   = 0;
   __i2c_driver[bus-1].len   = 2;
   __i2c_driver[bus-1].addr  = addr;
   __i2c_driver[bus-1].state = I2C_WRITE_START;
   // Best estimate for most devices. Only used in read callback.
   __i2c_driver[bus-1].reg   = reg;
   __i2c_get_hardware_cntl(bus)->SEN = 1; // send start condition

   return 0;
}

// syncronous I2C bus write, returns -1 if busy, -2 invalid, 0 if successful
int writeRegByteI2CBlocking(unsigned char bus,
                            unsigned char addr,
                            unsigned char reg,
                            unsigned char data)
{
   int r;

   r = writeRegByteI2C(bus, addr, reg, data);

   while( isBusyI2C(bus) ) { }

   return r;
}

/* asyncronous I2C bus READ, returns -1 if busy, -2 invalid, 0 if successful.
   Callback is called once all data is read. */
int readI2C(unsigned char bus,
            unsigned char addr,
            unsigned int len)
{
   if( isBusyI2C(bus) ) return -1; // busy

   if (len == 0) return -2;
   if (len > I2C_BUS_BUFFER) return -2;
   if ((bus > 3) || (bus < 1)) return -2;

   __i2c_driver[bus-1].pos   = 0;
   __i2c_driver[bus-1].len   = len;
   __i2c_driver[bus-1].addr  = addr;
   __i2c_driver[bus-1].state = I2C_READ_START;
   __i2c_get_hardware_cntl(bus)->SEN = 1; // send start condition

   return 0;
}


/* syncronous I2C bus READ, returns pointer to packet data
   Callback is called once all data is read. */
unsigned char *readI2CBlocking(unsigned char bus,
                               unsigned char addr,
                               unsigned int len)
{
   int r;

   if( isBusyI2C(bus) ) return 0;

   r = readI2C(bus, addr, len);

   if( r < 0 ) return 0;

   while( isBusyI2C(bus) ) { };

   return __i2c_driver[bus-1].buf;
}


/* asyncronous I2C bus READ, returns -1 if busy, -2 invalid, 0 if successful.
   Callback is called once all data is read. */
int readRegI2C(unsigned char bus,
               unsigned char addr,
               unsigned char reg,
               unsigned int len)
{
   if( isBusyI2C(bus) ) return -1; // busy

   if (len == 0) return -2;
   if (len > I2C_BUS_BUFFER) return -2;
   if ((bus > 3) || (bus < 1)) return -2;

   __i2c_driver[bus-1].pos   = 0;
   __i2c_driver[bus-1].len   = len;
   __i2c_driver[bus-1].addr  = addr;
   __i2c_driver[bus-1].reg   = reg;
   __i2c_driver[bus-1].state = I2C_READREG_START;
   __i2c_get_hardware_cntl(bus)->SEN = 1; // send start condition

   return 0;
}

/* syncronous I2C bus READ, returns pointer to packet data
   Callback is called once all data is read. */
unsigned char *readRegI2CBlocking(unsigned char bus,
                                  unsigned char addr,
                                  unsigned char reg,
                                  unsigned int len)
{
   int r;

   if( isBusyI2C(bus) ) return 0;

   r = readRegI2C(bus, addr, reg, len);

   if( r < 0 ) return 0;

   while( isBusyI2C(bus) ) { };

   return __i2c_driver[bus-1].buf;
}
