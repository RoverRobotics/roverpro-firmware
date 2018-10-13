/**
 * @file stdfunctions.c
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 *
 */
#include "stdhdr.h"
#include "i2c.h"

// these two lines needed for __delay_us
#define FCY 16000000UL // instruction clock
#include "libpic30.h"

int clamp(int value, int lo, int hi) { return (value < lo ? lo : value > hi ? hi : value); }

int mean(int count, int *values) {
    long total = 0;
    int i;
    for (i = 0; i < count; i++) {
        total += values[i];
    }
    return (int)(total / count);
}

long mean_l(int count, long *values) {
    long long total = 0;
    int i;
    for (i = 0; i < count; i++) {
        total += values[i];
    }
    return (int)(total / count);
}

void serial_delay_tick(ticks) {
    while (ticks--) {
    };
}

void block_ms(unsigned int ms) { __delay_ms(ms); }

unsigned char readI2C1_Reg(unsigned char add, unsigned char reg) // read an integer from address add
{
    unsigned char r;

    IdleI2C1();
    StartI2C1();
    IdleI2C1();

    MasterWriteI2C1(add << 1);

    IdleI2C1();

    MasterWriteI2C1(reg);

    IdleI2C1();
    StopI2C1();
    IdleI2C1();
    StartI2C1();
    IdleI2C1();

    MasterWriteI2C1((add << 1) | 0x01);
    IdleI2C1();

    __delay_us(100);

    IFS1bits.MI2C1IF = 0;

    // r = (unsigned char)(MasterReadI2C1());

    // AckI2C1();

    // IdleI2C1();

    r = (unsigned char)(MasterReadI2C1());

    // IdleI2C1();
    NotAckI2C1();

    // terminate read sequence (do not send ACK, send  STOP)
    IdleI2C1();
    StopI2C1();
    IdleI2C1();

    return r;
} // readI2C

unsigned char readI2C2_Reg(unsigned char add, unsigned char reg) // read an integer from address add
{
    unsigned char r;

    IdleI2C2();
    StartI2C2();
    IdleI2C2();

    MasterWriteI2C2(add << 1);

    IdleI2C2();

    MasterWriteI2C2(reg);

    IdleI2C2();
    StopI2C2();
    IdleI2C2();
    StartI2C2();
    IdleI2C2();

    MasterWriteI2C2((add << 1) | 0x01);
    IdleI2C2();

    __delay_us(100);

    _MI2C2IF = 0;

    // r = (unsigned char)(MasterReadI2C1());

    // AckI2C1();

    // IdleI2C1();

    r = (unsigned char)(MasterReadI2C2());

    // IdleI2C1();
    NotAckI2C2();

    // terminate read sequence (do not send ACK, send  STOP)
    IdleI2C2();
    StopI2C2();
    IdleI2C2();

    return r;
} // readI2C

unsigned char readI2C3_Reg(unsigned char add, unsigned char reg) // read an integer from address add
{
    unsigned char r;

    IdleI2C3();
    StartI2C3();
    IdleI2C3();

    MasterWriteI2C3(add << 1);

    IdleI2C3();

    MasterWriteI2C3(reg);

    IdleI2C3();
    StopI2C3();
    IdleI2C3();
    StartI2C3();
    IdleI2C3();

    MasterWriteI2C3((add << 1) | 0x01);
    IdleI2C3();

    __delay_us(100);

    _MI2C2IF = 0;

    // r = (unsigned char)(MasterReadI2C1());

    // AckI2C1();

    // IdleI2C1();

    r = (unsigned char)(MasterReadI2C3());

    // IdleI2C1();
    NotAckI2C3();

    // terminate read sequence (do not send ACK, send  STOP)
    IdleI2C3();
    StopI2C3();
    IdleI2C3();

    return r;
} // readI2C

void writeI2C1Reg(unsigned char add, unsigned char v,
                  unsigned char w) // write an integer v to address add
{
    IdleI2C1();
    StartI2C1();
    IdleI2C1();

    MasterWriteI2C1(add << 1);
    IdleI2C1();

    // Write data byte
    MasterWriteI2C1(v);
    IdleI2C1();

    // Write data byte
    MasterWriteI2C1(w);
    IdleI2C1();

    // Terminate command sequence with a stop condition
    StopI2C1();
    IdleI2C1();

} // writeI2C

void writeI2C2Reg(unsigned char add, unsigned char v,
                  unsigned char w) // write an integer v to address add
{
    IdleI2C2();
    StartI2C2();
    IdleI2C2();

    MasterWriteI2C2(add << 1);
    IdleI2C2();

    // Write data byte
    MasterWriteI2C2(v);
    IdleI2C2();

    // Write data byte
    MasterWriteI2C2(w);
    IdleI2C2();

    // Terminate command sequence with a stop condition
    StopI2C2();
    IdleI2C2();

} // writeI2C

void writeI2C3Reg(unsigned char add, unsigned char v,
                  unsigned char w) // write an integer v to address add
{
    IdleI2C3();
    StartI2C3();
    IdleI2C3();

    MasterWriteI2C3(add << 1);
    IdleI2C3();

    // Write data byte
    MasterWriteI2C3(v);
    IdleI2C3();

    // Write data byte
    MasterWriteI2C3(w);
    IdleI2C3();

    // Terminate command sequence with a stop condition
    StopI2C3();
    IdleI2C3();

} // writeI2C

unsigned int readI2C2_Word(unsigned char add, unsigned char reg) // read an integer from address add
{
    unsigned char a, b;

    IdleI2C2();
    StartI2C2();
    IdleI2C2();

    MasterWriteI2C2(add << 1);

    IdleI2C2();

    MasterWriteI2C2(reg);

    IdleI2C2();
    /*	StopI2C2();
            IdleI2C2();
            StartI2C2();*/
    RestartI2C2();
    IdleI2C2();

    MasterWriteI2C2((add << 1) | 0x01);
    IdleI2C2();

    __delay_us(100);

    _MI2C2IF = 0;

    // r = (unsigned char)(MasterReadI2C1());

    // AckI2C1();

    // IdleI2C1();

    a = (unsigned char)(MasterReadI2C2());

    AckI2C2();

    IdleI2C2();

    b = (unsigned char)(MasterReadI2C2());

    // IdleI2C1();
    NotAckI2C2();

    // terminate read sequence (do not send ACK, send  STOP)
    IdleI2C2();
    StopI2C2();
    IdleI2C2();

    return a + (b << 8);
} // readI2C

unsigned int readI2C3_Word(unsigned char add, unsigned char reg) // read an integer from address add
{
    unsigned char a, b;

    IdleI2C3();
    StartI2C3();
    IdleI2C3();

    MasterWriteI2C3(add << 1);

    IdleI2C3();

    MasterWriteI2C3(reg);

    IdleI2C3();
    /*	StopI2C3();
            IdleI2C3();
            StartI2C3();*/
    RestartI2C3();
    IdleI2C3();

    MasterWriteI2C3((add << 1) | 0x01);
    IdleI2C3();

    __delay_us(100);

    _MI2C3IF = 0;

    // r = (unsigned char)(MasterReadI2C1());

    // AckI2C1();

    // IdleI2C1();

    a = (unsigned char)(MasterReadI2C3());

    AckI2C3();

    IdleI2C3();

    b = (unsigned char)(MasterReadI2C3());

    // IdleI2C1();
    NotAckI2C3();

    // terminate read sequence (do not send ACK, send  STOP)
    IdleI2C3();
    StopI2C3();
    IdleI2C3();

    return a + (b << 8);
} // readI2C

void readI2C2_Block(unsigned char add, unsigned char reg, unsigned char block_length,
                    unsigned char *output) // read an integer from address add
{
    unsigned int i;

    for (i = 0; i < 20; i++) {
        output[i] = 'e';
    }

    IdleI2C2();
    StartI2C2();
    IdleI2C2();

    MasterWriteI2C2(add << 1);

    IdleI2C2();

    MasterWriteI2C2(reg);

    IdleI2C2();
    /*	StopI2C2();
            IdleI2C2();
            StartI2C2();*/
    RestartI2C2();
    IdleI2C2();

    MasterWriteI2C2((add << 1) | 0x01);
    IdleI2C2();

    __delay_us(100);

    _MI2C2IF = 0;

    // r = (unsigned char)(MasterReadI2C1());

    // AckI2C1();

    // IdleI2C1();

    for (i = 0; i < block_length; i++) {

        // first byte is length, don't include this in returned string
        if (i == 0)
            output[0] = (unsigned char)(MasterReadI2C2());
        else
            output[i - 1] = (unsigned char)(MasterReadI2C2());

        if (i == (block_length - 1)) {
            NotAckI2C2();
            break;
        }

        AckI2C2();
        IdleI2C2();
    }

    // IdleI2C1();

    // terminate read sequence (do not send ACK, send  STOP)
    IdleI2C2();
    StopI2C2();
    IdleI2C2();

    output[18] = 'x';

} // readI2C

void readI2C3_Block(unsigned char add, unsigned char reg, unsigned char block_length,
                    unsigned char *output) {
    unsigned int i;

    for (i = 0; i < 20; i++) {
        output[i] = 'e';
    }

    IdleI2C3();
    StartI2C3();
    IdleI2C3();

    MasterWriteI2C3(add << 1);

    IdleI2C3();

    MasterWriteI2C3(reg);

    IdleI2C3();
    /*	StopI2C2();
            IdleI2C2();
            StartI2C2();*/
    RestartI2C3();
    IdleI2C3();

    MasterWriteI2C3((add << 1) | 0x01);
    IdleI2C3();

    __delay_us(100);

    _MI2C3IF = 0;

    // r = (unsigned char)(MasterReadI2C1());

    // AckI2C1();

    // IdleI2C1();

    for (i = 0; i < block_length; i++) {

        // first byte is length, don't include this in returned string
        if (i == 0)
            output[0] = (unsigned char)(MasterReadI2C3());
        else
            output[i - 1] = (unsigned char)(MasterReadI2C3());

        if (i == (block_length - 1)) {
            NotAckI2C3();
            break;
        }

        AckI2C3();
        IdleI2C3();
    }

    // IdleI2C1();

    // terminate read sequence (do not send ACK, send  STOP)
    IdleI2C3();
    StopI2C3();
    IdleI2C3();

    output[18] = 'x';

} // readI2C
