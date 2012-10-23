void I2C3Update(void);
void I2C2Update(void);

void re_init_i2c2(void);
void re_init_i2c3(void);

unsigned char I2C2_read_block_nonblocking(unsigned char address, unsigned char reg, unsigned char length, unsigned char *output, unsigned char restart);

extern int I2C2TimerExpired;
extern int I2C3TimerExpired;

extern int I2C2XmitReset;
extern int I2C3XmitReset;
