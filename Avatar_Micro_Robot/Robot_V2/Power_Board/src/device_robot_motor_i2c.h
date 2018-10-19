#include <stdint.h>

void I2C2Update(void);
void I2C3Update(void);

void re_init_i2c2(void);
void re_init_i2c3(void);

extern bool I2C2TimerExpired;
extern bool I2C3TimerExpired;

extern bool I2C2XmitReset;
extern bool I2C3XmitReset;
