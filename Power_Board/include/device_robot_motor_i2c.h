#include <stdint.h>

void i2c2_tick(bool should_reset, bool *did_finish);
void i2c3_tick(bool should_reset, bool *out_did_finish);

void re_init_i2c2(void);
void re_init_i2c3(void);
