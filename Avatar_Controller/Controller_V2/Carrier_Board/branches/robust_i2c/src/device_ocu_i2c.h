void start_ocu_batt_i2c_read(unsigned char add, unsigned char reg);
void start_ocu_batt_i2c_write(unsigned char add, unsigned char reg, unsigned int data);
void ocu_i2c1_isr(void);
void ocu_i2c1_fsm(void);
void start_ocu_i2c1_write(unsigned char add, unsigned char reg, unsigned int data);
void start_ocu_i2c1_i2c_read(unsigned char add, unsigned char reg);

void ocu_batt_i2c_fsm(void);
void ocu_batt_smbus_isr(void);

void init_i2c(void);

void i2c1_detect_failure(void);
void i2c2_detect_failure(void);

extern int left_battery_current;
extern int right_battery_current;


#define TOUCH_CONTROLLER_I2C_ADD 0x4d
#define SMBUS_ADD_TMP112_2 0x48
#define SMBUS_ADD_TMP112 0x49
#define SMBUS_ADD_BQ2060A 0x0b
#define SMBUS_ADD_BQ24745 0x09
#define I2C_ADD_FAN_CONTROLLER	0x18

#define I2C_ADD_HMC5843 0x1e
#define I2C_ADXL345_ADD           0x53

#define I2C_MUX_EN(a)		_TRISF3 = !a
#define I2C_MUX_CH(a)		_LATF3 = a
