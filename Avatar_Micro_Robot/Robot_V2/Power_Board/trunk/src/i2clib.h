// I2C Control Register
typedef struct i2c_con_t {
	unsigned SEN:1;
	unsigned RSEN:1;
	unsigned PEN:1;
	unsigned RCEN:1;
	unsigned ACKEN:1;
	unsigned ACKDT:1;
	unsigned STREN:1;
	unsigned GCEN:1;
	unsigned SMEN:1;
	unsigned DISSLW:1;
	unsigned A10M:1;
	unsigned IPMIEN:1;
	unsigned SCLREL:1;
	unsigned I2CSIDL:1;
	unsigned :1;
	unsigned I2CEN:1;
} i2c_con_t;

// I2C Status Register
typedef struct i2c_stat_t {
	unsigned TBF:1;
	unsigned RBF:1;
	unsigned R_W:1;
	unsigned S:1;
	unsigned P:1;
	unsigned D_A:1;
	unsigned I2COV:1;
	unsigned IWCOL:1;
	unsigned ADD10:1;
	unsigned GCSTAT:1;
	unsigned BCL:1;
	unsigned :3;
	unsigned TRSTAT:1;
	unsigned ACKSTAT:1;
} i2c_stat_t;

// I2C bus definition
typedef struct i2c_busdef_t {
	// Memory location of Control Register
	volatile i2c_con_t  *CON;
	// Memory location of Status Register
	volatile i2c_stat_t *STAT;
	// Memory location of Transmit Data Register
	volatile unsigned int *TRN;
	// Memory location of Receive Data Register
	volatile unsigned int *RCV;
} i2c_busdef_t;

// I2C ACK
typedef enum i2c_ack_t {
	ACK = 0,
	NACK = 1
} i2c_ack_t;

// I2C read/write bit
typedef enum i2c_readwrite_t {
	I2C_WRITE = 0,
	I2C_READ = 1
} i2c_readwrite_t;

// The result of an I2C operation.
typedef enum i2c_result_t {
	I2C_OKAY,
	I2C_NOTYET,      // I2C is still busy with the last operation. Try again in a bit.
	I2C_ILLEGAL,     // Incorrect use of the I2C protocol, probably by calling functions in the wrong order.
} i2c_result_t;

typedef enum i2c_state_t {
	I2C_STARTED=0, // I2C has issued a start condition and is idle
	I2C_STOPPED=1, // I2C is idle and has either not yet issued a start condition or has issued a stop condition
	I2C_TRANSMITTING=2, // I2C is currently writing out data
	I2C_SEN=3,
	I2C_RSEN=4,
	I2C_PEN=5,
	I2C_RCEN=6,
	I2C_ACKEN=7,
	I2C_DISABLED=9 // I2C is not running
} i2c_state_t;

// Reference to the I2C bus definition.
typedef const i2c_busdef_t * i2c_bus_t;

i2c_state_t i2c_state(i2c_bus_t bus);

i2c_result_t i2c_enable(i2c_bus_t bus);
i2c_result_t i2c_start(i2c_bus_t bus);
i2c_result_t i2c_stop(i2c_bus_t bus);
i2c_result_t i2c_restart(i2c_bus_t bus);
i2c_result_t i2c_receive(i2c_bus_t bus);
i2c_result_t i2c_write_addr(i2c_bus_t bus, unsigned char addr, i2c_readwrite_t r);
i2c_result_t i2c_write_byte(i2c_bus_t bus, unsigned char data);
i2c_result_t i2c_check_ack(i2c_bus_t bus, i2c_ack_t * ack);
i2c_result_t i2c_read_byte(i2c_bus_t bus, i2c_ack_t acknack, unsigned char * data);
