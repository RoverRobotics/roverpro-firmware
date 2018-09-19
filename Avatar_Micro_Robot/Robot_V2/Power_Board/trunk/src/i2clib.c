#include "i2clib.h"

i2c_result_t i2c_enable(i2c_bus_t bus){
	bus->CON->I2CEN = 1;
	return I2C_OKAY;
}
i2c_state_t i2c_state(i2c_bus_t bus) {
	i2c_state_t state = (
		  bus->CON->SEN    ? I2C_SEN
		: bus->CON->RSEN   ? I2C_RSEN
		: bus->CON->PEN    ? I2C_PEN
		: bus->CON->RCEN   ? I2C_RCEN
		: bus->CON->ACKEN  ? I2C_ACKEN
		: bus->STAT->TRSTAT? I2C_TRANSMITTING
		: bus->STAT->S     ? I2C_STARTED
		: bus->CON->I2CEN  ? I2C_STOPPED
		: I2C_DISABLED
	);
	return state;
}

i2c_result_t i2c_start(i2c_bus_t bus){
	i2c_state_t state = i2c_state(bus);
	switch (state){
		case I2C_STARTED:
		case I2C_STOPPED:
			break;
		case I2C_PEN:   // start after stopping
		case I2C_TRANSMITTING: // repeated start condition
			return I2C_NOTYET;
		default:
			return I2C_ILLEGAL;
	}
	bus->CON->SEN = 1;
	return I2C_OKAY;
}

i2c_result_t i2c_stop(i2c_bus_t bus){
	switch (i2c_state(bus)){
		case I2C_TRANSMITTING:
		case I2C_STARTED:
			break;
		case I2C_DISABLED:
			return I2C_ILLEGAL;
		default:
			return I2C_NOTYET;
	}
	bus->CON->PEN = 1;
	return I2C_OKAY;
}

i2c_result_t i2c_restart(i2c_bus_t bus){
	switch (i2c_state(bus)){
		case I2C_TRANSMITTING:
		case I2C_STARTED:
			break;
		case I2C_DISABLED:
			return I2C_ILLEGAL;
		default:
			return I2C_NOTYET;
	}
	bus->CON->RSEN = 1;
	return I2C_OKAY;
}

i2c_result_t i2c_receive(i2c_bus_t bus){
	switch (i2c_state(bus)){
		case I2C_STARTED:
			break;
		case I2C_TRANSMITTING: // still sending data
		case I2C_RCEN: // still receiving the previous byte
			return I2C_NOTYET;
		default:
			return I2C_ILLEGAL;
	}
	bus->CON->RCEN = 1;
	return I2C_OKAY;
}

i2c_result_t i2c_write_addr(i2c_bus_t bus, unsigned char addr, i2c_readwrite_t rw){
	switch(i2c_state(bus)){
		case I2C_STARTED:
			break;
		case I2C_TRANSMITTING:
		case I2C_SEN:
		case I2C_RSEN:
			return I2C_NOTYET;
		default:
			return I2C_ILLEGAL;
	}
	*(bus->TRN) = (addr << 1) + rw;
	if (bus->STAT->IWCOL) {
		bus->STAT->IWCOL = 0;
		return I2C_NOTYET;
	}
	return I2C_OKAY;
}

i2c_result_t i2c_write_data(i2c_bus_t bus, unsigned char data){
	switch(i2c_state(bus)){
		case I2C_STARTED:
			break;
		case I2C_TRANSMITTING:
			return I2C_NOTYET;
		default:
			return I2C_ILLEGAL;
	}
	*(bus->TRN) = data;
	if (bus->STAT->IWCOL) {
		bus->STAT->IWCOL = 0;
		return I2C_NOTYET;
	}
	return I2C_OKAY;
}

i2c_result_t i2c_check_ack(i2c_bus_t bus, i2c_ack_t * ack){
	switch(i2c_state(bus)){
		case I2C_STARTED:
			break;
		case I2C_TRANSMITTING:
			return I2C_NOTYET;
		default:
			return I2C_ILLEGAL;
	}
	*ack = bus->STAT->ACKSTAT;
	return I2C_OKAY;
}

i2c_result_t i2c_read_byte(i2c_bus_t bus, i2c_ack_t acknack, unsigned char * data){
	switch(i2c_state(bus)){
		case I2C_STARTED:
			break;
		case I2C_RCEN:
			return I2C_NOTYET;
		default:
			return I2C_ILLEGAL;
	}
	*data = *bus->RCV;
	bus->CON->ACKDT = acknack;
	bus->CON->ACKEN = 1;
	return I2C_OKAY;
}

