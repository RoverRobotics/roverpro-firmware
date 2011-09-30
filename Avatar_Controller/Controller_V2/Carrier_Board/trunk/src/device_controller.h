/**
 * @file device_controller.h
 * @author J. Brinton
 * @author Robotex, Inc.
 *
 * Robotex controller PIC firmware.
 *
 */

extern int16_t gMotorVelocity;

void DeviceControllerInit();

void DeviceControllerProcessIO();
