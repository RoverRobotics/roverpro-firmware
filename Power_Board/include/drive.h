#ifndef DRIVE_H
#define DRIVE_H
#include "motor.h"
#include "stdint.h"

typedef struct {
    int16_t left;
    int16_t right;
    int16_t flipper;
} MotorEfforts;

static const MotorEfforts MOTOR_COMMANDS_BRAKE_ALL = {0, 0, 0};
static const MotorEfforts MOTOR_COMMANDS_COAST_ALL = {1, 1, 1};

void drive_init();
void drive_tick();
void drive_set_efforts(MotorEfforts e);
void drive_set_coast_lock(bool is_on);
bool drive_is_approximately_stopped();

#endif