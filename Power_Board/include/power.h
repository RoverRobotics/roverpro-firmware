#ifndef POWER_H
#define POWER_H
/// Brings both batteries up. Since the power bus may draw a lot of current at first and different
/// batteries have different overcurrent protection logic (which is proprietary, undocumented dark
/// magic), this function toggles the batteries to keep power draw reasonable. Note that there are 2
/// power buses: The "digital bus" (which powers this microcontroller and we can't turn off) and the
/// "main bus" (which powers the motor and payload). We are only dealing here with the main bus, but
/// a hardware overcurrent condition would kill power to both!
void init_power();

void power_tick();
#endif