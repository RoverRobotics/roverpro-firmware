/// @file
/// Warns of too-high current draw during operation of the rover. Although the batteries
/// have protection circuitry to prevent damage, doing so will cause the power supply to black out
/// and this firmware (as well as any payload hardware) to lose power.

#ifndef POWER2_H
#define POWER2_H

/// Do an incremental amount of work to protect robot and the batteries.
/// - If charging, only connects one battery at a time to the main power bus
/// - If discharging, warns (via the overcurrent flag) if current draw is trending too high
void power_tick();

#endif