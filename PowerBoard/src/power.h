/// @file
/// Handle bringing up the power supply. Prevents too-high current draw so the battery doesn't kill
/// power. Note this file is also used by the bootloader, so it shouldn't rely on rover settings or
/// any other rover subsystem

#pragma once

/// Brings both batteries up. Since the power bus may draw a lot of current at first and different
/// batteries have different overcurrent protection logic (which is proprietary, undocumented dark
/// magic), this function toggles the batteries to keep power draw reasonable. Note that there are 2
/// power buses: The "digital bus" (which powers this microcontroller and we can't turn off) and the
/// "main bus" (which powers the motor and payload). We are only dealing here with the main bus,
/// but a hardware overcurrent condition would kill power to both!
void power_init();
