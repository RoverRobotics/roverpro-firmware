/// @file
/// Declares shared globals @ref g_settings and @ref g_state for all rover functional areas

#ifndef MAIN_H
#define MAIN_H

#include "settings.h"
#include "state.h"
#include "stdhdr.h"

/// The current settings of the robot. Does not change over normal robot operation. May be changed
/// by commands sent over UART.
extern Settings g_settings;

/// The current state of the robot. Values may change frequently over normal robot operation. Some
/// additional state may be present in file-static or function-static variables, but any state
/// variable written by one functional area and read by another should live here.
extern State g_state;

#endif