#ifndef MAIN_H
#define MAIN_H

#include "settings.h"
#include "stdhdr.h"

extern Settings g_settings;
extern bool g_overcurrent;

#define REGISTER_START()
#define REGISTER(a, b, c, d, e) extern e a;
#define REGISTER_END()
#include "registers.h"
#undef REGISTER_START
#undef REGISTER
#undef REGISTER_END

#endif