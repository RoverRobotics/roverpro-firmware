 /// @file

#include "usb_config.h"

// -------- INSTANTIATE TELEMETRY VARIABLES --------

#define REGISTER_START()
#define REGISTER( a, b, c, d, e )    e a;
#define REGISTER_END()
#define MESSAGE_START( a )
#define MEMBER( a )
#define MESSAGE_END()
#include "registers.h"
#undef  REGISTER_START
#undef  REGISTER
#undef  REGISTER_END
#undef  MESSAGE_START
#undef  MEMBER
#undef  MESSAGE_END

// -------- LIST OF INSTANTIATED VARIABLE LOCATION FOR SERIALIZATION --------

#define REGISTER_START()            struct REGISTER registers[] = { 
#define REGISTER( a, b, c, d, e )                                   {sizeof(e),d,c,b,&a},
#define REGISTER_END()                                              {0} };
#define MESSAGE_START( a )
#define MEMBER( a )
#define MESSAGE_END()
#include "registers.h"
#undef  REGISTER_START
#undef  REGISTER
#undef  REGISTER_END
#undef  MESSAGE_START
#undef  MEMBER
#undef  MESSAGE_END
