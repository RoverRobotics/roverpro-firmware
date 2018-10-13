/**
 * @file stdint.h
 * @author J. Brinton
 * @author Brinton Engineering LLC
 *
 * C99 stdint.h for the Microchip C30 compiler
 *
 * note this is probably all messed up because I just copied a bunch of
 * shit until my program compiled. The C30 compiler defines an unsigned int
 * as 16-bits... I guessed that a uint32_t would be a unsigned long int.
 *
 * best of luck to you and your kin. peace.
 *
 */


#ifndef STDINT_H_
#define STDINT_H_

#include <limits.h>


// 7.18.1 Integer types

// 7.18.1.1 Exact-width integer types
typedef signed char       int8_t;
typedef signed int        int16_t;
typedef signed long int   int32_t;
typedef unsigned char     uint8_t;
typedef unsigned int      uint16_t;
typedef unsigned long int uint32_t;

// 7.18.1.2 Minimum-width integer types
typedef int8_t    int_least8_t;
typedef int16_t   int_least16_t;
typedef int32_t   int_least32_t;
typedef uint8_t   uint_least8_t;
typedef uint16_t  uint_least16_t;
typedef uint32_t  uint_least32_t;

// 7.18.1.3 Fastest minimum-width integer types
typedef int8_t    int_fast8_t;
typedef int16_t   int_fast16_t;
typedef int32_t   int_fast32_t;
typedef uint8_t   uint_fast8_t;
typedef uint16_t  uint_fast16_t;
typedef uint32_t  uint_fast32_t;

// 7.18.1.4 Integer types capable of holding object pointers

// 7.18.1.5 Greatest-width integer types
typedef int32_t   intmax_t;
typedef uint32_t  uintmax_t;


// 7.18.2 Limits of specified-width integer types

// 7.18.2.1 Limits of exact-width integer types
#define INT8_MIN     ((int8_t)_I8_MIN)
#define INT8_MAX     _I8_MAX
#define INT16_MIN    ((int16_t)_I16_MIN)
#define INT16_MAX    _I16_MAX
#define INT32_MIN    ((int32_t)_I32_MIN)
#define INT32_MAX    _I32_MAX
#define UINT8_MAX    _UI8_MAX
#define UINT16_MAX   _UI16_MAX
#define UINT32_MAX   _UI32_MAX

// 7.18.2.2 Limits of minimum-width integer types
#define INT_LEAST8_MIN    INT8_MIN
#define INT_LEAST8_MAX    INT8_MAX
#define INT_LEAST16_MIN   INT16_MIN
#define INT_LEAST16_MAX   INT16_MAX
#define INT_LEAST32_MIN   INT32_MIN
#define INT_LEAST32_MAX   INT32_MAX
#define UINT_LEAST8_MAX   UINT8_MAX
#define UINT_LEAST16_MAX  UINT16_MAX
#define UINT_LEAST32_MAX  UINT32_MAX

// 7.18.2.3 Limits of fastest minimum-width integer types
#define INT_FAST8_MIN    INT8_MIN
#define INT_FAST8_MAX    INT8_MAX
#define INT_FAST16_MIN   INT16_MIN
#define INT_FAST16_MAX   INT16_MAX
#define INT_FAST32_MIN   INT32_MIN
#define INT_FAST32_MAX   INT32_MAX
#define UINT_FAST8_MAX   UINT8_MAX
#define UINT_FAST16_MAX  UINT16_MAX
#define UINT_FAST32_MAX  UINT32_MAX

// 7.18.2.4 Limits of integer types capable of holding object pointers

// 7.18.2.5 Limits of greatest-width integer types
#define INTMAX_MIN   INT32_MIN
#define INTMAX_MAX   INT32_MAX
#define UINTMAX_MAX  UINT32_MAX

// 7.18.3 Limits of other integer types

// 7.18.4 Limits of other integer types

// 7.18.4.1 Macros for minimum-width integer constants

// 7.18.4.2 Macros for greatest-width integer constants
#define INTMAX_C   INT32_C
#define UINTMAX_C  UINT32_C


#endif
