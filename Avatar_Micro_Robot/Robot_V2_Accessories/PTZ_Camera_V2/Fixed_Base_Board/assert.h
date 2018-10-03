/*
 * @file assert.h
 * @author J. Brinton
 * @author Robotex Inc.
 *
 * assertion support for Robotex code 
 *
 */

#ifndef ASSERT_H
#define ASSERT_H

int _ASSERT(int val, int line, const char* expression, const char* path);

#define ASSERT(_EXPRESSION) _ASSERT((_EXPRESSION), __LINE__, \
                #_EXPRESSION, __FILE__)

#endif // ASSERT_H

