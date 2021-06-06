/* per-machine configuration. this file is automatically generated. */

#ifndef _ODE_CONFIG_H_
#define _ODE_CONFIG_H_

/* standard system headers */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <malloc.h>
#include <alloca.h>
#include <values.h>
#include <float.h>

#ifdef __cplusplus
extern "C" {
#endif

/* is this a pentium on a gcc-based platform? */
/* #define PENTIUM 1 */

/* integer types (we assume int >= 32 bits) */
typedef char int8;
typedef unsigned char uint8;
typedef short int16;
typedef unsigned short uint16;
typedef int int32;
typedef unsigned int uint32;

/* an integer type that we can safely cast a pointer to and
 * from without loss of bits.
 */
typedef unsigned int intP;

/* select the base floating point type */
#define dSINGLE 1

#define dEpsilon FLT_EPSILON

/* the floating point infinity */
#define dInfinity 99999.f

/* available functions */

#ifdef __cplusplus
}
#endif
#endif
