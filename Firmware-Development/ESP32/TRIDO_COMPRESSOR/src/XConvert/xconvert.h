#ifndef XCONVERT_H
#define XCONVERT_H
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) && __GNUC__ >= 10
#pragma GCC diagnostic ignored "-Wcast-function-type"
#endif

void xconvert (char* buf, const char* format, ...);

#ifdef __cplusplus
}
#endif

#endif
