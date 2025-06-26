//================================================================================
// Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
//  ALL RIGHTS RESERVED
//================================================================================
/**
 * Performs exponential moving average calculation.  This macro requires the use of the
 * MATH_EMAVG_C structure.  The structure should be initialized with the MATH_EMAVG_C_INIT
 * macro.
 */

#ifndef _EMAVG_H_
#define _EMAVG_H_

/**
 * Exponential-moving average structure
 */
typedef struct {
    float In;
    float Out;
    float Multiplier;
} MATH_EMAVG_C;


/**
 * Performs exponential moving average calculation.
 * @param m - MATH_EMAVG_C structure with values.
 * @return MATH_EMAVG_C Out parameter.
 */
#define MATH_EMAVG_C(v)									\
v.Out = (v.In - v.Out) * v.Multiplier + v.Out

#endif /* _EMAVG_H_ */
