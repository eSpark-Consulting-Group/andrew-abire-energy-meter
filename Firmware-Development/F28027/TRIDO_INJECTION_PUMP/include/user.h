#ifndef _USER_H_
#define _USER_H_
/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

//! \file   solutions/instaspin_foc/boards/boostxl-drv8305evm_revA/f28x/f2802xF/src/user.h
//! \brief  Contains the public interface for user initialization data for the CTRL, HAL, and EST modules 
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "ctrl_obj.h"
#include "est.h"
#include "est_Flux_states.h"
#include "est_Ls_states.h"
#include "est_Rs_states.h"
#include "est_states.h"
#include "motor.h"
#include "types.h"
#include "userParams.h"

//!
//!
//! \defgroup USER USER
//!
//@{


#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float_t USER_IQ_FULL_SCALE_CURRENT_A;
    float_t USER_IQ_FULL_SCALE_VOLTAGE_V;
    float_t USER_IQ_FULL_SCALE_FREQ_Hz;
    uint_least16_t USER_NUM_ISR_TICKS_PER_CTRL_TICK;
    uint_least16_t USER_NUM_CTRL_TICKS_PER_CURRENT_TICK;
    uint_least16_t USER_NUM_CTRL_TICKS_PER_EST_TICK;
    uint_least16_t USER_NUM_CTRL_TICKS_PER_SPEED_TICK;
    uint_least16_t USER_NUM_CTRL_TICKS_PER_TRAJ_TICK;
    uint_least8_t USER_NUM_CURRENT_SENSORS;
    uint_least8_t USER_NUM_VOLTAGE_SENSORS;
    float_t USER_OFFSET_POLE_rps;
    float_t USER_FLUX_POLE_rps;
    float_t USER_ZEROSPEEDLIMIT; // = (0.5 / USER_IQ_FULL_SCALE_FREQ_Hz);
    float_t USER_FORCE_ANGLE_FREQ_Hz; // = (2.0 * USER_ZEROSPEEDLIMIT * USER_IQ_FULL_SCALE_FREQ_Hz);
    float_t USER_MAX_ACCEL_Hzps;
    float_t USER_MAX_ACCEL_EST_Hzps;
    float_t USER_DIRECTION_POLE_rps;
    float_t USER_SPEED_POLE_rps;
    float_t USER_DCBUS_POLE_rps;
    float_t USER_FLUX_FRACTION;
    float_t USER_SPEEDMAX_FRACTION_FOR_L_IDENT;
    float_t USER_POWERWARP_GAIN;
    uint_least16_t USER_SYSTEM_FREQ_MHz;
    float_t USER_PWM_PERIOD_usec; //= (1000.0/USER_PWM_FREQ_kHz)
    float_t USER_VOLTAGE_SF; //= ((float_t)((USER_ADC_FULL_SCALE_VOLTAGE_V)/(USER_IQ_FULL_SCALE_VOLTAGE_V)))
    float_t USER_CURRENT_SF; //= ((float_t)((USER_ADC_FULL_SCALE_CURRENT_A)/(USER_IQ_FULL_SCALE_CURRENT_A)))
    float_t USER_VOLTAGE_FILTER_POLE_rps; //= (2.0 * MATH_PI *USER_VOLTAGE_FILTER_POLE_Hz)
    float_t USER_MAX_VS_MAG_PU;
    float_t USER_EST_KAPPAQ;
    MOTOR_Type_e USER_MOTOR_TYPE;
    uint_least16_t USER_MOTOR_NUM_POLE_PAIRS;
    float_t USER_MOTOR_RATED_FLUX;
    float_t USER_MOTOR_Rr;
    float_t USER_MOTOR_Rs;
    float_t USER_MOTOR_Ls_d;
    float_t USER_MOTOR_Ls_q;
    float_t USER_MOTOR_RES_EST_CURRENT;
    float_t USER_MOTOR_IND_EST_CURRENT;
    float_t USER_MOTOR_MAX_CURRENT;
    float_t USER_MAX_CURRENT_SLOPE; // = (USER_MOTOR_RES_EST_CURRENT/USER_IQ_FULL_SCALE_CURRENT_A/USER_TRAJ_FREQ_Hz)
    float_t USER_MAX_CURRENT_SLOPE_POWERWARP; // = (0.3*USER_MOTOR_RES_EST_CURRENT/USER_IQ_FULL_SCALE_CURRENT_A/USER_TRAJ_FREQ_Hz)
    float_t USER_MOTOR_MAGNETIZING_CURRENT;
    float_t USER_IDRATED_FRACTION_FOR_RATED_FLUX;
    float_t USER_IDRATED_FRACTION_FOR_L_IDENT;
    float_t USER_IDRATED_DELTA;
    float_t USER_MOTOR_FLUX_EST_FREQ_Hz;
    uint_least32_t USER_CTRL_FREQ_Hz; // = (uint_least32_t)(USER_ISR_FREQ_Hz/USER_NUM_ISR_TICKS_PER_CTRL_TICK)
    uint_least32_t USER_EST_FREQ_Hz; // = (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_EST_TICK)
    uint_least32_t USER_R_OVER_L_EST_FREQ_Hz;
    uint_least32_t USER_TRAJ_FREQ_Hz; // = (uint_least32_t)(USER_CTRL_FREQ_Hz/USER_NUM_CTRL_TICKS_PER_TRAJ_TICK)
    float_t USER_CTRL_PERIOD_sec; // = ((float_t)USER_CTRL_PERIOD_usec/(float_t)1000000.0)
    float_t USER_MAX_NEGATIVE_ID_REF_CURRENT_A; // = (-0.5 * USER_MOTOR_MAX_CURRENT)
    uint_least8_t USER_NUM_PWM_TICKS_PER_ISR_TICK;
    float_t USER_ISR_PERIOD_usec; // = (USER_PWM_PERIOD_usec * (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK)
    float_t USER_CTRL_PERIOD_usec; // = (USER_ISR_PERIOD_usec * USER_NUM_ISR_TICKS_PER_CTRL_TICK)
    float_t USER_ISR_FREQ_Hz; // = ((float_t)USER_PWM_FREQ_kHz * 1000.0 / (float_t)USER_NUM_PWM_TICKS_PER_ISR_TICK)
    float_t USER_PWM_FREQ_kHz;
    float_t USER_VOLTAGE_FILTER_POLE_Hz;
    float_t USER_ADC_FULL_SCALE_VOLTAGE_V;
    float_t USER_ADC_FULL_SCALE_CURRENT_A;
} USER_Var_t;

extern USER_Var_t gUserVar;
extern void USER_setVars(USER_Var_t *pvar, uint_least8_t mmodel);

// **************************************************************************
// the defines

//! \brief Defines the current sensor feedback positive/negative
#define POSITIVE_CURRENT_FEEDBACK         true

//! \brief CURRENTS AND VOLTAGES
// **************************************************************************
//! \brief ADC current offsets for A, B, and C phases
//! \brief One-time hardware dependent, though the calibration can be done at run-time as well
//! \brief After initial board calibration these values should be updated for your specific hardware so they are available after compile in the binary to be loaded to the controller
//! \brief Negative value when positive feedback circuit used
#define   I_A_offset (-1.0)
#define   I_B_offset (-1.0)
#define   I_C_offset (-1.0)

//! \brief ADC voltage offsets for A, B, and C phases
//! \brief One-time hardware dependent, though the calibration can be done at run-time as well
//! \brief After initial board calibration these values should be updated for your specific hardware so they are available after compile in the binary to be loaded to the controller
#define   V_A_offset (0.0)
#define   V_B_offset (0.0)
#define   V_C_offset (0.0)


//! \brief CLOCKS & TIMERS
// **************************************************************************

//! \brief Defines the address of estimator handle
//!
#define USER_EST_HANDLE_ADDRESS    (0x600)


//! \brief Defines the direct voltage (Vd) scale factor
//!
#define USER_VD_SF                 (0.95)

//#if (USER_MOTOR == Teknic_M_2311P_LN_04K)
//#define USER_MOTOR_TYPE                 MOTOR_Type_Pm
//#define USER_MOTOR_NUM_POLE_PAIRS       (4)
//#define USER_MOTOR_Rr                   (NULL)
//#define USER_MOTOR_Rs                   (0.345)
//#define USER_MOTOR_Ls_d                 (0.0003665)
//#define USER_MOTOR_Ls_q                 (0.0003665)
//#define USER_MOTOR_RATED_FLUX           (0.054)
//#define USER_MOTOR_MAGNETIZING_CURRENT  (NULL)
//#define USER_MOTOR_RES_EST_CURRENT      (1.0)
//#define USER_MOTOR_IND_EST_CURRENT      (-1.0)
//#define USER_MOTOR_MAX_CURRENT          (5.0)
//#define USER_MOTOR_FLUX_EST_FREQ_Hz     (20.0)

//#if (USER_MOTOR == OFFICE_1)                  // Name must match the motor #define
//#define USER_MOTOR_TYPE                 MOTOR_Type_Pm  // Motor_Type_Pm (All Synchronous: BLDC, PMSM, SMPM, IPM) or Motor_Type_Induction (Asynchronous ACI)
//#define USER_MOTOR_NUM_POLE_PAIRS       (5)            // PAIRS, not total poles. Used to calculate user RPM from rotor Hz only
//#define USER_MOTOR_Rr                   (NULL)         // Induction motors only, else NULL
//#define USER_MOTOR_Rs                   (0.4)     // Identified phase to neutral resistance in a Y equivalent circuit (Ohms, float)
//#define USER_MOTOR_Ls_d                 (0.0047)  // For PM, Identified average stator inductance  (Henry, float)
//#define USER_MOTOR_Ls_q                 (0.0041)  // For PM, Identified average stator inductance  (Henry, float)
//#define USER_MOTOR_RATED_FLUX           (0.53)         // Identified TOTAL flux linkage between the rotor and the stator (V/Hz)
//#define USER_MOTOR_MAGNETIZING_CURRENT  (NULL)         // Induction motors only, else NULL
//#define USER_MOTOR_RES_EST_CURRENT      (2.5)          // During Motor ID, maximum current (Amperes, float) used for Rs estimation, 10-20% rated current
//#define USER_MOTOR_IND_EST_CURRENT      (-2.5)         // During Motor ID, maximum current (negative Amperes, float) used for Ls estimation, use just enough to enable rotation
//#define USER_MOTOR_MAX_CURRENT          (8.0)         // CRITICAL: Used during ID and run-time, sets a limit on the maximum current command output of the provided Speed PI Controller to the Iq controller
//#define USER_MOTOR_FLUX_EST_FREQ_Hz     (10.0)         // During Motor ID, maximum commanded speed (Hz, float), ~10% rated

//#if (USER_MOTOR == OFFICE_2)
//#define USER_MOTOR_TYPE                 MOTOR_Type_Pm
//#define USER_MOTOR_NUM_POLE_PAIRS       (5)
//#define USER_MOTOR_Rr                   (NULL)
//#define USER_MOTOR_Rs                   (0.0253133)
//#define USER_MOTOR_Ls_d                 (0.00017)
//#define USER_MOTOR_Ls_q                 (0.00017)
//#define USER_MOTOR_RATED_FLUX           (0.06)
//#define USER_MOTOR_MAGNETIZING_CURRENT  (NULL)
//#define USER_MOTOR_RES_EST_CURRENT      (2.0)
//#define USER_MOTOR_IND_EST_CURRENT      (-2.0)
//#define USER_MOTOR_MAX_CURRENT          (8.0)
//#define USER_MOTOR_FLUX_EST_FREQ_Hz     (20.0)

//#if (USER_MOTOR == TRIDO_25)
//#define USER_MOTOR_TYPE                 MOTOR_Type_Pm
//#define USER_MOTOR_NUM_POLE_PAIRS       (5)
//#define USER_MOTOR_Rr                   (NULL)
//#define USER_MOTOR_Rs                   (0.432)
//#define USER_MOTOR_Ls_d                 (0.0055)
//#define USER_MOTOR_Ls_q                 (0.00404)
//#define USER_MOTOR_RATED_FLUX           (0.451)
//#define USER_MOTOR_MAGNETIZING_CURRENT  (NULL)
//#define USER_MOTOR_RES_EST_CURRENT      (2.0)
//#define USER_MOTOR_IND_EST_CURRENT      (-2.0)
//#define USER_MOTOR_MAX_CURRENT          (20.0)
//#define USER_MOTOR_FLUX_EST_FREQ_Hz     (30.0)

//#if (USER_MOTOR == TRIDO_35)
//#define USER_MOTOR_TYPE                 MOTOR_Type_Pm
//#define USER_MOTOR_NUM_POLE_PAIRS       (5)
//#define USER_MOTOR_Rr                   (NULL)
//#define USER_MOTOR_Rs                   (0.364)
//#define USER_MOTOR_Ls_d                 (0.004885)
//#define USER_MOTOR_Ls_q                 (0.004885)
//#define USER_MOTOR_RATED_FLUX           (0.5324)
//#define USER_MOTOR_MAGNETIZING_CURRENT  (NULL)
//#define USER_MOTOR_RES_EST_CURRENT      (2.0)
//#define USER_MOTOR_IND_EST_CURRENT      (-2.0)
//#define USER_MOTOR_MAX_CURRENT          (25.0)
//#define USER_MOTOR_FLUX_EST_FREQ_Hz     (10.0)

//#if (USER_MOTOR == TRIDO_50)
//#define USER_MOTOR_TYPE                 MOTOR_Type_Pm
//#define USER_MOTOR_NUM_POLE_PAIRS       (5)
//#define USER_MOTOR_Rr                   (NULL)
//#define USER_MOTOR_Rs                   (0.137)
//#define USER_MOTOR_Ls_d                 (0.00146)
//#define USER_MOTOR_Ls_q                 (0.00177)
//#define USER_MOTOR_RATED_FLUX           (0.3913)
//#define USER_MOTOR_MAGNETIZING_CURRENT  (NULL)
//#define USER_MOTOR_RES_EST_CURRENT      (2.0)
//#define USER_MOTOR_IND_EST_CURRENT      (-2.0)
//#define USER_MOTOR_MAX_CURRENT          (14.14)
//#define USER_MOTOR_FLUX_EST_FREQ_Hz     (3.0)

//#if (USER_MOTOR == TRIDO_GM)
//#define USER_MOTOR_TYPE                 MOTOR_Type_Pm
//#define USER_MOTOR_NUM_POLE_PAIRS       (5)
//#define USER_MOTOR_Rr                   (NULL)
//#define USER_MOTOR_Rs                   (0.27388)
//#define USER_MOTOR_Ls_d                 (0.003243)
//#define USER_MOTOR_Ls_q                 (0.003243)
//#define USER_MOTOR_RATED_FLUX           (0.588)
//#define USER_MOTOR_MAGNETIZING_CURRENT  (NULL)
//#define USER_MOTOR_RES_EST_CURRENT      (2.0)
//#define USER_MOTOR_IND_EST_CURRENT      (-2.0)
//#define USER_MOTOR_MAX_CURRENT          (30.0)
//#define USER_MOTOR_FLUX_EST_FREQ_Hz     (10.0)

//#if (USER_MOTOR == TRIDO_70)
//#define USER_MOTOR_TYPE                 MOTOR_Type_Pm
//#define USER_MOTOR_NUM_POLE_PAIRS       (5)
//#define USER_MOTOR_Rr                   (NULL)
//#define USER_MOTOR_Rs                   (0.1458)
//#define USER_MOTOR_Ls_d                 (0.00267)
//#define USER_MOTOR_Ls_q                 (0.00242)
//#define USER_MOTOR_RATED_FLUX           (0.527)
//#define USER_MOTOR_MAGNETIZING_CURRENT  (NULL)
//#define USER_MOTOR_RES_EST_CURRENT      (2.0)
//#define USER_MOTOR_IND_EST_CURRENT      (-2.0)
//#define USER_MOTOR_MAX_CURRENT          (40.0)
//#define USER_MOTOR_FLUX_EST_FREQ_Hz     (4.0)//4hz/5pole pair*60s=48rpm


// **************************************************************************
// the functions


//! \brief      Sets the user parameter values
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_setParams(USER_Params *pUserParams, USER_Var_t *pvar);


//! \brief      Checks for errors in the user parameter values
//! \param[in]  pUserParams  The pointer to the user param structure
extern void USER_checkForErrors(USER_Params *pUserParams, USER_Var_t *pvar);


//! \brief      Gets the error code in the user parameters
//! \param[in]  pUserParams  The pointer to the user param structure
//! \return     The error code
extern USER_ErrorCode_e USER_getErrorCode(USER_Params *pUserParams);


//! \brief      Sets the error code in the user parameters
//! \param[in]  pUserParams  The pointer to the user param structure
//! \param[in]  errorCode    The error code
extern void USER_setErrorCode(USER_Params *pUserParams,const USER_ErrorCode_e errorCode);


//! \brief      Recalculates Inductances with the correct Q Format
//! \param[in]  handle       The controller (CTRL) handle
extern void USER_softwareUpdate1p6(CTRL_Handle handle, USER_Var_t *pvar);


//! \brief      Updates Id and Iq PI gains
//! \param[in]  handle       The controller (CTRL) handle
extern void USER_calcPIgains(CTRL_Handle handle, USER_Var_t *pvar);


//! \brief      Computes the scale factor needed to convert from torque created by Ld, Lq, Id and Iq, from per unit to Nm
//! \return     The scale factor to convert torque from (Ld - Lq) * Id * Iq from per unit to Nm, in IQ24 format
extern _iq USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(USER_Var_t *pvar);


//! \brief      Computes the scale factor needed to convert from torque created by flux and Iq, from per unit to Nm
//! \return     The scale factor to convert torque from Flux * Iq from per unit to Nm, in IQ24 format
extern _iq USER_computeTorque_Flux_Iq_pu_to_Nm_sf(USER_Var_t *pvar);


//! \brief      Computes the scale factor needed to convert from per unit to Wb
//! \return     The scale factor to convert from flux per unit to flux in Wb, in IQ24 format
extern _iq USER_computeFlux_pu_to_Wb_sf(USER_Var_t *pvar);


//! \brief      Computes the scale factor needed to convert from per unit to V/Hz
//! \return     The scale factor to convert from flux per unit to flux in V/Hz, in IQ24 format
extern _iq USER_computeFlux_pu_to_VpHz_sf(USER_Var_t *pvar);


//! \brief      Computes Flux in Wb or V/Hz depending on the scale factor sent as parameter
//! \param[in]  handle       The controller (CTRL) handle
//! \param[in]  sf           The scale factor to convert flux from per unit to Wb or V/Hz
//! \return     The flux in Wb or V/Hz depending on the scale factor sent as parameter, in IQ24 format
extern _iq USER_computeFlux(CTRL_Handle handle, const _iq sf);


//! \brief      Computes Torque in Nm
//! \param[in]  handle          The controller (CTRL) handle
//! \param[in]  torque_Flux_sf  The scale factor to convert torque from (Ld - Lq) * Id * Iq from per unit to Nm
//! \param[in]  torque_Ls_sf    The scale factor to convert torque from Flux * Iq from per unit to Nm
//! \return     The torque in Nm, in IQ24 format
extern _iq USER_computeTorque_Nm(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf);


//! \brief      Computes Torque in lbin
//! \param[in]  handle          The controller (CTRL) handle
//! \param[in]  torque_Flux_sf  The scale factor to convert torque from (Ld - Lq) * Id * Iq from per unit to lbin
//! \param[in]  torque_Ls_sf    The scale factor to convert torque from Flux * Iq from per unit to lbin
//! \return     The torque in lbin, in IQ24 format
extern _iq USER_computeTorque_lbin(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf);


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _USER_H_ definition

