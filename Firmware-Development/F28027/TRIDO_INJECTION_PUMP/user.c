/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
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
//! \file   solutions/instaspin_foc/src/user.c
//! \brief Contains the function for setting initialization data to the CTRL, HAL, and EST modules
//!
//! (C) Copyright 2012, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include "main.h"
#include "user.h"
#include <math.h>
#include "ctrl.h"

USER_Var_t gUserVar;
extern uint16_t mtrModel;
extern uint16_t brdSeries;
extern volatile MOTOR_Vars_t gMotorVars;

// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


// **************************************************************************
// the functions

void USER_setVars(USER_Var_t *pvar, uint_least8_t mmodel)
{
  //! \brief Defines the system clock frequency, MHz
  pvar->USER_SYSTEM_FREQ_MHz = 60.0;
  //! \brief Defines the Pulse Width Modulation (PWM) frequency, kHz
  //! \brief PWM frequency can be set directly here up to 30 KHz safely (60 KHz MAX in some cases)
  //! \brief For higher PWM frequencies (60 KHz+ typical for low inductance, high current ripple motors) it is recommended to use the ePWM hardware
  //! \brief and adjustable ADC SOC to decimate the ADC conversion done interrupt to the control system, or to use the software Que example.
  //! \brief Otherwise you risk missing interrupts and disrupting the timing of the control state machine
  pvar->USER_PWM_FREQ_kHz = 8.0; //8.0 - 30.0 KHz typical; 45-80 KHz may be required for very low inductance, high speed motors
  //! \brief Defines the analog voltage filter pole location, Hz
  //! \brief Must match the hardware filter for Vph
  pvar->USER_VOLTAGE_FILTER_POLE_Hz = 112.0;
  //! \brief Defines the maximum voltage at the input to the AD converter
  //! \brief The value that will be represented by the maximum ADC input (3.3V) and conversion (0FFFh)
  //! \brief Hardware dependent, this should be based on the voltage sensing and scaling to the ADC input
  pvar->USER_ADC_FULL_SCALE_VOLTAGE_V = 43.3;
  //! \brief Defines the maximum current at the AD converter
  //! \brief The value that will be represented by the maximum ADC input (3.3V) and conversion (0FFFh)
  //! \brief Hardware dependent, this should be based on the current sensing and scaling to the ADC input
  pvar->USER_ADC_FULL_SCALE_CURRENT_A = 33.0;
  //! \brief Defines the full scale current for the IQ variables, A
  //! \brief All currents are converted into (pu) based on the ratio to this value
  //! \brief WARNING: this value MUST be larger than the maximum current readings that you are expecting from the motor or the reading will roll over to 0, creating a control issue
  switch(brdSeries)
  {
  case 0: // A = 0.002
    pvar->USER_IQ_FULL_SCALE_CURRENT_A = 42.0;
    //! \brief Defines full scale value for the IQ30 variable of Voltage inside the system
    //! \brief All voltages are converted into (pu) based on the ratio to this value
    //! \brief WARNING: this value MUST meet the following condition: USER_IQ_FULL_SCALE_VOLTAGE_V > 0.5 * USER_MOTOR_MAX_CURRENT * USER_MOTOR_Ls_d * USER_VOLTAGE_FILTER_POLE_rps,
    //! \brief WARNING: otherwise the value can saturate and roll-over, causing an inaccurate value
    //! \brief WARNING: this value is OFTEN greater than the maximum measured ADC value, especially with high Bemf motors operating at higher than rated speeds
    //! \brief WARNING: if you know the value of your Bemf constant, and you know you are operating at a multiple speed due to field weakening, be sure to set this value higher than the expected Bemf voltage
    //! \brief It is recommended to start with a value ~3x greater than the USER_ADC_FULL_SCALE_VOLTAGE_V and increase to 4-5x if scenarios where a Bemf calculation may exceed these limits
    //! \brief This value is also used to calculate the minimum flux value: USER_IQ_FULL_SCALE_VOLTAGE_V/USER_EST_FREQ_Hz/0.7
    pvar->USER_IQ_FULL_SCALE_VOLTAGE_V = 82.5;
    //! \brief Defines the full scale frequency for IQ variable, Hz
    //! \brief All frequencies are converted into (pu) based on the ratio to this value
    //! \brief this value MUST be larger than the maximum speed that you are expecting from the motor
    break;
  case 1: // D = 0.005
    pvar->USER_IQ_FULL_SCALE_CURRENT_A = 18.0;
    //! \brief Defines full scale value for the IQ30 variable of Voltage inside the system
    //! \brief All voltages are converted into (pu) based on the ratio to this value
    //! \brief WARNING: this value MUST meet the following condition: USER_IQ_FULL_SCALE_VOLTAGE_V > 0.5 * USER_MOTOR_MAX_CURRENT * USER_MOTOR_Ls_d * USER_VOLTAGE_FILTER_POLE_rps,
    //! \brief WARNING: otherwise the value can saturate and roll-over, causing an inaccurate value
    //! \brief WARNING: this value is OFTEN greater than the maximum measured ADC value, especially with high Bemf motors operating at higher than rated speeds
    //! \brief WARNING: if you know the value of your Bemf constant, and you know you are operating at a multiple speed due to field weakening, be sure to set this value higher than the expected Bemf voltage
    //! \brief It is recommended to start with a value ~3x greater than the USER_ADC_FULL_SCALE_VOLTAGE_V and increase to 4-5x if scenarios where a Bemf calculation may exceed these limits
    //! \brief This value is also used to calculate the minimum flux value: USER_IQ_FULL_SCALE_VOLTAGE_V/USER_EST_FREQ_Hz/0.7
    pvar->USER_IQ_FULL_SCALE_VOLTAGE_V = 36.0;
    //! \brief Defines the full scale frequency for IQ variable, Hz
    //! \brief All frequencies are converted into (pu) based on the ratio to this value
    //! \brief this value MUST be larger than the maximum speed that you are expecting from the motor
    break;
  default: // D = 0.005
      pvar->USER_IQ_FULL_SCALE_CURRENT_A = 18.0;
      //! \brief Defines full scale value for the IQ30 variable of Voltage inside the system
      //! \brief All voltages are converted into (pu) based on the ratio to this value
      //! \brief WARNING: this value MUST meet the following condition: USER_IQ_FULL_SCALE_VOLTAGE_V > 0.5 * USER_MOTOR_MAX_CURRENT * USER_MOTOR_Ls_d * USER_VOLTAGE_FILTER_POLE_rps,
      //! \brief WARNING: otherwise the value can saturate and roll-over, causing an inaccurate value
      //! \brief WARNING: this value is OFTEN greater than the maximum measured ADC value, especially with high Bemf motors operating at higher than rated speeds
      //! \brief WARNING: if you know the value of your Bemf constant, and you know you are operating at a multiple speed due to field weakening, be sure to set this value higher than the expected Bemf voltage
      //! \brief It is recommended to start with a value ~3x greater than the USER_ADC_FULL_SCALE_VOLTAGE_V and increase to 4-5x if scenarios where a Bemf calculation may exceed these limits
      //! \brief This value is also used to calculate the minimum flux value: USER_IQ_FULL_SCALE_VOLTAGE_V/USER_EST_FREQ_Hz/0.7
      pvar->USER_IQ_FULL_SCALE_VOLTAGE_V = 36.0;
      //! \brief Defines the full scale frequency for IQ variable, Hz
      //! \brief All frequencies are converted into (pu) based on the ratio to this value
      //! \brief this value MUST be larger than the maximum speed that you are expecting from the motor
  }
  pvar->USER_IQ_FULL_SCALE_FREQ_Hz = 150.0;
  //! \brief Defines the number of isr ticks (hardware) per controller clock tick (software)
  //! \brief Controller clock tick (CTRL) is the main clock used for all timing in the software
  //! \brief Typically the PWM Frequency triggers (can be decimated by the ePWM hardware for less overhead) an ADC SOC
  //! \brief ADC SOC triggers an ADC Conversion Done
  //! \brief ADC Conversion Done triggers ISR
  //! \brief This relates the hardware ISR rate to the software controller rate
  //! \brief Typically want to consider some form of decimation (ePWM hardware, CURRENT or EST) over 16KHz ISR to insure interrupt completes and leaves time for background tasks
  pvar->USER_NUM_ISR_TICKS_PER_CTRL_TICK = 2; // controller clock rate (CTRL) runs at PWM / 2; ex 30 KHz PWM, 15 KHz control
  //! \brief Defines the number of controller clock ticks per current controller clock tick
  //! \brief Relationship of controller clock rate to current controller (FOC) rate
  pvar->USER_NUM_CTRL_TICKS_PER_CURRENT_TICK = 1; // 1 Typical, Forward FOC current controller (Iq/Id/IPARK/SVPWM) runs at same rate as CTRL.
  //! \brief Defines the number of controller clock ticks per estimator clock tick
  //! \brief Relationship of controller clock rate to estimator (FAST) rate
  //! \brief Depends on needed dynamic performance, FAST provides very good results as low as 1 KHz while more dynamic or high speed applications may require up to 15 KHz
  pvar->USER_NUM_CTRL_TICKS_PER_EST_TICK = 1; // 1 Typical, FAST estimator runs at same rate as CTRL;
  //! \brief Defines the number of controller clock ticks per speed controller clock tick
  //! \brief Relationship of controller clock rate to speed loop rate
  pvar->USER_NUM_CTRL_TICKS_PER_SPEED_TICK = pvar->USER_PWM_FREQ_kHz; // Typical to match PWM, ex: 15KHz PWM, controller, and current loop, 1KHz speed loop
  //! \brief Defines the number of controller clock ticks per trajectory clock tick
  //! \brief Relationship of controller clock rate to trajectory loop rate
  //! \brief Typically the same as the speed rate
  pvar->USER_NUM_CTRL_TICKS_PER_TRAJ_TICK = pvar->USER_PWM_FREQ_kHz; // Typical to match PWM, ex: 10KHz controller & current loop, 1KHz speed loop, 1 KHz Trajectory
  //! \brief Defines the number of pwm clock ticks per isr clock tick
  //!  Note: Valid values are 1, 2 or 3 only
  pvar->USER_NUM_PWM_TICKS_PER_ISR_TICK = 3;
  //! \brief Defines the number of current sensors used
  //! \brief Defined by the hardware capability present
  //! \brief May be (2) or (3)
  pvar->USER_NUM_CURRENT_SENSORS = 3; // 3 Preferred setting for best performance across full speed range, allows for 100% duty cycle
  //! \brief Defines the number of voltage (phase) sensors
  //! \brief Must be (3)
  pvar->USER_NUM_VOLTAGE_SENSORS = 3; // 3 Required
  //! \brief Defines the software pole location for the voltage and current offset estimation, rad/s
  //! \brief Should not be changed from default of (20.0)
  pvar->USER_OFFSET_POLE_rps = 20.0; // 20.0 Default, do not change
  //! \brief Defines the software pole location for the flux estimation, rad/s
  //! \brief Should not be changed from default of (100.0)
  pvar->USER_FLUX_POLE_rps = 100.0; // 100.0 Default, do not change
  //! \brief Defines the low speed limit for the flux integrator, pu
  //! \brief This is the speed range (CW/CCW) at which the ForceAngle object is active, but only if Enabled
  //! \brief Outside of this speed - or if Disabled - the ForcAngle will NEVER be active and the angle is provided by FAST only
  pvar->USER_ZEROSPEEDLIMIT = 0.5 / pvar->USER_IQ_FULL_SCALE_FREQ_Hz; // 0.002 pu, 1-5 Hz typical; Hz = USER_ZEROSPEEDLIMIT * USER_IQ_FULL_SCALE_FREQ_Hz
  //! \brief Defines the force angle frequency, Hz
  //! \brief Frequency of stator vector rotation used by the ForceAngle object
  //! \brief Can be positive or negative
  pvar->USER_FORCE_ANGLE_FREQ_Hz = 2.0 * pvar->USER_ZEROSPEEDLIMIT * pvar->USER_IQ_FULL_SCALE_FREQ_Hz; // Default will keep FREQ >= 2.0 * low speed limit for the flux integrator
  //! \brief Defines the starting maximum acceleration AND deceleration for the speed profiles, Hz/s
  //! \brief Updated in run-time through user functions
  //! \brief Inverter, motor, inertia, and load will limit actual acceleration capability
  pvar->USER_MAX_ACCEL_Hzps = 20.0;  // 20.0 Default
  //! \brief Defines maximum acceleration for the estimation speed profiles, Hz/s
  //! \brief Only used during Motor ID (commission)
  pvar->USER_MAX_ACCEL_EST_Hzps = 5.0;  // 5.0 Default, don't change
  //! \brief Defines the software pole location for the direction filter, rad/s
  pvar->USER_DIRECTION_POLE_rps = 6.0; // 6.0 Default, do not change
  //! \brief Defines the software pole location for the speed control filter, rad/s
  pvar->USER_SPEED_POLE_rps = 100.0;  // 100.0 Default, do not change
  //! \brief Defines the software pole location for the DC bus filter, rad/s
  pvar->USER_DCBUS_POLE_rps = 100.0; // 100.0 Default, do not change
  //! \brief Defines flux fraction to use during inductance identification
  //!
  pvar->USER_FLUX_FRACTION = 1.0; // 1.0 Default, don't change
  //! \brief Defines the fraction of SpeedMax to use during inductance estimation
  //!
  pvar->USER_SPEEDMAX_FRACTION_FOR_L_IDENT = 1.0; // 1.0 Default, don't change
  //! \brief Defines the PowerWarp gain for computing Id reference
  //! \brief Induction motors only
  pvar->USER_POWERWARP_GAIN = 1.0; // 1.0 Default, don't change
  //! \brief Defines the voltage scale factor for the system
  //! \brief Compile time calculation for scale factor (ratio) used throughout the system
  pvar->USER_VOLTAGE_SF = ((float_t)((pvar->USER_ADC_FULL_SCALE_VOLTAGE_V)/(pvar->USER_IQ_FULL_SCALE_VOLTAGE_V)));
  //! \brief Defines the current scale factor for the system
  //! \brief Compile time calculation for scale factor (ratio) used throughout the system
  pvar->USER_CURRENT_SF = ((float_t)((pvar->USER_ADC_FULL_SCALE_CURRENT_A)/(pvar->USER_IQ_FULL_SCALE_CURRENT_A)));
  //! \brief Defines the analog voltage filter pole location, rad/s
  //! \brief Compile time calculation from Hz to rad/s
  pvar->USER_VOLTAGE_FILTER_POLE_rps = (2.0 * MATH_PI * pvar->USER_VOLTAGE_FILTER_POLE_Hz);
  //! \brief Defines the maximum Voltage vector (Vs) magnitude allowed.  This value sets the maximum magnitude for the output of the
  //! \brief Id and Iq PI current controllers.  The Id and Iq current controller outputs are Vd and Vq.
  //! \brief The relationship between Vs, Vd, and Vq is:  Vs = sqrt(Vd^2 + Vq^2).  In this FOC controller, the
  //! \brief Vd value is set equal to USER_MAX_VS_MAG*USER_VD_MAG_FACTOR.  Vq = sqrt(USER_MAX_VS_MAG^2 - Vd^2).
  //! \brief Set USER_MAX_VS_MAG = 0.5 for a pure sinewave with a peak at SQRT(3)/2 = 86.6% duty cycle.  No current reconstruction is needed for this scenario.
  //! \brief Set USER_MAX_VS_MAG = 1/SQRT(3) = 0.5774 for a pure sinewave with a peak at 100% duty cycle.  Current reconstruction will be needed for this scenario (Lab10a-x).
  //! \brief Set USER_MAX_VS_MAG = 2/3 = 0.6666 to create a trapezoidal voltage waveform.  Current reconstruction will be needed for this scenario (Lab10a-x).
  //! \brief For space vector over-modulation, see lab 10 for details on system requirements that will allow the SVM generator to go all the way to trapezoidal.
  pvar->USER_MAX_VS_MAG_PU = 0.5; // Set to 0.5 if a current reconstruction technique is not used.
  //! \brief Defines the convergence factor for the estimator
  //! \brief Do not change from default for FAST
  pvar->USER_EST_KAPPAQ = 1.5;   // 1.5 Default, do not change
  switch(mtrModel)
  {
  case 0: // TRIDO_25MM
      pvar->USER_MOTOR_TYPE = MOTOR_Type_Pm;
      pvar->USER_MOTOR_NUM_POLE_PAIRS = 5;
      pvar->USER_MOTOR_Rr = NULL;
      pvar->USER_MOTOR_Rs = 0.432;
      pvar->USER_MOTOR_Ls_d = 0.0055;
      pvar->USER_MOTOR_Ls_q = 0.00404;
      pvar->USER_MOTOR_RATED_FLUX = 0.451;
      pvar->USER_MOTOR_MAGNETIZING_CURRENT = NULL;
      pvar->USER_MOTOR_RES_EST_CURRENT = 2.0;
      pvar->USER_MOTOR_IND_EST_CURRENT = -2.0;
      pvar->USER_MOTOR_MAX_CURRENT = 20.0;
      if(brdSeries == 1)
        pvar->USER_MOTOR_MAX_CURRENT = 14.1;
      pvar->USER_MOTOR_FLUX_EST_FREQ_Hz = 30.0;
    break;
  case 1: // TRIDO_35MM
      pvar->USER_MOTOR_TYPE = MOTOR_Type_Pm;
      pvar->USER_MOTOR_NUM_POLE_PAIRS = 5;
      pvar->USER_MOTOR_Rr = NULL;
      pvar->USER_MOTOR_Rs = 0.364;
      pvar->USER_MOTOR_Ls_d = 0.004885;
      pvar->USER_MOTOR_Ls_q = 0.004885;
      pvar->USER_MOTOR_RATED_FLUX = 0.5324;
      pvar->USER_MOTOR_MAGNETIZING_CURRENT = NULL;
      pvar->USER_MOTOR_RES_EST_CURRENT = 2.0;
      pvar->USER_MOTOR_IND_EST_CURRENT = -2.0;
      pvar->USER_MOTOR_MAX_CURRENT = 25.0;
      if(brdSeries == 1)
        pvar->USER_MOTOR_MAX_CURRENT = 14.1;
      pvar->USER_MOTOR_FLUX_EST_FREQ_Hz = 10.0;
    break;
  case 2: // TRIDO_50MM
      pvar->USER_MOTOR_TYPE = MOTOR_Type_Pm;
      pvar->USER_MOTOR_NUM_POLE_PAIRS = 5;
      pvar->USER_MOTOR_Rr = NULL;
      pvar->USER_MOTOR_Rs = 0.137;
      pvar->USER_MOTOR_Ls_d = 0.00146;
      pvar->USER_MOTOR_Ls_q = 0.00177;
      pvar->USER_MOTOR_RATED_FLUX = 0.3913;
      pvar->USER_MOTOR_MAGNETIZING_CURRENT = NULL;
      pvar->USER_MOTOR_RES_EST_CURRENT = 2.0;
      pvar->USER_MOTOR_IND_EST_CURRENT = -2.0;
      pvar->USER_MOTOR_MAX_CURRENT = 14.1;
      pvar->USER_MOTOR_FLUX_EST_FREQ_Hz = 3.0;
    break;
  case 3: // TRIDO_70MM
      pvar->USER_MOTOR_TYPE = MOTOR_Type_Pm;
      pvar->USER_MOTOR_NUM_POLE_PAIRS = 5;
      pvar->USER_MOTOR_Rr = NULL;
      pvar->USER_MOTOR_Rs = 0.1458;
      pvar->USER_MOTOR_Ls_d = 0.00267;
      pvar->USER_MOTOR_Ls_q = 0.00242;
      pvar->USER_MOTOR_RATED_FLUX = 0.527;
      pvar->USER_MOTOR_MAGNETIZING_CURRENT = NULL;
      pvar->USER_MOTOR_RES_EST_CURRENT = 2.0;
      pvar->USER_MOTOR_IND_EST_CURRENT = -2.0;
      pvar->USER_MOTOR_MAX_CURRENT = 40.0;
      if(brdSeries == 1)
        pvar->USER_MOTOR_MAX_CURRENT = 14.1;
      pvar->USER_MOTOR_FLUX_EST_FREQ_Hz = 4.0;
    break;
  case 4: // TRIDO_GMI
      pvar->USER_MOTOR_TYPE = MOTOR_Type_Pm;
      pvar->USER_MOTOR_NUM_POLE_PAIRS = 5;
      pvar->USER_MOTOR_Rr = NULL;
      pvar->USER_MOTOR_Rs = 0.27388;
      pvar->USER_MOTOR_Ls_d = 0.003243;
      pvar->USER_MOTOR_Ls_q = 0.003243;
      pvar->USER_MOTOR_RATED_FLUX = 0.588;
      pvar->USER_MOTOR_MAGNETIZING_CURRENT = NULL;
      pvar->USER_MOTOR_RES_EST_CURRENT = 2.0;
      pvar->USER_MOTOR_IND_EST_CURRENT = -2.0;
      pvar->USER_MOTOR_MAX_CURRENT = 30.0;
      if(brdSeries == 1)
        pvar->USER_MOTOR_MAX_CURRENT = 14.1;
      pvar->USER_MOTOR_FLUX_EST_FREQ_Hz = 10.0;
    break;
  case 5: // TRIDO_70MM_LS
        pvar->USER_MOTOR_TYPE = MOTOR_Type_Pm;
        pvar->USER_MOTOR_NUM_POLE_PAIRS = 5;
        pvar->USER_MOTOR_Rr = NULL;
        pvar->USER_MOTOR_Rs = 0.16;
        pvar->USER_MOTOR_Ls_d = 0.00285925;
        pvar->USER_MOTOR_Ls_q = 0.0029065;
        pvar->USER_MOTOR_RATED_FLUX = 0.989646772;
        pvar->USER_MOTOR_MAGNETIZING_CURRENT = NULL;
        pvar->USER_MOTOR_RES_EST_CURRENT = 2.0;
        pvar->USER_MOTOR_IND_EST_CURRENT = -2.0;
        pvar->USER_MOTOR_MAX_CURRENT = 40.0;
        if(brdSeries == 1)
          pvar->USER_MOTOR_MAX_CURRENT = 14.1;
        pvar->USER_MOTOR_FLUX_EST_FREQ_Hz = 4.0;
      break;
  default:
      // TRIDO_50MM
      pvar->USER_MOTOR_TYPE = MOTOR_Type_Pm;
      pvar->USER_MOTOR_NUM_POLE_PAIRS = 5;
      pvar->USER_MOTOR_Rr = NULL;
      pvar->USER_MOTOR_Rs = 0.137;
      pvar->USER_MOTOR_Ls_d = 0.00146;
      pvar->USER_MOTOR_Ls_q = 0.00177;
      pvar->USER_MOTOR_RATED_FLUX = 0.3913;
      pvar->USER_MOTOR_MAGNETIZING_CURRENT = NULL;
      pvar->USER_MOTOR_RES_EST_CURRENT = 2.0;
      pvar->USER_MOTOR_IND_EST_CURRENT = -2.0;
      pvar->USER_MOTOR_MAX_CURRENT = 14.1;
      pvar->USER_MOTOR_FLUX_EST_FREQ_Hz = 3.0;
    break;
  }

  //! \brief Defines the Interrupt Service Routine (ISR) frequency, Hz
  //!
  pvar->USER_ISR_FREQ_Hz = ((float_t)pvar->USER_PWM_FREQ_kHz * 1000.0 / (float_t)pvar->USER_NUM_PWM_TICKS_PER_ISR_TICK);
  //! \brief Defines the controller frequency, Hz
  //! \brief Compile time calculation
  pvar->USER_CTRL_FREQ_Hz = (uint_least32_t)(pvar->USER_ISR_FREQ_Hz/pvar->USER_NUM_ISR_TICKS_PER_CTRL_TICK);
  //! \brief Defines the trajectory frequency, Hz
  //! \brief Compile time calculation
  pvar->USER_TRAJ_FREQ_Hz = (uint_least32_t)(pvar->USER_CTRL_FREQ_Hz/pvar->USER_NUM_CTRL_TICKS_PER_TRAJ_TICK);
  //! \brief Defines the estimator frequency, Hz
  //! \brief Compile time calculation
  pvar->USER_EST_FREQ_Hz = (uint_least32_t)(pvar->USER_CTRL_FREQ_Hz/pvar->USER_NUM_CTRL_TICKS_PER_EST_TICK);
  //! \brief Defines the maximum current slope for Id trajectory during estimation
  pvar->USER_MAX_CURRENT_SLOPE = (pvar->USER_MOTOR_RES_EST_CURRENT/pvar->USER_IQ_FULL_SCALE_CURRENT_A/pvar->USER_TRAJ_FREQ_Hz);
  //! \brief Defines the maximum current slope for Id trajectory during PowerWarp
  //! \brief For Induction motors only, controls how fast Id input can change under PowerWarp control
  // 0.3*RES_EST_CURRENT / IQ_FULL_SCALE_CURRENT / TRAJ_FREQ Typical to produce 1-sec rampup/down
  pvar->USER_MAX_CURRENT_SLOPE_POWERWARP = (0.3*pvar->USER_MOTOR_RES_EST_CURRENT/pvar->USER_IQ_FULL_SCALE_CURRENT_A/pvar->USER_TRAJ_FREQ_Hz);
  //! \brief Defines the fraction of IdRated to use during rated flux estimation
  //!
  pvar->USER_IDRATED_FRACTION_FOR_RATED_FLUX = 1.0; // 1.0 Default, don't change
  //! \brief Defines the fraction of IdRated to use during inductance estimation
  //!
  pvar->USER_IDRATED_FRACTION_FOR_L_IDENT = 1.0; // 1.0 Default, don't change
  //! \brief Defines the IdRated delta to use during estimation
  //!
  pvar->USER_IDRATED_DELTA = 0.00002;
  //! \brief Defines the R/L estimation frequency, Hz
  //! \brief User higher values for low inductance motors and lower values for higher inductance
  //! \brief motors.  The values can range from 100 to 300 Hz.
  pvar->USER_R_OVER_L_EST_FREQ_Hz = 100; // 300 Default for high speed motors, can reduce to 100 if RoverL from Motor ID is < 2000
  //! \brief Defines the maximum negative current to be applied in Id reference
  //! \brief Used in field weakening only, this is a safety setting (e.g. to protect against demagnetization)
  //! \brief User must also be aware that overall current magnitude [sqrt(Id^2 + Iq^2)] should be kept below any machine design specifications
  pvar->USER_MAX_NEGATIVE_ID_REF_CURRENT_A = (-0.5 * pvar->USER_MOTOR_MAX_CURRENT); // -0.5 * USER_MOTOR_MAX_CURRENT Example, adjust to meet safety needs of your motor
  //! \brief Defines the Pulse Width Modulation (PWM) period, usec
  //! \brief Compile time calculation
  pvar->USER_PWM_PERIOD_usec = (1000.0/pvar->USER_PWM_FREQ_kHz);
  //! \brief Defines the Interrupt Service Routine (ISR) period, usec
  //!
  pvar->USER_ISR_PERIOD_usec = (pvar->USER_PWM_PERIOD_usec * (float_t)pvar->USER_NUM_PWM_TICKS_PER_ISR_TICK);
  //! \brief Defines the controller execution period, usec
  //! \brief Compile time calculation
  pvar->USER_CTRL_PERIOD_usec = (pvar->USER_ISR_PERIOD_usec * pvar->USER_NUM_ISR_TICKS_PER_CTRL_TICK);
  //! \brief Defines the controller execution period, sec
  //! \brief Compile time calculation
  pvar->USER_CTRL_PERIOD_sec = ((float_t)pvar->USER_CTRL_PERIOD_usec/(float_t)1000000.0);
}

void USER_setParams(USER_Params *pUserParams, USER_Var_t *pvar)
{
  pUserParams->iqFullScaleCurrent_A = pvar->USER_IQ_FULL_SCALE_CURRENT_A;
  pUserParams->iqFullScaleVoltage_V = pvar->USER_IQ_FULL_SCALE_VOLTAGE_V;

  pUserParams->iqFullScaleFreq_Hz = pvar->USER_IQ_FULL_SCALE_FREQ_Hz;

  pUserParams->numIsrTicksPerCtrlTick = pvar->USER_NUM_ISR_TICKS_PER_CTRL_TICK;
  pUserParams->numCtrlTicksPerCurrentTick = pvar->USER_NUM_CTRL_TICKS_PER_CURRENT_TICK;
  pUserParams->numCtrlTicksPerEstTick = pvar->USER_NUM_CTRL_TICKS_PER_EST_TICK;
  pUserParams->numCtrlTicksPerSpeedTick = pvar->USER_NUM_CTRL_TICKS_PER_SPEED_TICK;
  pUserParams->numCtrlTicksPerTrajTick = pvar->USER_NUM_CTRL_TICKS_PER_TRAJ_TICK;

  pUserParams->numCurrentSensors = pvar->USER_NUM_CURRENT_SENSORS;
  pUserParams->numVoltageSensors = pvar->USER_NUM_VOLTAGE_SENSORS;

  pUserParams->offsetPole_rps = pvar->USER_OFFSET_POLE_rps;
  pUserParams->fluxPole_rps = pvar->USER_FLUX_POLE_rps;

  pUserParams->zeroSpeedLimit = pvar->USER_ZEROSPEEDLIMIT;

  pUserParams->forceAngleFreq_Hz = pvar->USER_FORCE_ANGLE_FREQ_Hz;

  pUserParams->maxAccel_Hzps = pvar->USER_MAX_ACCEL_Hzps;

  pUserParams->maxAccel_est_Hzps = pvar->USER_MAX_ACCEL_EST_Hzps;

  pUserParams->directionPole_rps = pvar->USER_DIRECTION_POLE_rps;

  pUserParams->speedPole_rps = pvar->USER_SPEED_POLE_rps;

  pUserParams->dcBusPole_rps = pvar->USER_DCBUS_POLE_rps;

  pUserParams->fluxFraction = pvar->USER_FLUX_FRACTION;

  pUserParams->indEst_speedMaxFraction = pvar->USER_SPEEDMAX_FRACTION_FOR_L_IDENT;

  pUserParams->systemFreq_MHz = pvar->USER_SYSTEM_FREQ_MHz;

  pUserParams->pwmPeriod_usec = pvar->USER_PWM_PERIOD_usec;

  pUserParams->voltage_sf = pvar->USER_VOLTAGE_SF;

  pUserParams->current_sf = pvar->USER_CURRENT_SF;

  pUserParams->voltageFilterPole_rps = pvar->USER_VOLTAGE_FILTER_POLE_rps;

  pUserParams->maxVsMag_pu = pvar->USER_MAX_VS_MAG_PU;

  pUserParams->estKappa = pvar->USER_EST_KAPPAQ;

  pUserParams->motor_type = pvar->USER_MOTOR_TYPE;
  pUserParams->motor_numPolePairs = pvar->USER_MOTOR_NUM_POLE_PAIRS;
  pUserParams->motor_ratedFlux = pvar->USER_MOTOR_RATED_FLUX;
  pUserParams->motor_Rr = pvar->USER_MOTOR_Rr;
  pUserParams->motor_Rs = pvar->USER_MOTOR_Rs;
  pUserParams->motor_Ls_d = pvar->USER_MOTOR_Ls_d;
  pUserParams->motor_Ls_q = pvar->USER_MOTOR_Ls_q;

/*  if((pUserParams->motor_Rr > (float_t)0.0) && (pUserParams->motor_Rs > (float_t)0.0))
    {
      pUserParams->powerWarpGain = sqrt((float_t)1.0 + pUserParams->motor_Rr/pUserParams->motor_Rs);
    }
  else
*/    {
      pUserParams->powerWarpGain = pvar->USER_POWERWARP_GAIN;
    }

  pUserParams->maxCurrent_resEst = pvar->USER_MOTOR_RES_EST_CURRENT;
  pUserParams->maxCurrent_indEst = pvar->USER_MOTOR_IND_EST_CURRENT;
  pUserParams->maxCurrent = pvar->USER_MOTOR_MAX_CURRENT;

  pUserParams->maxCurrentSlope = pvar->USER_MAX_CURRENT_SLOPE;
  pUserParams->maxCurrentSlope_powerWarp = pvar->USER_MAX_CURRENT_SLOPE_POWERWARP;

  pUserParams->IdRated = pvar->USER_MOTOR_MAGNETIZING_CURRENT;
  pUserParams->IdRatedFraction_ratedFlux = pvar->USER_IDRATED_FRACTION_FOR_RATED_FLUX;
  pUserParams->IdRatedFraction_indEst = pvar->USER_IDRATED_FRACTION_FOR_L_IDENT;
  pUserParams->IdRated_delta = pvar->USER_IDRATED_DELTA;

  pUserParams->fluxEstFreq_Hz = pvar->USER_MOTOR_FLUX_EST_FREQ_Hz;

  pUserParams->ctrlWaitTime[CTRL_State_Error]         = 0;
  pUserParams->ctrlWaitTime[CTRL_State_Idle]          = 0;
  pUserParams->ctrlWaitTime[CTRL_State_OffLine]       = (uint_least32_t)( 5.0 * pvar->USER_CTRL_FREQ_Hz);
  pUserParams->ctrlWaitTime[CTRL_State_OnLine]        = 0;

  pUserParams->estWaitTime[EST_State_Error]           = 0;
  pUserParams->estWaitTime[EST_State_Idle]            = 0;
  pUserParams->estWaitTime[EST_State_RoverL]          = (uint_least32_t)( 8.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_Rs]              = 0;
  pUserParams->estWaitTime[EST_State_RampUp]          = (uint_least32_t)((5.0 + pvar->USER_MOTOR_FLUX_EST_FREQ_Hz / pvar->USER_MAX_ACCEL_EST_Hzps) * pvar->USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_IdRated]         = (uint_least32_t)(30.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_RatedFlux_OL]    = (uint_least32_t)( 0.2 * pvar->USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_RatedFlux]       = 0;
  pUserParams->estWaitTime[EST_State_RampDown]        = (uint_least32_t)( 2.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_LockRotor]       = 0;
  pUserParams->estWaitTime[EST_State_Ls]              = 0;
  pUserParams->estWaitTime[EST_State_Rr]              = (uint_least32_t)(20.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->estWaitTime[EST_State_MotorIdentified] = 0;
  pUserParams->estWaitTime[EST_State_OnLine]          = 0;

  pUserParams->FluxWaitTime[EST_Flux_State_Error]     = 0;
  pUserParams->FluxWaitTime[EST_Flux_State_Idle]      = 0;
  pUserParams->FluxWaitTime[EST_Flux_State_CL1]       = (uint_least32_t)(10.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_CL2]       = (uint_least32_t)( 0.2 * pvar->USER_EST_FREQ_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_Fine]      = (uint_least32_t)( 4.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->FluxWaitTime[EST_Flux_State_Done]      = 0;

  pUserParams->LsWaitTime[EST_Ls_State_Error]        = 0;
  pUserParams->LsWaitTime[EST_Ls_State_Idle]         = 0;
  pUserParams->LsWaitTime[EST_Ls_State_RampUp]       = (uint_least32_t)( 3.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Init]         = (uint_least32_t)( 3.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Coarse]       = (uint_least32_t)( 0.2 * pvar->USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Fine]         = (uint_least32_t)(30.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->LsWaitTime[EST_Ls_State_Done]         = 0;

  pUserParams->RsWaitTime[EST_Rs_State_Error]        = 0;
  pUserParams->RsWaitTime[EST_Rs_State_Idle]         = 0;
  pUserParams->RsWaitTime[EST_Rs_State_RampUp]       = (uint_least32_t)( 1.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Coarse]       = (uint_least32_t)( 2.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Fine]         = (uint_least32_t)( 7.0 * pvar->USER_EST_FREQ_Hz);
  pUserParams->RsWaitTime[EST_Rs_State_Done]         = 0;

  pUserParams->ctrlFreq_Hz = pvar->USER_CTRL_FREQ_Hz;

  pUserParams->estFreq_Hz = pvar->USER_EST_FREQ_Hz;

  pUserParams->RoverL_estFreq_Hz = pvar->USER_R_OVER_L_EST_FREQ_Hz;

  pUserParams->trajFreq_Hz = pvar->USER_TRAJ_FREQ_Hz;

  pUserParams->ctrlPeriod_sec = pvar->USER_CTRL_PERIOD_sec;

  pUserParams->maxNegativeIdCurrent_a = pvar->USER_MAX_NEGATIVE_ID_REF_CURRENT_A;

  return;
} // end of USER_setParams() function


void USER_checkForErrors(USER_Params *pUserParams, USER_Var_t *pvar)
{
  USER_setErrorCode(pUserParams, USER_ErrorCode_NoError);

  if((pvar->USER_IQ_FULL_SCALE_CURRENT_A <= 0.0) ||
    (pvar->USER_IQ_FULL_SCALE_CURRENT_A <= (0.02 * pvar->USER_MOTOR_MAX_CURRENT * pvar->USER_IQ_FULL_SCALE_FREQ_Hz / 128.0)) ||
    (pvar->USER_IQ_FULL_SCALE_CURRENT_A <= (2.0 * pvar->USER_MOTOR_MAX_CURRENT * pvar->USER_IQ_FULL_SCALE_FREQ_Hz * pvar->USER_CTRL_PERIOD_sec / 128.0)))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleCurrent_A_Low);
    }

  if((pvar->USER_IQ_FULL_SCALE_CURRENT_A < pvar->USER_MOTOR_MAGNETIZING_CURRENT) ||
    (pvar->USER_IQ_FULL_SCALE_CURRENT_A < pvar->USER_MOTOR_RES_EST_CURRENT) ||
    (pvar->USER_IQ_FULL_SCALE_CURRENT_A < pvar->USER_MOTOR_IND_EST_CURRENT) ||
    (pvar->USER_IQ_FULL_SCALE_CURRENT_A < pvar->USER_MOTOR_MAX_CURRENT))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleCurrent_A_Low);
    }

  if((pvar->USER_MOTOR_RATED_FLUX > 0.0) && (pvar->USER_MOTOR_TYPE == MOTOR_Type_Pm))
    {
      if(pvar->USER_IQ_FULL_SCALE_VOLTAGE_V >= ((float_t)pvar->USER_EST_FREQ_Hz * pvar->USER_MOTOR_RATED_FLUX * 0.7))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_High);
        }
    }

  if((pvar->USER_MOTOR_RATED_FLUX > 0.0) && (pvar->USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
      if(pvar->USER_IQ_FULL_SCALE_VOLTAGE_V >= ((float_t)pvar->USER_EST_FREQ_Hz * pvar->USER_MOTOR_RATED_FLUX * 0.05))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_High);
        }
    }

  if((pvar->USER_IQ_FULL_SCALE_VOLTAGE_V <= 0.0) ||
    (pvar->USER_IQ_FULL_SCALE_VOLTAGE_V <= (0.5 * pvar->USER_MOTOR_MAX_CURRENT * pvar->USER_MOTOR_Ls_d * pvar->USER_VOLTAGE_FILTER_POLE_rps)) ||
    (pvar->USER_IQ_FULL_SCALE_VOLTAGE_V <= (0.5 * pvar->USER_MOTOR_MAX_CURRENT * pvar->USER_MOTOR_Ls_q * pvar->USER_VOLTAGE_FILTER_POLE_rps)))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleVoltage_V_Low);
    }

  if((pvar->USER_IQ_FULL_SCALE_FREQ_Hz > (4.0 * pvar->USER_VOLTAGE_FILTER_POLE_Hz)) ||
    (pvar->USER_IQ_FULL_SCALE_FREQ_Hz >= ((128.0 * pvar->USER_IQ_FULL_SCALE_CURRENT_A) / (0.02 * pvar->USER_MOTOR_MAX_CURRENT))) ||
    (pvar->USER_IQ_FULL_SCALE_FREQ_Hz >= ((128.0 * pvar->USER_IQ_FULL_SCALE_CURRENT_A) / (2.0 * pvar->USER_MOTOR_MAX_CURRENT * pvar->USER_CTRL_PERIOD_sec))) ||
    (pvar->USER_IQ_FULL_SCALE_FREQ_Hz >= (128.0 * (float_t)pvar->USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / 60.0)))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleFreq_Hz_High);
    }

  if((pvar->USER_IQ_FULL_SCALE_FREQ_Hz < 50.0) ||
    (pvar->USER_IQ_FULL_SCALE_FREQ_Hz < pvar->USER_MOTOR_FLUX_EST_FREQ_Hz) ||
    (pvar->USER_IQ_FULL_SCALE_FREQ_Hz < pvar->USER_SPEED_POLE_rps) ||
    (pvar->USER_IQ_FULL_SCALE_FREQ_Hz <= ((float_t)pvar->USER_MOTOR_NUM_POLE_PAIRS * 1000.0 / (60.0 * 128.0))) ||
    (pvar->USER_IQ_FULL_SCALE_FREQ_Hz < (pvar->USER_MAX_ACCEL_Hzps / ((float_t)pvar->USER_TRAJ_FREQ_Hz))) ||
    (pvar->USER_IQ_FULL_SCALE_FREQ_Hz < (pvar->USER_MAX_ACCEL_EST_Hzps / ((float_t)pvar->USER_TRAJ_FREQ_Hz))) ||
    (pvar->USER_IQ_FULL_SCALE_FREQ_Hz < ((float_t)pvar->USER_R_OVER_L_EST_FREQ_Hz)))
    {
	  USER_setErrorCode(pUserParams, USER_ErrorCode_iqFullScaleFreq_Hz_Low);
    }

  if(pvar->USER_NUM_PWM_TICKS_PER_ISR_TICK > 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numPwmTicksPerIsrTick_High);
    }

  if(pvar->USER_NUM_PWM_TICKS_PER_ISR_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numPwmTicksPerIsrTick_Low);
    }

  if(pvar->USER_NUM_ISR_TICKS_PER_CTRL_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numIsrTicksPerCtrlTick_Low);
    }

  if(pvar->USER_NUM_CTRL_TICKS_PER_CURRENT_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerCurrentTick_Low);
    }

  if(pvar->USER_NUM_CTRL_TICKS_PER_EST_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerEstTick_Low);
    }

  if((pvar->USER_NUM_CTRL_TICKS_PER_SPEED_TICK < 1) ||
    (pvar->USER_NUM_CTRL_TICKS_PER_SPEED_TICK < pvar->USER_NUM_CTRL_TICKS_PER_CURRENT_TICK))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerSpeedTick_Low);
    }

  if(pvar->USER_NUM_CTRL_TICKS_PER_TRAJ_TICK < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCtrlTicksPerTrajTick_Low);
    }

  if(pvar->USER_NUM_CURRENT_SENSORS > 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCurrentSensors_High);
    }

  if(pvar->USER_NUM_CURRENT_SENSORS < 2)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numCurrentSensors_Low);
    }

  if(pvar->USER_NUM_VOLTAGE_SENSORS > 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numVoltageSensors_High);
    }

  if(pvar->USER_NUM_VOLTAGE_SENSORS < 3)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_numVoltageSensors_Low);
    }

  if(pvar->USER_OFFSET_POLE_rps > ((float_t)pvar->USER_CTRL_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_offsetPole_rps_High);
    }

  if(pvar->USER_OFFSET_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_offsetPole_rps_Low);
    }

  if(pvar->USER_FLUX_POLE_rps > ((float_t)pvar->USER_EST_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxPole_rps_High);
    }

  if(pvar->USER_FLUX_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxPole_rps_Low);
    }

  if(pvar->USER_ZEROSPEEDLIMIT > 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_zeroSpeedLimit_High);
    }

  if(pvar->USER_ZEROSPEEDLIMIT <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_zeroSpeedLimit_Low);
    }

  if(pvar->USER_FORCE_ANGLE_FREQ_Hz > ((float_t)pvar->USER_EST_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_forceAngleFreq_Hz_High);
    }

  if(pvar->USER_FORCE_ANGLE_FREQ_Hz <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_forceAngleFreq_Hz_Low);
    }

  if(pvar->USER_MAX_ACCEL_Hzps > ((float_t)pvar->USER_TRAJ_FREQ_Hz * pvar->USER_IQ_FULL_SCALE_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_Hzps_High);
    }

  if(pvar->USER_MAX_ACCEL_Hzps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_Hzps_Low);
    }

  if(pvar->USER_MAX_ACCEL_EST_Hzps > ((float_t)pvar->USER_TRAJ_FREQ_Hz * pvar->USER_IQ_FULL_SCALE_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_est_Hzps_High);
    }

  if(pvar->USER_MAX_ACCEL_EST_Hzps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxAccel_est_Hzps_Low);
    }

  if(pvar->USER_DIRECTION_POLE_rps > ((float_t)pvar->USER_EST_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_directionPole_rps_High);
    }

  if(pvar->USER_DIRECTION_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_directionPole_rps_Low);
    }

  if((pvar->USER_SPEED_POLE_rps > pvar->USER_IQ_FULL_SCALE_FREQ_Hz) ||
    (pvar->USER_SPEED_POLE_rps > ((float_t)pvar->USER_EST_FREQ_Hz)))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_speedPole_rps_High);
    }

  if(pvar->USER_SPEED_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_speedPole_rps_Low);
    }

  if(pvar->USER_DCBUS_POLE_rps > ((float_t)pvar->USER_EST_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_dcBusPole_rps_High);
    }

  if(pvar->USER_DCBUS_POLE_rps <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_dcBusPole_rps_Low);
    }

  if(pvar->USER_FLUX_FRACTION > 1.2)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxFraction_High);
    }

  if(pvar->USER_FLUX_FRACTION < 0.05)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxFraction_Low);
    }

  if(pvar->USER_SPEEDMAX_FRACTION_FOR_L_IDENT > (pvar->USER_IQ_FULL_SCALE_CURRENT_A / pvar->USER_MOTOR_MAX_CURRENT))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_indEst_speedMaxFraction_High);
    }

  if(pvar->USER_SPEEDMAX_FRACTION_FOR_L_IDENT <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_indEst_speedMaxFraction_Low);
    }

  if(pvar->USER_POWERWARP_GAIN > 2.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_powerWarpGain_High);
    }

  if(pvar->USER_POWERWARP_GAIN < 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_powerWarpGain_Low);
    }

  if(pvar->USER_SYSTEM_FREQ_MHz > 90.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_systemFreq_MHz_High);
    }

  if(pvar->USER_SYSTEM_FREQ_MHz <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_systemFreq_MHz_Low);
    }

  if(pvar->USER_PWM_FREQ_kHz > (1000.0 * pvar->USER_SYSTEM_FREQ_MHz / 100.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_pwmFreq_kHz_High);
    }

  if(pvar->USER_PWM_FREQ_kHz < (1000.0 * pvar->USER_SYSTEM_FREQ_MHz / 65536.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_pwmFreq_kHz_Low);
    }

  if(pvar->USER_VOLTAGE_SF >= 128.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltage_sf_High);
    }

  if(pvar->USER_VOLTAGE_SF < 0.1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltage_sf_Low);
    }

  if(pvar->USER_CURRENT_SF >= 128.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_current_sf_High);
    }

  if(pvar->USER_CURRENT_SF < 0.1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_current_sf_Low);
    }

  if(pvar->USER_VOLTAGE_FILTER_POLE_Hz > ((float_t)pvar->USER_EST_FREQ_Hz / MATH_PI))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltageFilterPole_Hz_High);
    }

  if(pvar->USER_VOLTAGE_FILTER_POLE_Hz < (pvar->USER_IQ_FULL_SCALE_FREQ_Hz / 4.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_voltageFilterPole_Hz_Low);
    }

  if(pvar->USER_MAX_VS_MAG_PU > (4.0 / 3.0))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxVsMag_pu_High);
    }

  if(pvar->USER_MAX_VS_MAG_PU <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxVsMag_pu_Low);
    }

  if(pvar->USER_EST_KAPPAQ > 1.5)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_estKappa_High);
    }

  if(pvar->USER_EST_KAPPAQ < 1.5)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_estKappa_Low);
    }

  if((pvar->USER_MOTOR_TYPE != MOTOR_Type_Induction) && (pvar->USER_MOTOR_TYPE != MOTOR_Type_Pm))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_motor_type_Unknown);
    }

  if(pvar->USER_MOTOR_NUM_POLE_PAIRS < 1)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_motor_numPolePairs_Low);
    }

  if((pvar->USER_MOTOR_RATED_FLUX != 0.0) && (pvar->USER_MOTOR_TYPE == MOTOR_Type_Pm))
    {
      if(pvar->USER_MOTOR_RATED_FLUX > (pvar->USER_IQ_FULL_SCALE_FREQ_Hz * 65536.0 / (float_t)pvar->USER_EST_FREQ_Hz / 0.7))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_High);
        }

      if(pvar->USER_MOTOR_RATED_FLUX < (pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / (float_t)pvar->USER_EST_FREQ_Hz / 0.7))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_Low);
        }
    }

  if((pvar->USER_MOTOR_RATED_FLUX != 0.0) && (pvar->USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
      if(pvar->USER_MOTOR_RATED_FLUX > (pvar->USER_IQ_FULL_SCALE_FREQ_Hz * 65536.0 / (float_t)pvar->USER_EST_FREQ_Hz / 0.05))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_High);
        }

      if(pvar->USER_MOTOR_RATED_FLUX < (pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / (float_t)pvar->USER_EST_FREQ_Hz / 0.05))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_ratedFlux_Low);
        }
    }

  if(pvar->USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
      if(pvar->USER_MOTOR_Rr > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_High);
        }

      if(pvar->USER_MOTOR_Rr < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_Low);
        }
    }

  if((pvar->USER_MOTOR_Rr != 0.0) && (pvar->USER_MOTOR_TYPE == MOTOR_Type_Induction))
    {
      if(pvar->USER_MOTOR_Rr > (0.7 * 65536.0 * pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / pvar->USER_IQ_FULL_SCALE_CURRENT_A))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_High);
        }

      if(pvar->USER_MOTOR_Rr < (0.7 * pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / (pvar->USER_IQ_FULL_SCALE_CURRENT_A * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rr_Low);
        }
    }

  if(pvar->USER_MOTOR_Rs != 0.0)
    {
      if(pvar->USER_MOTOR_Rs > (0.7 * 65536.0 * pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / pvar->USER_IQ_FULL_SCALE_CURRENT_A))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rs_High);
        }

      if(pvar->USER_MOTOR_Rs < (0.7 * pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / (pvar->USER_IQ_FULL_SCALE_CURRENT_A * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Rs_Low);
        }
    }

  if(pvar->USER_MOTOR_Ls_d != 0.0)
    {
      if(pvar->USER_MOTOR_Ls_d > (0.7 * 65536.0 * pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / (pvar->USER_IQ_FULL_SCALE_CURRENT_A * pvar->USER_VOLTAGE_FILTER_POLE_rps)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_d_High);
        }

      if(pvar->USER_MOTOR_Ls_d < (0.7 * pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / (pvar->USER_IQ_FULL_SCALE_CURRENT_A * pvar->USER_VOLTAGE_FILTER_POLE_rps * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_d_Low);
        }
    }

  if(pvar->USER_MOTOR_Ls_q != 0.0)
    {
      if(pvar->USER_MOTOR_Ls_q > (0.7 * 65536.0 * pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / (pvar->USER_IQ_FULL_SCALE_CURRENT_A * pvar->USER_VOLTAGE_FILTER_POLE_rps)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_q_High);
        }

      if(pvar->USER_MOTOR_Ls_q < (0.7 * pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / (pvar->USER_IQ_FULL_SCALE_CURRENT_A * pvar->USER_VOLTAGE_FILTER_POLE_rps * 65536.0)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_motor_Ls_q_Low);
        }
    }

  if(pvar->USER_MOTOR_RES_EST_CURRENT > pvar->USER_MOTOR_MAX_CURRENT)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_resEst_High);
    }

  if(pvar->USER_MOTOR_RES_EST_CURRENT < 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_resEst_Low);
    }

  if(pvar->USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
      if(pvar->USER_MOTOR_IND_EST_CURRENT > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_High);
        }

      if(pvar->USER_MOTOR_IND_EST_CURRENT < (-pvar->USER_MOTOR_MAX_CURRENT))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_Low);
        }
    }

  if(pvar->USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(pvar->USER_MOTOR_IND_EST_CURRENT > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_High);
        }

      if(pvar->USER_MOTOR_IND_EST_CURRENT < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_indEst_Low);
        }
    }

  if(pvar->USER_MOTOR_MAX_CURRENT > pvar->USER_IQ_FULL_SCALE_CURRENT_A)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_High);
    }

  if(pvar->USER_MOTOR_MAX_CURRENT <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrent_Low);
    }

  if(pvar->USER_MAX_CURRENT_SLOPE > 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_High);
    }

  if(pvar->USER_MAX_CURRENT_SLOPE <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_Low);
    }

  if(pvar->USER_MAX_CURRENT_SLOPE_POWERWARP > 1.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_powerWarp_High);
    }

  if(pvar->USER_MAX_CURRENT_SLOPE_POWERWARP <= 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxCurrentSlope_powerWarp_Low);
    }

  if(pvar->USER_MOTOR_TYPE == MOTOR_Type_Pm)
    {
      if(pvar->USER_MOTOR_MAGNETIZING_CURRENT > 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_High);
        }

      if(pvar->USER_MOTOR_MAGNETIZING_CURRENT < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_Low);
        }
    }

  if(pvar->USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(pvar->USER_MOTOR_MAGNETIZING_CURRENT > pvar->USER_MOTOR_MAX_CURRENT)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_High);
        }

      if(pvar->USER_MOTOR_MAGNETIZING_CURRENT < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_Low);
        }
    }

  if(pvar->USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(pvar->USER_IDRATED_FRACTION_FOR_RATED_FLUX > (pvar->USER_IQ_FULL_SCALE_CURRENT_A / (1.2 * pvar->USER_MOTOR_MAX_CURRENT)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_ratedFlux_High);
        }

      if(pvar->USER_IDRATED_FRACTION_FOR_RATED_FLUX < 0.1)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_ratedFlux_Low);
        }
    }

  if(pvar->USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(pvar->USER_IDRATED_FRACTION_FOR_L_IDENT > (pvar->USER_IQ_FULL_SCALE_CURRENT_A / pvar->USER_MOTOR_MAX_CURRENT))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_indEst_High);
        }

      if(pvar->USER_IDRATED_FRACTION_FOR_L_IDENT < 0.1)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRatedFraction_indEst_Low);
        }
    }

  if(pvar->USER_MOTOR_TYPE == MOTOR_Type_Induction)
    {
      if(pvar->USER_IDRATED_DELTA > (pvar->USER_IQ_FULL_SCALE_CURRENT_A / ((float_t)pvar->USER_NUM_ISR_TICKS_PER_CTRL_TICK * pvar->USER_MOTOR_MAX_CURRENT)))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_delta_High);
        }

      if(pvar->USER_IDRATED_DELTA < 0.0)
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_IdRated_delta_Low);
        }
    }

  if(pvar->USER_MOTOR_FLUX_EST_FREQ_Hz > pvar->USER_IQ_FULL_SCALE_FREQ_Hz)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxEstFreq_Hz_High);
    }

  if((pvar->USER_MOTOR_FLUX_EST_FREQ_Hz < 0.0) ||
    (pvar->USER_MOTOR_FLUX_EST_FREQ_Hz < (pvar->USER_ZEROSPEEDLIMIT * pvar->USER_IQ_FULL_SCALE_FREQ_Hz)))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_fluxEstFreq_Hz_Low);
    }

  if(pvar->USER_MOTOR_Ls_d != 0.0)
    {
      if(((float_t)pvar->USER_CTRL_FREQ_Hz >= (128.0 * pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / (0.5 * (pvar->USER_MOTOR_Ls_d + 1e-9) * pvar->USER_IQ_FULL_SCALE_CURRENT_A))))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_High);
        }
    }

  if(pvar->USER_MOTOR_Ls_q != 0.0)
    {
      if(((float_t)pvar->USER_CTRL_FREQ_Hz >= (128.0 * pvar->USER_IQ_FULL_SCALE_VOLTAGE_V / (0.5 * (pvar->USER_MOTOR_Ls_q + 1e-9) * pvar->USER_IQ_FULL_SCALE_CURRENT_A))))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_High);
        }
    }

  if(((float_t)pvar->USER_CTRL_FREQ_Hz < pvar->USER_IQ_FULL_SCALE_FREQ_Hz) ||
    ((float_t)pvar->USER_CTRL_FREQ_Hz < pvar->USER_OFFSET_POLE_rps) ||
    ((float_t)pvar->USER_CTRL_FREQ_Hz < 250.0) ||
    ((float_t)pvar->USER_CTRL_FREQ_Hz <= (2.0 * pvar->USER_IQ_FULL_SCALE_FREQ_Hz * pvar->USER_MOTOR_MAX_CURRENT / (128.0 * pvar->USER_IQ_FULL_SCALE_CURRENT_A))))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_Low);
    }

  if((pvar->USER_MOTOR_Rs != 0.0) && (pvar->USER_MOTOR_Ls_d != 0.0) && (pvar->USER_MOTOR_Ls_q != 0.0))
    {
      if(((float_t)pvar->USER_CTRL_FREQ_Hz <= (pvar->USER_MOTOR_Rs / (pvar->USER_MOTOR_Ls_d + 1e-9))) ||
        ((float_t)pvar->USER_CTRL_FREQ_Hz <= (pvar->USER_MOTOR_Rs / (pvar->USER_MOTOR_Ls_q + 1e-9))))
        {
          USER_setErrorCode(pUserParams, USER_ErrorCode_ctrlFreq_Hz_Low);
        }
    }

  if(((float_t)pvar->USER_EST_FREQ_Hz < pvar->USER_FORCE_ANGLE_FREQ_Hz) ||
    ((float_t)pvar->USER_EST_FREQ_Hz < pvar->USER_VOLTAGE_FILTER_POLE_rps) ||
    ((float_t)pvar->USER_EST_FREQ_Hz < pvar->USER_DCBUS_POLE_rps) ||
    ((float_t)pvar->USER_EST_FREQ_Hz < pvar->USER_FLUX_POLE_rps) ||
    ((float_t)pvar->USER_EST_FREQ_Hz < pvar->USER_DIRECTION_POLE_rps) ||
    ((float_t)pvar->USER_EST_FREQ_Hz < pvar->USER_SPEED_POLE_rps) ||
    ((float_t)pvar->USER_EST_FREQ_Hz < 0.2))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_estFreq_Hz_Low);
    }

  if(pvar->USER_R_OVER_L_EST_FREQ_Hz > pvar->USER_IQ_FULL_SCALE_FREQ_Hz)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_RoverL_estFreq_Hz_High);
    }

  if(((float_t)pvar->USER_TRAJ_FREQ_Hz < 1.0) ||
    ((float_t)pvar->USER_TRAJ_FREQ_Hz < pvar->USER_MAX_ACCEL_Hzps / pvar->USER_IQ_FULL_SCALE_FREQ_Hz) ||
    ((float_t)pvar->USER_TRAJ_FREQ_Hz < pvar->USER_MAX_ACCEL_EST_Hzps / pvar->USER_IQ_FULL_SCALE_FREQ_Hz))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_trajFreq_Hz_Low);
    }

  if(pvar->USER_MAX_NEGATIVE_ID_REF_CURRENT_A > 0.0)
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxNegativeIdCurrent_a_High);
    }

  if(pvar->USER_MAX_NEGATIVE_ID_REF_CURRENT_A < (-pvar->USER_MOTOR_MAX_CURRENT))
    {
      USER_setErrorCode(pUserParams, USER_ErrorCode_maxNegativeIdCurrent_a_Low);
    }

  // Only for debug testing, only be commented
//  USER_setErrorCode(pUserParams, USER_ErrorCode_NoError);

  return;
} // end of USER_checkForErrors() function


USER_ErrorCode_e USER_getErrorCode(USER_Params *pUserParams)
{
  return(pUserParams->errorCode);
} // end of USER_getErrorCode() function


void USER_setErrorCode(USER_Params *pUserParams,const USER_ErrorCode_e errorCode)
{
  pUserParams->errorCode = errorCode;

  return;
} // end of USER_setErrorCode() function


void USER_softwareUpdate1p6(CTRL_Handle handle, USER_Var_t *pvar)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t fullScaleInductance = pvar->USER_IQ_FULL_SCALE_VOLTAGE_V/(pvar->USER_IQ_FULL_SCALE_CURRENT_A*pvar->USER_VOLTAGE_FILTER_POLE_rps);
  float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(obj->estHandle));
  int_least8_t lShift = ceil(log(obj->motorParams.Ls_d_H/(Ls_coarse_max*fullScaleInductance))/log(2.0));
  uint_least8_t Ls_qFmt = 30 - lShift;
  float_t L_max = fullScaleInductance * pow(2.0,lShift);
  _iq Ls_d_pu = _IQ30(obj->motorParams.Ls_d_H / L_max);
  _iq Ls_q_pu = _IQ30(obj->motorParams.Ls_q_H / L_max);


  // store the results
  EST_setLs_d_pu(obj->estHandle,Ls_d_pu);
  EST_setLs_q_pu(obj->estHandle,Ls_q_pu);
  EST_setLs_qFmt(obj->estHandle,Ls_qFmt);

  return;
} // end of softwareUpdate1p6() function


#ifndef NO_CTRL
void USER_calcPIgains(CTRL_Handle handle, USER_Var_t *pvar)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;
  float_t fullScaleCurrent = pvar->USER_IQ_FULL_SCALE_CURRENT_A;
  float_t fullScaleVoltage = pvar->USER_IQ_FULL_SCALE_VOLTAGE_V;
  float_t ctrlPeriod_sec = CTRL_getCtrlPeriod_sec(handle);
  float_t Ls_d;
  float_t Ls_q;
  float_t Rs;
  float_t RoverLs_d;
  float_t RoverLs_q;
  _iq Kp_Id;
  _iq Ki_Id;
  _iq Kp_Iq;
  _iq Ki_Iq;
  _iq Kd;

#ifdef __TMS320C28XX_FPU32__
  int32_t tmp;

  // when calling EST_ functions that return a float, and fpu32 is enabled, an integer is needed as a return
  // so that the compiler reads the returned value from the accumulator instead of fpu32 registers
  tmp = EST_getLs_d_H(obj->estHandle);
  Ls_d = *((float_t *)&tmp);

  tmp = EST_getLs_q_H(obj->estHandle);
  Ls_q = *((float_t *)&tmp);

  tmp = EST_getRs_Ohm(obj->estHandle);
  Rs = *((float_t *)&tmp);
#else
  Ls_d = EST_getLs_d_H(obj->estHandle);

  Ls_q = EST_getLs_q_H(obj->estHandle);

  Rs = EST_getRs_Ohm(obj->estHandle);
#endif

  RoverLs_d = Rs/Ls_d;
  Kp_Id = _IQ((0.25*Ls_d*fullScaleCurrent)/(ctrlPeriod_sec*fullScaleVoltage));
  Ki_Id = _IQ(RoverLs_d*ctrlPeriod_sec);

  RoverLs_q = Rs/Ls_q;
  Kp_Iq = _IQ((0.25*Ls_q*fullScaleCurrent)/(ctrlPeriod_sec*fullScaleVoltage));
  Ki_Iq = _IQ(RoverLs_q*ctrlPeriod_sec);

  Kd = _IQ(0.0);

  // set the Id controller gains
  PID_setKi(obj->pidHandle_Id,Ki_Id);
  CTRL_setGains(handle,CTRL_Type_PID_Id,Kp_Id,Ki_Id,Kd);

  // set the Iq controller gains
  PID_setKi(obj->pidHandle_Iq,Ki_Iq);
  CTRL_setGains(handle,CTRL_Type_PID_Iq,Kp_Iq,Ki_Iq,Kd);

  return;
} // end of calcPIgains() function
#endif


//! \brief     Computes the scale factor needed to convert from torque created by Ld, Lq, Id and Iq, from per unit to Nm
//!
_iq USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(USER_Var_t *pvar)
{
  float_t FullScaleInductance = (pvar->USER_IQ_FULL_SCALE_VOLTAGE_V/(pvar->USER_IQ_FULL_SCALE_CURRENT_A*pvar->USER_VOLTAGE_FILTER_POLE_rps));
  float_t FullScaleCurrent = (pvar->USER_IQ_FULL_SCALE_CURRENT_A);
  float_t lShift = ceil(log(pvar->USER_MOTOR_Ls_d/(0.7*FullScaleInductance))/log(2.0));

  return(_IQ(FullScaleInductance*FullScaleCurrent*FullScaleCurrent*pvar->USER_MOTOR_NUM_POLE_PAIRS*1.5*pow(2.0,lShift)));
} // end of USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf() function


//! \brief     Computes the scale factor needed to convert from torque created by flux and Iq, from per unit to Nm
//!
_iq USER_computeTorque_Flux_Iq_pu_to_Nm_sf(USER_Var_t *pvar)
{
  float_t FullScaleFlux = (pvar->USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)pvar->USER_EST_FREQ_Hz);
  float_t FullScaleCurrent = (pvar->USER_IQ_FULL_SCALE_CURRENT_A);
  float_t maxFlux = (pvar->USER_MOTOR_RATED_FLUX*((pvar->USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
  float_t lShift = -ceil(log(FullScaleFlux/maxFlux)/log(2.0));

  return(_IQ(FullScaleFlux/(2.0*MATH_PI)*FullScaleCurrent*pvar->USER_MOTOR_NUM_POLE_PAIRS*1.5*pow(2.0,lShift)));
} // end of USER_computeTorque_Flux_Iq_pu_to_Nm_sf() function


//! \brief     Computes the scale factor needed to convert from per unit to Wb
//!
_iq USER_computeFlux_pu_to_Wb_sf(USER_Var_t *pvar)
{
  float_t FullScaleFlux = (pvar->USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)pvar->USER_EST_FREQ_Hz);
  float_t maxFlux = (pvar->USER_MOTOR_RATED_FLUX*((pvar->USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
  float_t lShift = -ceil(log(FullScaleFlux/maxFlux)/log(2.0));

  return(_IQ(FullScaleFlux/(2.0*MATH_PI)*pow(2.0,lShift)));
} // end of USER_computeFlux_pu_to_Wb_sf() function


//! \brief     Computes the scale factor needed to convert from per unit to V/Hz
//!
_iq USER_computeFlux_pu_to_VpHz_sf(USER_Var_t *pvar)
{
  float_t FullScaleFlux = (pvar->USER_IQ_FULL_SCALE_VOLTAGE_V/(float_t)pvar->USER_EST_FREQ_Hz);
  float_t maxFlux = (pvar->USER_MOTOR_RATED_FLUX*((pvar->USER_MOTOR_TYPE==MOTOR_Type_Induction)?0.05:0.7));
  float_t lShift = -ceil(log(FullScaleFlux/maxFlux)/log(2.0));

  return(_IQ(FullScaleFlux*pow(2.0,lShift)));
} // end of USER_computeFlux_pu_to_VpHz_sf() function


//! \brief     Computes Flux in Wb or V/Hz depending on the scale factor sent as parameter
//!
_iq USER_computeFlux(CTRL_Handle handle, const _iq sf)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  return(_IQmpy(EST_getFlux_pu(obj->estHandle),sf));
} // end of USER_computeFlux() function


//! \brief     Computes Torque in Nm
//!
_iq USER_computeTorque_Nm(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  _iq Flux_pu = EST_getFlux_pu(obj->estHandle);
  _iq Id_pu = PID_getFbackValue(obj->pidHandle_Id);
  _iq Iq_pu = PID_getFbackValue(obj->pidHandle_Iq);
  _iq Ld_minus_Lq_pu = _IQ30toIQ(EST_getLs_d_pu(obj->estHandle)-EST_getLs_q_pu(obj->estHandle));
  _iq Torque_Flux_Iq_Nm = _IQmpy(_IQmpy(Flux_pu,Iq_pu),torque_Flux_sf);
  _iq Torque_Ls_Id_Iq_Nm = _IQmpy(_IQmpy(_IQmpy(Ld_minus_Lq_pu,Id_pu),Iq_pu),torque_Ls_sf);
  _iq Torque_Nm = Torque_Flux_Iq_Nm + Torque_Ls_Id_Iq_Nm;

  return(Torque_Nm);
} // end of USER_computeTorque_Nm() function


//! \brief     Computes Torque in Nm
//!
_iq USER_computeTorque_lbin(CTRL_Handle handle, const _iq torque_Flux_sf, const _iq torque_Ls_sf)
{
  CTRL_Obj *obj = (CTRL_Obj *)handle;

  _iq Flux_pu = EST_getFlux_pu(obj->estHandle);
  _iq Id_pu = PID_getFbackValue(obj->pidHandle_Id);
  _iq Iq_pu = PID_getFbackValue(obj->pidHandle_Iq);
  _iq Ld_minus_Lq_pu = _IQ30toIQ(EST_getLs_d_pu(obj->estHandle)-EST_getLs_q_pu(obj->estHandle));
  _iq Torque_Flux_Iq_Nm = _IQmpy(_IQmpy(Flux_pu,Iq_pu),torque_Flux_sf);
  _iq Torque_Ls_Id_Iq_Nm = _IQmpy(_IQmpy(_IQmpy(Ld_minus_Lq_pu,Id_pu),Iq_pu),torque_Ls_sf);
  _iq Torque_Nm = Torque_Flux_Iq_Nm + Torque_Ls_Id_Iq_Nm;

  return(_IQmpy(Torque_Nm, _IQ(MATH_Nm_TO_lbin_SF)));
} // end of USER_computeTorque_lbin() function


// end of file

