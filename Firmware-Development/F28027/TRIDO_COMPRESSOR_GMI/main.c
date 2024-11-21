/******************************************************************************************************
 *   Project: Compressor Application
 *   Version: COMPRESSOR_V1.3
 *   Modified: 09/03/2023
 *******************************************************************************************************/

// **************************************************************************
// the includes

// modbus include
#include "main.h"
#include "userio.h"
#include "modbus_user.h"

// system includes
#include <math.h>

#ifdef FLASH
#pragma CODE_SECTION(mainISR,"ramfuncs");
#endif

// Include header files used in the main function
#include "include/config.h"
#if SYSTEMTASK_ENABLE == 1
#include "include/system.h"
#endif

// **************************************************************************
// the function prototypes
void updateIdIqRef(CTRL_Handle handle);

// **************************************************************************
// the defines

// **************************************************************************
// the globals

uint_least16_t gCounter_updateGlobals = 0;

bool Flag_Latch_softwareUpdate = true;

CTRL_Handle ctrlHandle;

#ifdef CSM_ENABLE
#pragma DATA_SECTION(halHandle,"rom_accessed_data");
#endif

HAL_Handle halHandle;

#ifdef CSM_ENABLE
#pragma DATA_SECTION(gUserParams,"rom_accessed_data");
#endif

USER_Params gUserParams;

HAL_PwmData_t gPwmData = { _IQ(0.0), _IQ(0.0), _IQ(0.0) };

HAL_AdcData_t gAdcData;

MATH_vec3 gAdcBiasI = {_IQ(I_A_offset), _IQ(I_A_offset), _IQ(I_A_offset)};
MATH_vec3 gAdcBiasV = {_IQ(0.0), _IQ(0.0), _IQ(0.0)};

_iq gMaxCurrentSlope = _IQ(0.0);

#ifdef FAST_ROM_V1p6
CTRL_Obj *controller_obj;
#else

#ifdef CSM_ENABLE
#pragma DATA_SECTION(ctrl,"rom_accessed_data");
#endif

CTRL_Obj ctrl; //v1p7 format
#endif

volatile MOTOR_Vars_t gMotorVars = MOTOR_Vars_INIT;

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
        extern uint16_t *RamfuncsLoadStart,
*RamfuncsLoadEnd, *RamfuncsRunStart;

#ifdef CSM_ENABLE
extern uint16_t *econst_start, *econst_end, *econst_ram_load;
extern uint16_t *switch_start, *switch_end, *switch_ram_load;
#endif
#endif

#if SVGENCURRENT_ENABLE == 1
SVGENCURRENT_Obj svgencurrent;
SVGENCURRENT_Handle svgencurrentHandle;
#endif

// set the offset, default value of 1 microsecond
int16_t gCmpOffset = (int16_t) (1.0 * USER_SYSTEM_FREQ_MHz);

MATH_vec3 gIavg = { _IQ(0.0), _IQ(0.0), _IQ(0.0) };
uint16_t gIavg_shift = 1;
MATH_vec3 gPwmData_prev = { _IQ(0.0), _IQ(0.0), _IQ(0.0) };

_iq gFlux_pu_to_Wb_sf;

_iq gFlux_pu_to_VpHz_sf;

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf;

_iq gTorque_Flux_Iq_pu_to_Nm_sf;

bool isrTick = false;

_iq voltageOffsetUnBias_pu;

// **************************************************************************
// the functions

// assembly file
extern void usDelay(uint32_t Count);

void gMotorVars_init()
{
    gMotorVars.Flag_enableSys = false;
    gMotorVars.Flag_Run_Identify = false;
    gMotorVars.Flag_MotorIdentified = false;
    gMotorVars.Flag_enableForceAngle = true;
    gMotorVars.Flag_enableFieldWeakening = false;
    gMotorVars.Flag_enableRsRecalc = false;
    gMotorVars.Flag_enableUserParams = true;
    gMotorVars.Flag_enableOffsetcalc = true;
    gMotorVars.Flag_enablePowerWarp = false;
    gMotorVars.Flag_enableSpeedCtrl = false;

    gMotorVars.Flag_enableRun = false;
    gMotorVars.Flag_RunState = false;
    gMotorVars.Flag_enableFlyingStart = false;

    gMotorVars.CtrlState = CTRL_State_Idle;
    gMotorVars.EstState = EST_State_Idle;

    gMotorVars.UserErrorCode = USER_ErrorCode_NoError;

    gMotorVars.CtrlVersion.rsvd = 0;
    gMotorVars.CtrlVersion.targetProc = CTRL_TargetProc_Unknown;
    gMotorVars.CtrlVersion.major = 0;
    gMotorVars.CtrlVersion.minor = 0;

    gMotorVars.SpeedRef_pu = _IQ(0.0);
    gMotorVars.SpeedRef_krpm = _IQ(0.1);
    gMotorVars.SpeedTraj_krpm = _IQ(0.0);
    gMotorVars.MaxAccel_krpmps = _IQ(0.2);
    gMotorVars.Speed_krpm = _IQ(0.0);
    gMotorVars.OverModulation = _IQ(USER_MAX_VS_MAG_PU);
    gMotorVars.RsOnLineCurrent_A = _IQ(0.1 * USER_MOTOR_MAX_CURRENT);
    gMotorVars.SvgenMaxModulation_ticks = 400;
    gMotorVars.Flux_Wb = _IQ(0.0);
    gMotorVars.Torque_Nm = _IQ(0.0);

    gMotorVars.MagnCurr_A = 0.0;
    gMotorVars.Rr_Ohm = 0.0;
    gMotorVars.Rs_Ohm = 0.0;
    gMotorVars.RsOnLine_Ohm = 0.0;
    gMotorVars.Lsd_H = 0.0;
    gMotorVars.Lsq_H = 0.0;
    gMotorVars.Flux_VpHz = 0.0;

    gMotorVars.ipd_excFreq_Hz = 0.0;
    gMotorVars.ipd_Kspd = _IQ(0.0);
    gMotorVars.ipd_excMag_coarse_pu = _IQ(0.0);
    gMotorVars.ipd_excMag_fine_pu = _IQ(0.0);
    gMotorVars.ipd_waitTime_coarse_sec = 0.0;
    gMotorVars.ipd_waitTime_fine_sec = 0.0;

    gMotorVars.Kp_spd = _IQ(0.0);
    gMotorVars.Ki_spd = _IQ(0.0);

    gMotorVars.Kp_Idq = _IQ(0.0);
    gMotorVars.Ki_Idq = _IQ(0.0);

    gMotorVars.Vd = _IQ(0.0);
    gMotorVars.Vq = _IQ(0.0);
    gMotorVars.Vs = _IQ(0.0);
    gMotorVars.VsRef = _IQ(0.8 * USER_MAX_VS_MAG_PU);
    gMotorVars.VdcBus_kV = _IQ(0.0);

//    gMotorVars.Id_A = _IQ(0.0);
//    gMotorVars.Iq_A = _IQ(0.0);
//    gMotorVars.Is_A = _IQ(0.0);

    gMotorVars.I_bias.value[0] = 0;
    gMotorVars.I_bias.value[1] = 0;
    gMotorVars.I_bias.value[2] = 0;
    gMotorVars.V_bias.value[0] = 0;
    gMotorVars.V_bias.value[1] = 0;
    gMotorVars.V_bias.value[2] = 0;

    gMotorVars.SpeedSet_krpm = _IQ(0.6);

    gMotorVars.angle_sen_pu = _IQ(0.0);
    gMotorVars.angle_est_pu = _IQ(0.0);
    gMotorVars.speed_sen_pu = _IQ(0.0);
    gMotorVars.speed_est_pu = _IQ(0.0);

    gMotorVars.speedHigh_hall2fast_pu = _IQ(0.0);
    gMotorVars.speedLow_hall2fast_pu = _IQ(0.0);
    gMotorVars.IdSet_A = _IQ(0.0);
    gMotorVars.IqSet_A = _IQ(0.0);
    gMotorVars.IdRef_pu = _IQ(0.0);
    gMotorVars.IqRef_pu = _IQ(0.0);
}

void main(void)
{
    uint_least8_t estNumber = 0;
    uint_least8_t cnt;

//  gMotorVars_init();

#ifdef FAST_ROM_V1p6
  uint_least8_t ctrlNumber = 0;
#endif

    // Only used if running from FLASH
    // Note that the variable FLASH is defined by the project
#ifdef FLASH
    // Copy time critical code and Flash setup code to RAM
    // The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
    memCopy((uint16_t*) &RamfuncsLoadStart, (uint16_t*) &RamfuncsLoadEnd, (uint16_t*) &RamfuncsRunStart);

#ifdef CSM_ENABLE
  //copy .econst to unsecure RAM
  if(*econst_end - *econst_start)
  {
     memCopy((uint16_t *)&econst_start,(uint16_t *)&econst_end,(uint16_t *)&econst_ram_load);
  }

  //copy .switch ot unsecure RAM
  if(*switch_end - *switch_start)
  {
    memCopy((uint16_t *)&switch_start,(uint16_t *)&switch_end,(uint16_t *)&switch_ram_load);
  }
  #endif
#endif

    usDelay(300);

    // initialize the hardware abstraction layer
    halHandle = HAL_init(&hal, sizeof(hal));

    // check for errors in user parameters
    USER_checkForErrors(&gUserParams);

    // store user parameter error in global variable
    gMotorVars.UserErrorCode = USER_getErrorCode(&gUserParams);

    // do not allow code execution if there is a user parameter error
    if (gMotorVars.UserErrorCode != USER_ErrorCode_NoError)
    {
        for (;;)
        {
            gMotorVars.Flag_enableSys = false;
        }
    }

    // initialize the user parameters
    USER_setParams(&gUserParams);

    // set the hardware abstraction layer parameters
    HAL_setParams(halHandle, &gUserParams);

    // initialize the controller
#ifdef FAST_ROM_V1p6
  ctrlHandle = CTRL_initCtrl(ctrlNumber, estNumber);//v1p6 format (06xF and 06xM devices)
  controller_obj = (CTRL_Obj *)ctrlHandle;
#else
    ctrlHandle = CTRL_initCtrl(estNumber, &ctrl, sizeof(ctrl));  //v1p7 format default
#endif

    {
        CTRL_Version version;

        // get the version number
        CTRL_getVersion(ctrlHandle, &version);

        gMotorVars.CtrlVersion = version;
    }

    // set the default controller parameters
    CTRL_setParams(ctrlHandle, &gUserParams);

#if SVGENCURRENT_ENABLE == 1
    // Initialize and setup the 100% SVM generator
    svgencurrentHandle = SVGENCURRENT_init(&svgencurrent, sizeof(svgencurrent));

    // setup svgen current
    {
        float_t minWidth_microseconds = 2.0;  //0.5-(2*2ms*8kHz*0.001)=0.468
        uint16_t minWidth_counts = (uint16_t) (minWidth_microseconds * USER_SYSTEM_FREQ_MHz);
        float_t fdutyLimit = 0.5 - (2.0 * minWidth_microseconds * USER_PWM_FREQ_kHz * 0.001);  //pwm duty limit 0.5 to -0.5
        _iq dutyLimit = _IQ(fdutyLimit);

        SVGENCURRENT_setMinWidth(svgencurrentHandle, minWidth_counts);
        SVGENCURRENT_setIgnoreShunt(svgencurrentHandle, use_all);
        SVGENCURRENT_setMode(svgencurrentHandle, all_phase_measurable);
        SVGENCURRENT_setVlimit(svgencurrentHandle, dutyLimit);
    }
#endif

    // setup faults, pwm trip zone
    HAL_setupFaults(halHandle);

    // initialize the interrupt vector table
    HAL_initIntVectorTable(halHandle);

    // enable the ADC interrupts
    HAL_enableAdcInts(halHandle);

#if SYSTEMTASK_ENABLE == 1
  //user initialization before enabling interrupt
  sys_init(&sys, halHandle, ctrlHandle, &gUserParams, &gMotorVars, &gAdcData, &gPwmData);
#endif

    // enable global interrupts
    HAL_enableGlobalInts(halHandle);

    // enable debug interrupts
    HAL_enableDebugInt(halHandle);

    // disable the PWM
    HAL_disablePwm(halHandle);

    //Set the maximum current controller output for the Iq and Id current controllers to enable
    //over-modulation.
    //An input into the SVM above 1/SQRT(3) = 0.5774 is in the over-modulation region.  An input of 0.5774 is where
    //the crest of the sinewave touches the 100% duty cycle.  At an input of 2/3, the SVM generator
    //produces a trapezoidal waveform touching every corner of the hexagon
    CTRL_setMaxVsMag_pu(ctrlHandle, gMotorVars.OverModulation = _IQ(0.5774));

    // enable DC bus compensation
    CTRL_setFlag_enableDcBusComp(ctrlHandle, true);
    // enable or disable power warp
    CTRL_setFlag_enablePowerWarp(ctrlHandle, gMotorVars.Flag_enablePowerWarp = false);
    // Enable the Library internal PI.  Iq is referenced by the speed PI now
    CTRL_setFlag_enableSpeedCtrl(ctrlHandle, gMotorVars.Flag_enableSpeedCtrl = true);
    // enable/disable the use of motor parameters being loaded from user.h
    CTRL_setFlag_enableUserMotorParams(ctrlHandle, gMotorVars.Flag_enableUserParams = true);
    // enable/disable Rs recalibration during motor startup
    EST_setFlag_enableRsRecalc(ctrlHandle->estHandle, gMotorVars.Flag_enableRsRecalc = false);
#if FLYING_START_ENABLE == 1
  // Enable the Library internal PI.  Iq is referenced by the speed PI now
  CTRL_setFlag_enableSpeedCtrl(ctrlHandle, gMotorVars.Flag_enableSpeedCtrl = false);
  // enable/disable automatic calculation of bias values
  CTRL_setFlag_enableOffset(ctrlHandle,gMotorVars.Flag_enableOffsetcalc = false);
  // enable/disable the forced angle
  EST_setFlag_enableForceAngle(ctrlHandle->estHandle,gMotorVars.Flag_enableForceAngle = false);
#else
    // enable/disable automatic calculation of bias values
    CTRL_setFlag_enableOffset(ctrlHandle, gMotorVars.Flag_enableOffsetcalc = true);
    // enable/disable the forced angle
    EST_setFlag_enableForceAngle(ctrlHandle->estHandle, gMotorVars.Flag_enableForceAngle = true);
#endif

    // compute scaling factors for flux and torque calculations
    gFlux_pu_to_Wb_sf = USER_computeFlux_pu_to_Wb_sf();
    gFlux_pu_to_VpHz_sf = USER_computeFlux_pu_to_VpHz_sf();
    gTorque_Ls_Id_Iq_pu_to_Nm_sf = USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf();
    gTorque_Flux_Iq_pu_to_Nm_sf = USER_computeTorque_Flux_Iq_pu_to_Nm_sf();

    // Below two lines code for Flash Testing, need to be commented
    gMotorVars.Flag_enableSys = true;
    gMotorVars.SpeedRef_krpm = 0;
    gMotorVars.MaxAccel_krpmps = _IQ(80.0 / 1000.0);

    // Not used but to initialize to known value
    for (cnt = 0; cnt < 3; cnt++)
    {
        halHandle->offset_V[cnt].bias = 0;
        halHandle->offset_I[cnt].bias = 0;
    }

    halHandle->adcBias.dcBus = 0;

    InitGpios();
    modbus_setup();

    //Set WDOG prescaler
    HAL_WDOG_setPreScaler(halHandle, WDOG_PreScaler_OscClk_by_512_by_64);

    // Reset the watchdog counter
    HAL_kickWdog(halHandle);

    // enable watchdog
    HAL_enableWdog(halHandle);

    for (;;)
    {
        // Waiting for enable system flag to be set
        while (!(gMotorVars.Flag_enableSys))
        {
            readIOstates();
            modbus_loop(); // enable modbus communication
            HAL_kickWdog(halHandle); // Reset the watchdog counter
        }

#if SYSTEMTASK_ENABLE == 1
    sys_loop(&sys);
    if(isrTick){
        sys_isrFollower(&sys);
        isrTick = false;
    }
#endif

        // loop while the enable system flag is true
        while (gMotorVars.Flag_enableSys)
        {
            CTRL_Obj *obj = (CTRL_Obj*) ctrlHandle;

            // increment counters
            gCounter_updateGlobals++;

#if SYSTEMTASK_ENABLE == 1
        sys_loop(&sys);
        if(isrTick){
            sys_isrFollower(&sys);
            isrTick = false;
        }
#endif

            if (CTRL_isError(ctrlHandle))
            {
                error.bit.esterr = true;

                // set the enable controller flag to false
                CTRL_setFlag_enableCtrl(ctrlHandle, false);

                // set the enable system flag to false
                gMotorVars.Flag_enableSys = false;

                // disable the PWM
                HAL_disablePwm(halHandle);
            }
            else
            {
                // update the controller state
                bool flag_ctrlStateChanged = CTRL_updateState(ctrlHandle);

                // enable or disable the control
                CTRL_setFlag_enableCtrl(ctrlHandle, gMotorVars.Flag_Run_Identify);

                if (flag_ctrlStateChanged)
                {
                    CTRL_State_e ctrlState = CTRL_getState(ctrlHandle);

                    if (ctrlState == CTRL_State_OffLine)
                    {
#if (FLYING_START_ENABLE == 0 && SYS_PWMFIRE_ENABLE == 1)
                        // enable the PWM
                        HAL_enablePwm(halHandle);
#endif
                    }
                    else if (ctrlState == CTRL_State_OnLine)
                    {
                        error.bit.esterr = false;

                        // offset calibration
                        if (gMotorVars.Flag_enableOffsetcalc)
                        {
                            for (cnt = 0; cnt < HAL_getNumVoltageSensors(halHandle); cnt++)
                            {
                                gAdcBiasV.value[cnt] += OFFSET_getOffset(halHandle->offsetHandle_V[cnt]);
                                HAL_setBias(halHandle, HAL_SensorType_Voltage, cnt, gAdcBiasV.value[cnt]);
                            }
                            for (cnt = 0; cnt < HAL_getNumCurrentSensors(halHandle); cnt++)
                            {
                                gAdcBiasI.value[cnt] += OFFSET_getOffset(halHandle->offsetHandle_I[cnt]);
                                HAL_setBias(halHandle, HAL_SensorType_Current, cnt, gAdcBiasI.value[cnt]);
                            }
                        }
                        else //set the voltage and current measurement offsets from user_*.h
                        {
                            // set the voltage bias to default
                            HAL_setBias(halHandle, HAL_SensorType_Voltage, 0, _IQ(V_A_offset));
                            HAL_setBias(halHandle, HAL_SensorType_Voltage, 1, _IQ(V_B_offset));
                            HAL_setBias(halHandle, HAL_SensorType_Voltage, 2, _IQ(V_C_offset));

                            // set the current bias to default
                            HAL_setBias(halHandle, HAL_SensorType_Current, 0, _IQ(I_A_offset));
                            HAL_setBias(halHandle, HAL_SensorType_Current, 1, _IQ(I_B_offset));
                            HAL_setBias(halHandle, HAL_SensorType_Current, 2, _IQ(I_C_offset));
                        }

#if (FLYING_START_ENABLE == 0 && SYS_PWMFIRE_ENABLE == 1)
                        // enable the PWM
                        HAL_enablePwm(halHandle);
#endif
                    }
                    else if (ctrlState == CTRL_State_Idle)
                    {
                        // disable the PWM
                        HAL_disablePwm(halHandle);
                        gMotorVars.Flag_Run_Identify = false;
                    }

                    if ((CTRL_getFlag_enableUserMotorParams(ctrlHandle) == true)
                            && (ctrlState > CTRL_State_Idle)
                            && (gMotorVars.CtrlVersion.minor == 6))
                    {
                        // call this function to fix 1p6
                        USER_softwareUpdate1p6(ctrlHandle);
                    }
                }
            }

            if (EST_isMotorIdentified(obj->estHandle))
            {
//            _iq Id_squared_pu = _IQmpy(CTRL_getId_ref_pu(ctrlHandle),CTRL_getId_ref_pu(ctrlHandle));

                // set the current ramp
                EST_setMaxCurrentSlope_pu(obj->estHandle, gMaxCurrentSlope);
                gMotorVars.Flag_MotorIdentified = true;

                // set the speed acceleration
                CTRL_setMaxAccel_pu(ctrlHandle, _IQmpy(MAX_ACCEL_KRPMPS_SF, gMotorVars.MaxAccel_krpmps));

                if (Flag_Latch_softwareUpdate)
                {
                    Flag_Latch_softwareUpdate = false;

                    USER_calcPIgains(ctrlHandle);

                    // initialize the watch window kp and ki current values with pre-calculated values
                    gMotorVars.Kp_Idq = CTRL_getKp(ctrlHandle, CTRL_Type_PID_Id);
                    gMotorVars.Ki_Idq = CTRL_getKi(ctrlHandle, CTRL_Type_PID_Id);
                }
            }
            else
            {
                Flag_Latch_softwareUpdate = true;

                // initialize the watch window kp and ki values with pre-calculated values
                gMotorVars.Kp_spd = CTRL_getKp(ctrlHandle, CTRL_Type_PID_spd);
                gMotorVars.Ki_spd = CTRL_getKi(ctrlHandle, CTRL_Type_PID_spd);

                // the estimator sets the maximum current slope during identification
                gMaxCurrentSlope = EST_getMaxCurrentSlope_pu(obj->estHandle);
            }

            // when appropriate, update the global variables
            if (gCounter_updateGlobals >= NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE)
            {
                // reset the counter
                gCounter_updateGlobals = 0;

                updateGlobalVariables_motor(ctrlHandle);
            }

#if GUICONTROL_ENABLE == 1
            if (gMotorVars.Flag_enableSpeedCtrl)
            {
                // set the speed reference
                CTRL_setSpd_ref_krpm(ctrlHandle, gMotorVars.SpeedRef_krpm);
                gMotorVars.IdRef_pu = _IQmpy(gMotorVars.IdRef_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));
            }
            else
            {
                // update Iq reference
                gMotorVars.IqRef_pu = _IQmpy(gMotorVars.IqRef_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));
                gMotorVars.IdRef_pu = _IQmpy(gMotorVars.IdRef_A, _IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));
                updateIdIqRef(ctrlHandle);
            }
#endif

            //default calculation
            //gMotorVars.Kp_Idq = _IQ(0.244);
            //gMotorVars.Ki_Idq = _IQ(0.0703766942);

            //manual tunning
            //gMotorVars.Kp_Idq = _IQ(2.0);
            //gMotorVars.Ki_Idq = _IQ(0.0703766942);
            // update Kp and Ki gains from watch window
            updateKpKiGains(ctrlHandle);

            readIOstates();
            modbus_loop(); // enable modbus communication
            HAL_kickWdog(halHandle); // Reset the watchdog counter
        } // end of while(gFlag_enableSys) loop

        // disable the PWM
        HAL_disablePwm(halHandle);

        // set the default controller parameters (Reset the control to re-identify the motor)
        CTRL_setParams(ctrlHandle, &gUserParams);
        gMotorVars.Flag_Run_Identify = false;

        readIOstates();
        modbus_loop(); // enable modbus communication when motor off
        HAL_kickWdog(halHandle); // Reset the watchdog counter
    } // end of for(;;) loop

} // end of main() function

interrupt void mainISR(void)
{
#if SVGENCURRENT_ENABLE == 1
#if __TMS320C28XX_FPU32__ == 1
  SVGENCURRENT_IgnoreShunt_e ignoreShuntThisCycle = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle);
#else
    SVGENCURRENT_MeasureShunt_e measurableShuntThisCycle = SVGENCURRENT_getMode(svgencurrentHandle);
#endif
#endif

    // acknowledge the ADC interrupt
    HAL_acqAdcInt(halHandle, ADC_IntNumber_1);

    // convert the ADC data
    HAL_readAdcData(halHandle, &gAdcData);

#if SVGENCURRENT_ENABLE == 1
#if __TMS320C28XX_FPU32__ == 1
  // run the current reconstruction algorithm
  SVGENCURRENT_RunRegenCurrent(svgencurrentHandle, (MATH_vec3 *)(gAdcData.I.value));

  gIavg.value[0] += (gAdcData.I.value[0] - gIavg.value[0])>>gIavg_shift;
  gIavg.value[1] += (gAdcData.I.value[1] - gIavg.value[1])>>gIavg_shift;
  gIavg.value[2] += (gAdcData.I.value[2] - gIavg.value[2])>>gIavg_shift;

  if(ignoreShuntThisCycle > use_all)
  {
      gAdcData.I.value[0] = gIavg.value[0];
      gAdcData.I.value[1] = gIavg.value[1];
      gAdcData.I.value[2] = gIavg.value[2];
  }
#else
    // run the current reconstruction algorithm
    SVGENCURRENT_RunRegenCurrent(svgencurrentHandle, (MATH_vec3*) (gAdcData.I.value));

    gIavg.value[0] += (gAdcData.I.value[0] - gIavg.value[0]) >> gIavg_shift;
    gIavg.value[1] += (gAdcData.I.value[1] - gIavg.value[1]) >> gIavg_shift;
    gIavg.value[2] += (gAdcData.I.value[2] - gIavg.value[2]) >> gIavg_shift;

    if (measurableShuntThisCycle > two_phase_measurable)
    {
        gAdcData.I.value[0] = gIavg.value[0];
        gAdcData.I.value[1] = gIavg.value[1];
        gAdcData.I.value[2] = gIavg.value[2];
    }
#endif
#endif

        // run the controller
        CTRL_run(ctrlHandle, halHandle, &gAdcData, &gPwmData); //comment to test 50% pwm

#if SVGENCURRENT_ENABLE == 1
        // run the PWM compensation and current ignore algorithm
        SVGENCURRENT_compPwmData(svgencurrentHandle, &(gPwmData.Tabc), &gPwmData_prev);
#endif

    // write the PWM compare values
    HAL_writePwmData(halHandle, &gPwmData); //halHandle->pwmHandle[PWM_Number_1]->TZCLR. PWM_TZCLR_OST_BITS is located in bit 2 (0b100)

#if SVGENCURRENT_ENABLE == 1
#if __TMS320C28XX_FPU32__ == 1
  {
    SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle);
    SVGENCURRENT_VmidShunt_e midVolShunt = SVGENCURRENT_getVmid(svgencurrentHandle);

    // Set trigger point in the middle of the low side pulse
    HAL_setTrigger(halHandle,ignoreShuntNextCycle,midVolShunt);
  }
#else
    {
        SVGENCURRENT_IgnoreShunt_e ignoreShuntNextCycle = SVGENCURRENT_getIgnoreShunt(svgencurrentHandle);
        SVGENCURRENT_VmidShunt_e midVolShunt = SVGENCURRENT_getVmid(svgencurrentHandle);

        // Set trigger point in the middle of the low side pulse
        HAL_setTrigger(halHandle, ignoreShuntNextCycle, midVolShunt);
    }
#endif
#endif

    // setup the controller
    CTRL_setup(ctrlHandle);

#if SYSTEMTASK_ENABLE == 1
  sys_isr(&sys);
  isrTick = true;
#endif

    return;
} // end of mainISR() function

void updateGlobalVariables_motor(CTRL_Handle handle)
{
    CTRL_Obj *obj = (CTRL_Obj*) handle;
#if __TMS320C28XX_FPU32__ == 1
  int32_t tmp;
  #endif

    // get the speed estimate
    gMotorVars.Speed_krpm = EST_getSpeed_krpm(obj->estHandle);

    // get the real time speed reference coming out of the speed trajectory generator
    gMotorVars.SpeedTraj_krpm = _IQmpy(CTRL_getSpd_int_ref_pu(handle), EST_get_pu_to_krpm_sf(obj->estHandle));

    // get the torque estimate
    gMotorVars.Torque_Nm = USER_computeTorque_Nm(handle, gTorque_Flux_Iq_pu_to_Nm_sf, gTorque_Ls_Id_Iq_pu_to_Nm_sf);

    // when calling EST_ functions that return a float, and fpu32 is enabled, an integer is needed as a return
    // so that the compiler reads the returned value from the accumulator instead of fpu32 registers
    // get the magnetizing current
#if __TMS320C28XX_FPU32__ == 1
  tmp = EST_getIdRated(obj->estHandle);
  gMotorVars.MagnCurr_A = *((float_t *)&tmp);
  #else
    gMotorVars.MagnCurr_A = EST_getIdRated(obj->estHandle);
#endif

    // get the rotor resistance
#if __TMS320C28XX_FPU32__ == 1
  tmp = EST_getRr_Ohm(obj->estHandle);
  gMotorVars.Rr_Ohm = *((float_t *)&tmp);
  #else
    gMotorVars.Rr_Ohm = EST_getRr_Ohm(obj->estHandle);
#endif

    // get the stator resistance
#if __TMS320C28XX_FPU32__ == 1
  tmp = EST_getRs_Ohm(obj->estHandle);
  gMotorVars.Rs_Ohm = *((float_t *)&tmp);
  #else
    gMotorVars.Rs_Ohm = EST_getRs_Ohm(obj->estHandle);
#endif

    // get the stator inductance in the direct coordinate direction
#if __TMS320C28XX_FPU32__ == 1
  tmp = EST_getLs_d_H(obj->estHandle);
  gMotorVars.Lsd_H = *((float_t *)&tmp);
  #else
    gMotorVars.Lsd_H = EST_getLs_d_H(obj->estHandle);
#endif

    // get the stator inductance in the quadrature coordinate direction
#if __TMS320C28XX_FPU32__ == 1
  tmp = EST_getLs_q_H(obj->estHandle);
  gMotorVars.Lsq_H = *((float_t *)&tmp);
  #else
    gMotorVars.Lsq_H = EST_getLs_q_H(obj->estHandle);
#endif

    // get the flux in V/Hz in floating point
#if __TMS320C28XX_FPU32__ == 1
  tmp = EST_getFlux_VpHz(obj->estHandle);
  gMotorVars.Flux_VpHz = *((float_t *)&tmp);
  #else
    gMotorVars.Flux_VpHz = EST_getFlux_VpHz(obj->estHandle);
#endif

    // get the flux in Wb in fixed point
    gMotorVars.Flux_Wb = USER_computeFlux(handle, gFlux_pu_to_Wb_sf);

    // get the controller state
    gMotorVars.CtrlState = CTRL_getState(handle);

    // get the estimator state
    gMotorVars.EstState = EST_getState(obj->estHandle);

    // read Vd and Vq vectors per units
    gMotorVars.Vd = CTRL_getVd_out_pu(ctrlHandle);
    gMotorVars.Vq = CTRL_getVq_out_pu(ctrlHandle);

    // calculate vector Vs in per units
    gMotorVars.Vs = _IQsqrt(_IQmpy(gMotorVars.Vd, gMotorVars.Vd) + _IQmpy(gMotorVars.Vq, gMotorVars.Vq));

    // read Id and Iq vectors in amps
    gMotorVars.Id_pu = CTRL_getId_in_pu(ctrlHandle);
    gMotorVars.Iq_pu = CTRL_getIq_in_pu(ctrlHandle);

    // calculate vector Is in amps, Is = sqrt (Iq^2 + Id^2)
    gMotorVars.Is_pu = _IQsqrt(_IQmpy(gMotorVars.Id_pu, gMotorVars.Id_pu) + _IQmpy(gMotorVars.Iq_pu, gMotorVars.Iq_pu));

    // for GUI showing correctly
    gMotorVars.Id_A = gMotorVars.Id_pu;
    gMotorVars.Iq_A = gMotorVars.Iq_pu;
    gMotorVars.Is_A = gMotorVars.Is_pu;

    // Get the DC bus voltage
    gMotorVars.VdcBus_kV = _IQmpy(gAdcData.dcBus, _IQ(USER_IQ_FULL_SCALE_VOLTAGE_V/1000.0));

    // get the estimator angle and frequency values
    gMotorVars.angle_est_pu = EST_getAngle_pu(ctrlHandle->estHandle);
    gMotorVars.speed_est_pu = EST_getFm_pu(ctrlHandle->estHandle);

    return;
} // end of updateGlobalVariables_motor() function

void updateIdIqRef(CTRL_Handle handle)
{
    _iq iq_ref = gMotorVars.IqRef_pu; //_IQmpy(gMotorVars.IqRef_A,_IQ(1.0/USER_IQ_FULL_SCALE_CURRENT_A));

    // set the speed reference so that the forced angle rotates in the correct direction for startup
    if (_IQabs(gMotorVars.Speed_krpm) < _IQ(0.01))
    {
        if (iq_ref < _IQ(0.0))
        {
            CTRL_setSpd_ref_krpm(handle, _IQ(-0.01));
        }
        else if (iq_ref > _IQ(0.0))
        {
            CTRL_setSpd_ref_krpm(handle, _IQ(0.01));
        }
    }

    // Set the Id Iq reference that use to come out of the PI speed control
    CTRL_setIq_ref_pu(handle, gMotorVars.IqRef_pu);
    CTRL_setId_ref_pu(handle, gMotorVars.IdRef_pu);

    return;
} // end of updateIdIqRef() function

void updateKpKiGains(CTRL_Handle handle)
{
    if ((gMotorVars.CtrlState == CTRL_State_OnLine) && (gMotorVars.Flag_MotorIdentified == true) && (Flag_Latch_softwareUpdate == false))
    {
        // set the kp and ki speed values from the watch window
        CTRL_setKp(handle, CTRL_Type_PID_spd, gMotorVars.Kp_spd);
        CTRL_setKi(handle, CTRL_Type_PID_spd, gMotorVars.Ki_spd);

        // set the kp and ki current values for Id and Iq from the watch window
        CTRL_setKp(handle, CTRL_Type_PID_Id, gMotorVars.Kp_Idq);
        CTRL_setKi(handle, CTRL_Type_PID_Id, gMotorVars.Ki_Idq);
        CTRL_setKp(handle, CTRL_Type_PID_Iq, gMotorVars.Kp_Idq);
        CTRL_setKi(handle, CTRL_Type_PID_Iq, gMotorVars.Ki_Idq);
    }

    return;
} // end of updateKpKiGains() function

//@} //defgroup
// end of file
