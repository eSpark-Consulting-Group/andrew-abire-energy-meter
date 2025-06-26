#include "main.h"
#include "modbus_user.h"
#include "emavg.h"
#include "scidrv.h"
#include "modbus_slave.h"
#include "math.h"
#include "userio.h"
#include <stdlib.h>
#include "RTD.h"

#ifdef FLASH
#pragma DATA_SECTION(holdingRegs, "modbus_data");
#pragma DATA_SECTION(mbmsg, "modbus_data");
#pragma DATA_SECTION(mtrModel, "reserved_ram");
#pragma DATA_SECTION(brdSeries, "reserved_ram");
#endif

extern HAL_Obj hal;
extern HAL_Handle halHandle;
extern CTRL_Handle ctrlHandle;
extern unsigned int holdingRegs[]; // function 3 and 16 register array
extern volatile MOTOR_Vars_t gMotorVars;

bool overcurrentFlag = false;
bool warncurrentFlag = false;

uint16_t mtrModel;
uint16_t brdSeries;
float_t rDC_MOhm = 5.0;

MODBUS_MSG_T mbmsg;
union ERROR_DATA_T error;
union FAULT_T fault;
union STATUS_T stat;
TIMEOUT_T to;
uint16_t holdingRegs[TOTAL_PACKETS]; // function 3 and 16 register array

MATH_EMAVG_C vdc_filter = { 0.0, 0.0, 0.008 };
MATH_EMAVG_C idc_filter = { 0.0, 0.0, 0.004 };
MATH_EMAVG_C rpm_filter = { 0.0, 0.0, 0.001 };
MATH_EMAVG_C motora_filter = { 0.0, 0.0, 0.001 };

uint32_t resetCounter = 0;
uint16_t resetState = 0;

void modbus_setup()
{
    mbSlaveInit(/*slave id*/1, GPIO_Number_12, TOTAL_PACKETS, holdingRegs);
    //common
    mbmsg.speed_in = 0;
    mbmsg.speed_out = 0;
    mbmsg.dcvoltage_out = 0;
    mbmsg.dccurrent_out = 0;
    mbmsg.dccurrent_out_f = 0.0;
    mbmsg.motor_current_out = 0;
    mbmsg.onoff_in = 0;
    mbmsg.onoff_out = 0;
    mbmsg.mode = 0;
    mbmsg.communication_fail_action = 0;
    mbmsg.prev_heartbeat_in = 0;
    mbmsg.heartbeat_in = 0;
    mbmsg.heartbeat_out = 0;
    mbmsg.min_dcv_input = 0;
    mbmsg.max_dcv_input = 0;
    mbmsg.slow_down_dcv = 0;
    mbmsg.ocp_dcamp_set = 0;
    mbmsg.off_temperature = -20;
    mbmsg.on_temperature = -10;
    mbmsg.temperature_now = 0;
    mbmsg.tempctrl.all = 0;
    mbmsg.ocp_dcamp_set_f = 0.0;
    mbmsg.motor_current_running = 0.0;
    mbmsg.io.all = 0;
    mbmsg.motor_model.all = 0;
    mbmsg.reset = 0;
    mbmsg.reset_ok = 0;
    error.all = 0;
    stat.all = 0;
    fault.all = 0;
    //timeout manager
    to.t15Count = 0;
    to.t15TimeoutFlag = false;
    to.t35Count = 0;
    to.hearbeatCount = 0;
    to.commCount = 0;
    to.commTimeoutFlag = false;
    to.timer2minCount = 0;
    to.timer15sCount = 0;
}

void modbus_loop()
{
    int16_t temp_now;

    holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 6] = mbmsg.heartbeat_out;  // controller heartbeat       ->

    rpm_filter.In = _IQtoF(gMotorVars.Speed_krpm) * 1000.0L;
    MATH_EMAVG_C(rpm_filter);
    if (gMotorVars.Flag_Run_Identify)
    {
        if (CTRL_getState(ctrlHandle) != CTRL_State_OffLine)
        {
            mbmsg.speed_out = (int16_t) roundf(rpm_filter.Out);
            holdingRegs[HOLDREG_OUT_BASE_ADDRESS] = mbmsg.speed_out;  // motor rpm running      ->
            mbmsg.onoff_out = 1;
        }
        else
        {
            mbmsg.speed_out = 0;
            holdingRegs[HOLDREG_OUT_BASE_ADDRESS] = 0;
            mbmsg.onoff_out = 0;
        }
    }
    else
    {
        mbmsg.speed_out = 0;
        holdingRegs[HOLDREG_OUT_BASE_ADDRESS] = 0;
        mbmsg.onoff_out = 0;
    }

    holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 3] = mbmsg.onoff_out;  // motor onoff       ->

    vdc_filter.In = _IQtoF(gMotorVars.VdcBus_kV) * 10000.0L * VOLTAGE_CORRECTION_FACTOR;
    MATH_EMAVG_C(vdc_filter);
    mbmsg.dcvoltage_out = (int16_t) vdc_filter.Out;
    holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 1] = mbmsg.dcvoltage_out;  // DC bus voltage running     ->

    // if Rsns = 1 mOhm, sensor gain = 20V/V
    // then 1A = 0.001V x 20V/V = 0.02V = 0.02V/3.3V*4095 = 24.81812 * IDC_SENSE_RESISTOR_MOHM
    if(brdSeries == 1)
    {
        rDC_MOhm = 5.0; // D = 0.005
    }
    else if(brdSeries == 0)
    {
        rDC_MOhm = 2.0; // A = 0.002
    }
    idc_filter.In = 10.0L * (float) IDC_ADC_RESULT/ (24.81812L * rDC_MOhm); //B7 SOC CH9
    MATH_EMAVG_C(idc_filter);
    mbmsg.dccurrent_out = (int16_t) idc_filter.Out;
    holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 2] = mbmsg.dccurrent_out; // DC bus current running     ->

    motora_filter.In = 10.0L * mbmsg.motor_current_running;
    MATH_EMAVG_C(motora_filter);
    if (gMotorVars.Flag_Run_Identify)
    {
        if (CTRL_getState(ctrlHandle) == CTRL_State_OffLine || CTRL_getState(ctrlHandle) == CTRL_State_Idle)
        {
            mbmsg.motor_current_out = 0;
            holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 5] = 0;
        }
        else
        {
            mbmsg.motor_current_out = (int16_t) motora_filter.Out;
            holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 5] = mbmsg.motor_current_out; // motor current running     ->
        }
    }
    else
    {
        mbmsg.motor_current_out = 0;
        holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 5] = 0;
    }

    holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 7] = mbmsg.io.all;                 // DIO status     ->

    temp_now = getTemperature16(AIN4, 1.0L);
    if(mbmsg.tempctrl.bit.unit == true) //deg C to F
    {
        mbmsg.temperature_now = getFfromDegC(temp_now);
    }
    else
    {
        mbmsg.temperature_now = temp_now;
    }
    holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 8] = mbmsg.temperature_now;        // Current temperature reading   ->

    if(mbmsg.reset == 1)
    {
      mbmsg.reset_ok = 1;
      resetState = 1;
    }
    else
    {
        if(resetState == 1)
        {
          mtrModel = mbmsg.motor_model.bit.BYTE0;
          brdSeries = mbmsg.motor_model.bit.BYTE1;
          // reset
          HAL_disablePwm(halHandle); // disable the PWM
          gMotorVars.Flag_enableSys = false;
          HAL_resetDevice(halHandle); // restart the device
        }
    }

    holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 9] = mbmsg.reset_ok; // response a reset ok

    modbus_update();

    mbmsg.heartbeat_in = holdingRegs[HOLDREG_IN_BASE_ADDRESS];       // Client heartbeat       <-
    mbmsg.onoff_in = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 1];       // ON/OFF control input   <-
    mbmsg.speed_in = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 2];       // Motor speed set        <-
    mbmsg.min_dcv_input = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 3];  // min_dcv_input set      <-
    mbmsg.max_dcv_input = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 4];  // max_dcv_input set      <-
    mbmsg.slow_down_dcv = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 5];  // slow_down_dcv set      <-
    mbmsg.ocp_dcamp_set = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 6];  // ocp_ratio set          <-
    mbmsg.mode = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 7];           // control mode           <-
    mbmsg.communication_fail_action = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 8];           // Communication fail action           <-
    mbmsg.off_temperature = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 9];           // Off temperature            <-
    mbmsg.on_temperature = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 10];           // On temperature             <-
    mbmsg.tempctrl.all = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 11];             // Temperature On/Off, unit   <
    mbmsg.motor_model.all = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 12];          // Motor model                <-
    mbmsg.reset = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 13];                    // Controller reset           <-

    // if heartbeat increment is received from ESP32 normally, communication is normal
    if (mbmsg.heartbeat_in != mbmsg.prev_heartbeat_in)
    {
        //OVP
        if (mbmsg.dcvoltage_out >= mbmsg.max_dcv_input)
        {
            error.bit.ovperr = 1;
        }
        else
        {
            error.bit.ovperr = 0;
        }
        //UVP
        if (mbmsg.dcvoltage_out <= mbmsg.min_dcv_input)
        {
            error.bit.uvperr = 1;
        }
        else
        {
            error.bit.uvperr = 0;
        }
        //LOW VOLTAGE
        if (mbmsg.dcvoltage_out <= mbmsg.slow_down_dcv)
        {
            error.bit.slowdown = 1;
        }
        else
        {
            error.bit.slowdown = 0;
        }
        //OTHERS - settings error
        if ((mbmsg.slow_down_dcv >= mbmsg.max_dcv_input) || (mbmsg.slow_down_dcv <= mbmsg.min_dcv_input))
        {
            error.bit.settingerr = 1;
        }
        else
        {
            error.bit.settingerr = 0;
        }

        //stop motor on wrong settings & on OCP
        if ((error.bit.ovperr) || (error.bit.uvperr)
         || (error.bit.settingerr)
         || (error.bit.ocperr) || (error.bit.rpmerr))
        {
            gMotorVars.Flag_Run_Identify = false;
        }
        else
        {
            if (mbmsg.onoff_in || (stat.bit.onoff && (mbmsg.mode == 2)))
            {
                // Temperature control
                if (mbmsg.tempctrl.bit.onoff == false)
                {
                    if (mbmsg.temperature_now <= mbmsg.off_temperature)
                    {
                        gMotorVars.Flag_Run_Identify = false;
                    }
                    else if (mbmsg.temperature_now >= mbmsg.on_temperature)
                    {
                        gMotorVars.Flag_Run_Identify = true;
                    }
                }
                else
                {
                    gMotorVars.Flag_Run_Identify = true;
                }
            }
            if (!stat.bit.onoff && (mbmsg.mode == 2))
            {
                gMotorVars.Flag_Run_Identify = false;
            }
        }

        //check rpm range
        if (abs(mbmsg.speed_in) < 20)
        {
            error.bit.rpmerr = 1;
        }
        else
        {
            error.bit.rpmerr = 0;
        }

        //slow down the motor if low battery
        if (error.bit.slowdown)
        {
            //slow down to half speed
            if (abs(mbmsg.speed_in) >= 40) //minimum stable FOC speed is 20rpm
            {
                //DIO mode, direction controlled by DI2 but not +/-rpm set
                if (mbmsg.mode == 2)
                {
                    if (stat.bit.dir == DIR_CW)
                    {
                        gMotorVars.SpeedRef_krpm = _IQ((float)(abs(mbmsg.speed_in)) / 2000.0L); //reduce to half speed
                    }
                    else if (stat.bit.dir == DIR_CCW)
                    {
                        gMotorVars.SpeedRef_krpm = _IQ((float)(-abs(mbmsg.speed_in)) / 2000.0L); //reduce to half speed
                    }
                }
                else //direction controlled by +/-rpm set
                {
                    gMotorVars.SpeedRef_krpm = _IQ((float)mbmsg.speed_in / 2000.0L); //reduce to half speed
                }
            }
            else if(abs(mbmsg.speed_in) >= 20) //minimum stable FOC speed is 20rpm
            {
                //DIO mode, direction controlled by DI2 but not +/-rpm set
                if (mbmsg.mode == 2)
                {
                    if (stat.bit.dir == DIR_CW)
                    {
                        gMotorVars.SpeedRef_krpm = _IQ(20.0L / 1000.0L);
                    }
                    else if (stat.bit.dir == DIR_CCW)
                    {
                        gMotorVars.SpeedRef_krpm = _IQ(-20.0L / 1000.0L);
                    }
                }
                else //direction controlled by +/-rpm set
                {
                    if (mbmsg.speed_in > 0)
                    {
                        gMotorVars.SpeedRef_krpm = _IQ(20.0L / 1000.0L);
                    }
                    else if (mbmsg.speed_in < 0)
                    {
                        gMotorVars.SpeedRef_krpm = _IQ(-20.0L / 1000.0L);
                    }
                }
            }
        }
        else
        {
            //DIO mode, direction controlled by DI2 but not +/-rpm set
            if (mbmsg.mode == 2)
            {
                if (stat.bit.dir == DIR_CW)
                {
                    gMotorVars.SpeedRef_krpm = _IQ((float)(abs(mbmsg.speed_in)) / 1000.0L);
                }
                else if (stat.bit.dir == DIR_CCW)
                {
                    gMotorVars.SpeedRef_krpm = _IQ((float)(-abs(mbmsg.speed_in)) / 1000.0L);
                }
            }
            else //direction controlled by +/-rpm set
            {
                gMotorVars.SpeedRef_krpm = _IQ((float)mbmsg.speed_in / 1000.0L);
            }
        }

        //over current protection
        mbmsg.ocp_dcamp_set_f = (float) mbmsg.ocp_dcamp_set / 10.0L;

        if (CTRL_getState(ctrlHandle) == CTRL_State_OffLine || CTRL_getState(ctrlHandle) == CTRL_State_Idle)
        {
            mbmsg.motor_current_running = 0.0;
        }
        else
        {
            mbmsg.motor_current_running = _IQtoF(gMotorVars.Is_pu) * gUserVar.USER_IQ_FULL_SCALE_CURRENT_A / 1.414213562373L;
        }

        mbmsg.dccurrent_out_f = (float)mbmsg.dccurrent_out / 10.0L;
        //if over current, start timer counter
        if (mbmsg.dccurrent_out_f >= mbmsg.ocp_dcamp_set_f * 1.5L) //1.5 x current set
        {
            overcurrentFlag = true; //this will start the timer counter
        }
        else if (mbmsg.dccurrent_out_f >= mbmsg.ocp_dcamp_set_f * 1.2L) //1.2 x current set
        {
            warncurrentFlag = true; //this will start the timer counter
        }
        else
        {
            warncurrentFlag = false;
            overcurrentFlag = false;
            to.timer15sCount = 0; //restart counter
            to.timer2minCount = 0; //restart counter
            error.bit.ocpwrn = 0; //reset over current warning
        }

        //stop motor if 150% over current for 15s
        if (to.timer15sCount >= 300000UL) //no. of count of 50us
        {
            error.bit.ocperr = 1; //this flag cannot be reset once set unless user hit the "OFF" key
        }

        //warn if 120% over current for 2 mins
        if (to.timer2minCount >= 2400000UL) //no. of count of 50us
        {
            error.bit.ocpwrn = 1;
        }

        holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 4] = error.all;         // Error data                 ->

        mbmsg.prev_heartbeat_in = mbmsg.heartbeat_in; // save previous state
        to.commCount = Cputimer1IntCount; //restart timeout counter
        to.commTimeoutFlag = false;
        fault.bit.comfail = false;
        resetCounter = 0;
    }
    else // we treat it as a fault if timeout
    {
        if (to.commTimeoutFlag == true)
        {
            // If user selection is STOP MOTOR on communication failure
            if (mbmsg.communication_fail_action == 1)
            {
                // resetCounter will not increment after a device reset by watchdog (since mbmsg.communication_fail_action = 0)
                // we only want a reset (all motor setting from MODBUS will be reset) if user selection is STOP MOTOR, otherwise, let the motor run
                resetCounter++;
                if (resetCounter > 10) // 10 * 3sec = 30sec
                {
                    resetCounter = 0;
                    HAL_resetDevice(halHandle); // restart the device every 30min if no communication to prevent stucks
                }
                gMotorVars.Flag_Run_Identify = false; // Stop motor
                mbmsg.onoff_out = 0;
                holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 3] = mbmsg.onoff_out;  // motor onoff       ->
                fault.bit.comfail = true;
                SCI_disableRxFifoInt(hal.sciAHandle); // Disable Rx FIFO Int
                to.t15Count = Cputimer1IntCount; // restart timeout counter
                to.t15TimeoutFlag = 0;
                // Init var
                mb.err_count = 0;
                mb.pass_count = 0;
                mb.bufsize = 0;
                mb.framesize = 0;
                mb.byteTxed = 0;
                mb.state = MODBUS_IDLE;
                //SCI reset
                scia_init();        // Init SCI-A
                scia_fifo_init();   // Init SCI-A Fifos
                DELAY_US(10000);
            }
            to.commTimeoutFlag = false; //retry within another 3 seconds
        }
    }

    // no control is requested, turn off the motor
    if (!mbmsg.onoff_in && (mbmsg.mode != 2))
    {
        gMotorVars.Flag_Run_Identify = false; // Stop motor
        mbmsg.onoff_out = 0;
        holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 3] = mbmsg.onoff_out;  // motor onoff       ->
        mbmsg.motor_current_running = 0;
        // reset when user hit "OFF" key to give user another try
        warncurrentFlag = false; //reset over current warning flag
        overcurrentFlag = false; //reset over current error flag
        to.timer15sCount = 0; //restart counter
        to.timer2minCount = 0; //restart counter
        error.bit.ocpwrn = 0; //reset over current warning
        error.bit.ocperr = 0; //reset over current error
    }

    //DO1 indicates the on/off state
    if (gMotorVars.Flag_Run_Identify)
    {
        mbmsg.io.bit.DO1 = true;
        DO1_CLOSE(); //DO1 Low Z
    }
    else
    {
        mbmsg.io.bit.DO1 = false;
        DO1_OPEN(); //DO1 High Z
    }

    fault.bit.ocp = error.bit.ocperr;
    fault.bit.ovp = error.bit.ovperr;
    fault.bit.uvp = error.bit.uvperr;

    //DO2 indicates the fault state
    if (fault.all)
    {
        mbmsg.io.bit.DO2 = true;
        DO2_OPEN();
    }
    else
    {
        mbmsg.io.bit.DO2 = false;
        DO2_CLOSE();
    }

    // reset SCI if there is a hardware error
    if (hal.sciAHandle->SCIRXST & SCI_SCIRXST_BRKDT_BITS)
    {
        scia_init();        // Init SCI-A
        scia_fifo_init();   // Init SCI-A Fifos
        DELAY_US(10000);
    }
}

//===========================================================================
// No more.
//===========================================================================
