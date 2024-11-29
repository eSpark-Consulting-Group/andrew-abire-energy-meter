#include "main.h"
#include "modbus_user.h"
#include "emavg.h"
#include "scidrv.h"
#include "modbus_slave.h"
#include "math.h"
#include "userio.h"
#include <stdlib.h>

#ifdef FLASH
#pragma DATA_SECTION(holdingRegs, "modbus_data");
#pragma DATA_SECTION(mbmsg, "modbus_data");
#endif

extern HAL_Obj hal;
extern HAL_Handle halHandle;
extern CTRL_Handle ctrlHandle;
extern unsigned int holdingRegs[]; // function 3 and 16 register array
extern volatile MOTOR_Vars_t gMotorVars;

bool overcurrentFlag = false;
bool warncurrentFlag = false;

MODBUS_MSG_T mbmsg;
SENSOR_T sensor;
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
    mbmsg.ocp_dcamp_set_f = 0.0;
    mbmsg.motor_current_running = 0.0;
    mbmsg.io.all = 0;
    error.all = 0;
    stat.all = 0;
    fault.all = 0;
    //compressor only
    mbmsg.start_psi = 0;
    mbmsg.stop_psi = 0;
    mbmsg.tank_pressure = 0;
    mbmsg.zero_psi_offset = 10;
    mbmsg.pressure_control_method = 0;
    mbmsg.max_psi_sensor = 100;
    mbmsg.zero_psi_sensor_volt = 1000; //mV
    mbmsg.max_psi_sensor_volt = 5000; //mV
    sensor.max_psi_sensor = 100;
    sensor.zero_psi_adc_reading = 376;
    sensor.max_psi_adc_reading = 1504;
    sensor.err_psi_adc_reading_low = 300;
    sensor.err_psi_adc_reading_high = 1580;
    sensor.max_start_psi = 97;
    sensor.max_stop_psi = 100;
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

int16_t tank_pressure = 999;

void modbus_loop()
{
    _iq krpm_ref = 0;

    holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 6] = mbmsg.heartbeat_out;  // controller heartbeat       ->

    rpm_filter.In = _IQtoF(gMotorVars.Speed_krpm) * 1000.0L;
    MATH_EMAVG_C(rpm_filter);
    if (gMotorVars.Flag_Run_Identify)
    {
        if (CTRL_getState(ctrlHandle) != CTRL_State_OffLine)
        {
            mbmsg.speed_out = (int16_t) roundf(rpm_filter.Out);
            holdingRegs[HOLDREG_OUT_BASE_ADDRESS] = mbmsg.speed_out;      // motor rpm running      ->
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
    idc_filter.In = 10.0L * (float) IDC_ADC_RESULT/ (24.81812L * IDC_SENSE_RESISTOR_MOHM); //B7 SOC CH9
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

    // low = 1V = 0PSI, high = 4V = 100PSI
    // pressure = 100PSI x (Vout-1V)/(4V-1V)
    if ((TANK_PRESSURE_ADC_RESULT <= sensor.err_psi_adc_reading_low)
      ||(TANK_PRESSURE_ADC_RESULT >= sensor.err_psi_adc_reading_high))
    {
        error.bit.snserr = 1;
    }
    else
    {
        error.bit.snserr = 0;
    }

    //sensor normal
    tank_pressure = (int16_t) (sensor.max_psi_sensor * ((float) TANK_PRESSURE_ADC_RESULT- sensor.zero_psi_adc_reading)
    / (sensor.max_psi_adc_reading - sensor.zero_psi_adc_reading)) + PRESSURE_SENSOR_OFFSET;
    if (tank_pressure > 999)
    {
        tank_pressure = 999;
    }
    if (tank_pressure < -999)
    {
        tank_pressure = -999;
    }

    // user psi correction above/below sea level and pressure sensor tolerance, 2.9psi is +5000m
    mbmsg.tank_pressure = tank_pressure - mbmsg.zero_psi_offset;

    holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 8] = mbmsg.tank_pressure; // tank pressure  ->
    holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 7] = mbmsg.io.all;        // DIO status     ->

    modbus_update();

    //
    mbmsg.heartbeat_in = holdingRegs[HOLDREG_IN_BASE_ADDRESS];       // Client heartbeat       <-
    mbmsg.onoff_in = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 1];       // ON/OFF control input   <-
    mbmsg.speed_in = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 2];       // Motor speed set        <-
    mbmsg.min_dcv_input = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 3];  // min_dcv_input set      <-
    mbmsg.max_dcv_input = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 4];  // max_dcv_input set      <-
    mbmsg.slow_down_dcv = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 5];  // slow_down_dcv set      <-
    mbmsg.ocp_dcamp_set = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 6];  // ocp_ratio set          <-
    mbmsg.mode = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 7];           // control mode           <-
    if (holdingRegs[HOLDREG_IN_BASE_ADDRESS + 8] <= 2) // check range to prevent fatal error
    {
        mbmsg.pressure_control_method = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 8]; //pressure control method     <-
    }
    else
    {
        mbmsg.pressure_control_method = 0; // set to default "stop motor"
    }
    mbmsg.start_psi = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 9];      // start pressure set     <-
    mbmsg.stop_psi = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 10];      // stop speed set         <-
    if (holdingRegs[HOLDREG_IN_BASE_ADDRESS + 11] <= 10) // check range to prevent fatal error
    {
        mbmsg.zero_psi_offset = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 11]; // zero psi offset set      <-
    }
    else
    {
        mbmsg.zero_psi_offset = 5; // set to default 0 psi offset
    }
    mbmsg.max_psi_sensor = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 12];           // max sensor psi set                     <-
    mbmsg.zero_psi_sensor_volt = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 13];     // zero psi sensor output volt set        <-
    mbmsg.max_psi_sensor_volt = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 14];      // max psi sensor output volt set         <-
    mbmsg.communication_fail_action = holdingRegs[HOLDREG_IN_BASE_ADDRESS + 15];           // Communication fail action           <-

    sensor.max_psi_sensor = mbmsg.max_psi_sensor;
    sensor.zero_psi_adc_reading = (uint16_t)((float)mbmsg.zero_psi_sensor_volt / 1000.0L * 400.0L);
    sensor.max_psi_adc_reading = (uint16_t)((float)mbmsg.max_psi_sensor_volt / 1000.0L * 400.0L);
    sensor.err_psi_adc_reading_low = (uint16_t)((float)sensor.zero_psi_adc_reading * 0.5L);
    sensor.err_psi_adc_reading_high = (uint16_t)((float)sensor.max_psi_adc_reading + ((float)sensor.zero_psi_adc_reading - (float)sensor.err_psi_adc_reading_low));
    sensor.max_start_psi = sensor.max_psi_sensor - 3;
    sensor.max_stop_psi = sensor.max_psi_sensor;

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
        if ((mbmsg.slow_down_dcv >= mbmsg.max_dcv_input) || (mbmsg.slow_down_dcv <= mbmsg.min_dcv_input)
         || (mbmsg.start_psi == 0) || (mbmsg.start_psi > sensor.max_start_psi)
         || (mbmsg.stop_psi < 4 || mbmsg.stop_psi > sensor.max_stop_psi)
         || (mbmsg.stop_psi - mbmsg.start_psi < 3)
         || (mbmsg.start_psi > mbmsg.stop_psi))
        {
            error.bit.settingerr = 1;
        }
        else
        {
            error.bit.settingerr = 0;
        }

        //stop motor on wrong settings & on OCP
        // start pressure set should be higher than or equal to 1
        // stop pressure set should be less than or equal to MAXIMUM_STOP_PSI and >= 3
        // minimum pressure difference to start the motor is 3psi
        // if tank pressure is less than start pressure, motor on
        // if tank pressure is higher than stop pressure, motor off
        if ((error.bit.ovperr) || (error.bit.uvperr)
         || (error.bit.settingerr) || (error.bit.snserr)
         || (error.bit.ocperr || error.bit.ocpwrn) || (error.bit.rpmerr))
        {
            gMotorVars.Flag_Run_Identify = false;
        }
        else
        {
            if (mbmsg.onoff_in || (stat.bit.onoff && (mbmsg.mode == 2)))
            {
                if ((mbmsg.tank_pressure < mbmsg.start_psi) || ((65536 - mbmsg.tank_pressure) < mbmsg.start_psi))
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
        if ((abs(mbmsg.speed_in) < 20) || (abs(mbmsg.speed_in) > MAXIMUM_RPM))
        {
            error.bit.rpmerr = 1;
        }
        else
        {
            error.bit.rpmerr = 0;
        }

        //check tank pressure
        if ((mbmsg.tank_pressure >= sensor.max_stop_psi) && ((65536 - mbmsg.tank_pressure) >= sensor.max_stop_psi)) //stop at max. sensor output > MAXIMUM_STOP_PSI
        {
            gMotorVars.Flag_Run_Identify = false;
        }
        else if ((mbmsg.tank_pressure >= mbmsg.stop_psi) && ((65536 - mbmsg.tank_pressure) >= mbmsg.stop_psi))
        {
            if (mbmsg.pressure_control_method == 2) // == DOUT3 CLOSE
            {
                DO3_CLOSE(); //DO3 Low Z
                mbmsg.io.bit.DO3 = 1;
            }
            else if (mbmsg.pressure_control_method == 1) //== DOUT3 OPEN
            {
                DO3_OPEN(); //DO3 High Z
                mbmsg.io.bit.DO3 = 0;
            }
            else //should be 0, any value other than 1, 2 will use "STOP MOTOR"
            {
                DO3_OPEN(); //DO3 High Z, nothing should be connected to DO3
                mbmsg.io.bit.DO3 = 0;
                gMotorVars.Flag_Run_Identify = false;
            }
            // max. +3psi is allowed when the solenoid valve cannot release air pressure faster than motor compression
            if (mbmsg.tank_pressure >= mbmsg.stop_psi + 3)
            {
                gMotorVars.Flag_Run_Identify = false;
            }
        }
        else if ((mbmsg.tank_pressure <= (mbmsg.stop_psi - 3)) && ((65536 - mbmsg.tank_pressure) <= (mbmsg.stop_psi - 3)))
        {
            if (mbmsg.pressure_control_method == 2) // == DOUT3 CLOSE
            {
                DO3_OPEN(); //DO3 High Z
                mbmsg.io.bit.DO3 = 0;
            }
            else if (mbmsg.pressure_control_method == 1) //== DOUT3 OPEN
            {
                DO3_CLOSE(); //DO3 Low Z
                mbmsg.io.bit.DO3 = 1;
            }
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
                        krpm_ref = _IQ((float)(abs(mbmsg.speed_in)) / 2000.0L); //reduce to half speed
                    }
                    else if (stat.bit.dir == DIR_CCW)
                    {
                        krpm_ref = _IQ((float)(-abs(mbmsg.speed_in)) / 2000.0L); //reduce to half speed
                    }
                }
                else //direction controlled by +/-rpm set
                {
                    krpm_ref = _IQ((float)mbmsg.speed_in / 2000.0L); //reduce to half speed
                }
            }
            else if(abs(mbmsg.speed_in) >= 20) //minimum stable FOC speed is 20rpm
            {
                //DIO mode, direction controlled by DI2 but not +/-rpm set
                if (mbmsg.mode == 2)
                {
                    if (stat.bit.dir == DIR_CW)
                    {
                        krpm_ref = _IQ(20.0L / 1000.0L);
                    }
                    else if (stat.bit.dir == DIR_CCW)
                    {
                        krpm_ref = _IQ(-20.0L / 1000.0L);
                    }
                }
                else //direction controlled by +/-rpm set
                {
                    if (mbmsg.speed_in > 0)
                    {
                        krpm_ref = _IQ(20.0L / 1000.0L);
                    }
                    else if (mbmsg.speed_in < 0)
                    {
                        krpm_ref = _IQ(-20.0L / 1000.0L);
                    }
                }
            }
        }
        else //normal speed
        {
            //DIO mode, direction controlled by DI2 but not +/-rpm set
            if (mbmsg.mode == 2)
            {
                if (stat.bit.dir == DIR_CW)
                {
                    krpm_ref = _IQ((float)(abs(mbmsg.speed_in)) / 1000.0L);
                }
                else if (stat.bit.dir == DIR_CCW)
                {
                    krpm_ref = _IQ((float)(-abs(mbmsg.speed_in)) / 1000.0L);
                }
            }
            else //direction controlled by +/-rpm set
            {
                krpm_ref = _IQ((float)mbmsg.speed_in / 1000.0L);
            }
        }

        /*
        //ratiometrically speed limit
        if(mbmsg.tank_pressure >= 90)
        {
            if(mbmsg.speed_in > 0.25)
            krpm_ref = _IQ(0.25);
        }
        else if(mbmsg.tank_pressure >= 80)
        {
            if(mbmsg.speed_in > 0.26)
            krpm_ref = _IQ(0.26);
        }
        else if(mbmsg.tank_pressure >= 70)
        {
            if(mbmsg.speed_in > 0.26)
            krpm_ref = _IQ(0.26);
        }
        else if(mbmsg.tank_pressure >= 60)
        {
            if(mbmsg.speed_in > 0.275)
            krpm_ref = _IQ(0.275);
        }
        else if(mbmsg.tank_pressure >= 50)
        {
            if(mbmsg.speed_in > 0.29)
            krpm_ref = _IQ(0.29);
        }
        else if(mbmsg.tank_pressure >= 40)
        {
            if(mbmsg.speed_in > 0.31)
            krpm_ref = _IQ(0.31);
        }
        else if(mbmsg.tank_pressure >= 30)
        {
            if(mbmsg.speed_in > 0.32)
            krpm_ref = _IQ(0.32);
        }
        else if(mbmsg.tank_pressure >= 20)
        {
            if(mbmsg.speed_in > 0.33)
            krpm_ref = _IQ(0.33);
        }
        else if(mbmsg.tank_pressure >= 10)
        {
            if(mbmsg.speed_in > 0.34)
            krpm_ref = _IQ(0.34);
        }*/

        gMotorVars.SpeedRef_krpm = krpm_ref;

        //over current protection
        mbmsg.ocp_dcamp_set_f = (float) mbmsg.ocp_dcamp_set / 10.0L;

        if (CTRL_getState(ctrlHandle) == CTRL_State_OffLine || CTRL_getState(ctrlHandle) == CTRL_State_Idle)
        {
            mbmsg.motor_current_running = 0.0;
        }
        else
        {
            mbmsg.motor_current_running = _IQtoF(gMotorVars.Is_pu) * USER_IQ_FULL_SCALE_CURRENT_A / 1.414213562373L;
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
            error.bit.ocperr = 1; //this flag cannot be reset once set
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
            if (mbmsg.communication_fail_action == 1)
            {
                // resetCounter will not increment after a device reset by watchdog (since mbmsg.communication_fail_action = 0)
                // we only want a reset (all motor setting from MODBUS will be reset) if user selection is STOP MOTOR, otherwise, let the motor run
                resetCounter++;
                if (resetCounter > 10) //10 * 3sec = 30sec
                {
                    resetCounter = 0;
                    HAL_resetDevice(halHandle); //restart the device every 30min if no communication to prevent stucks
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
        mbmsg.onoff_out = gMotorVars.Flag_Run_Identify;
        holdingRegs[HOLDREG_OUT_BASE_ADDRESS + 3] = mbmsg.onoff_out;  // motor onoff       ->
        mbmsg.motor_current_running = 0;
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
    fault.bit.sensor = error.bit.snserr;

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
