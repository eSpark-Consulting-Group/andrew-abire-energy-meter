/*
 * 20220308
 * sync with 27&69
 */
//*****************************************************************************
// the includes
//*****************************************************************************
#include "system.h"
#if MODBUS_ENABLE == 1
#include "ModbusSlave.h"
#include "serial.h"
#endif
#include "est.h"


//*****************************************************************************
// #defines
//*****************************************************************************



//*****************************************************************************
// globals
//*****************************************************************************
SYS_Obj sys;
MATH_vec2 gVdq_in;// this global variable is referenced in ctrl.h, for flying start

#if MODBUS_ENABLE == 1
extern ModbusSlave mb;
int debugcount = 0;
int mbCounter = 0;
#endif

//*****************************************************************************
// the functions
//*****************************************************************************
void sys_init(SYS_Obj* const self, HAL_Handle halHandle, CTRL_Handle ctrlHandle, USER_Params *pUserParams, volatile MOTOR_Vars_t *pMotorVars, const HAL_AdcData_t *pAdcData, HAL_PwmData_t *pPwmData)
{
    //external object pointer
    self->halHandle = halHandle;
    self->ctrlHandle = ctrlHandle;
    self->pUserParams = pUserParams;
    self->pMotorVars = pMotorVars;
    self->pAdcData = pAdcData;
    self->pPwmData = pPwmData;

    //task scheduling
    sys_timer_init(self);

    //protection
    sys_protect_init(self);

    //energisation
    self->pwmFire = false;//set 1 to fire pwm, it will set to 0 automatically
    self->pwmEnergized = false;//indicate whether pwm is energized
    self->relay_Contactor = self->relay_Precharge = false;
    START_UP_CHECK_init(&self->startUpCheck, self->runFreq_Hz, SYS_START_NONE_MS, SYS_START_INITIAL_MS, SYS_START_FS_MS ,SYS_START_CTRL_MS);
    self->onOffSwitch = false;
    self->systemOnVoltage_pu = _IQ(SYSTEM_ON_VOLTAGE/pUserParams->iqFullScaleVoltage_V);
    self->systemOffVoltage_pu = _IQ(SYSTEM_OFF_VOLTAGE/pUserParams->iqFullScaleVoltage_V);

    //current
    self->currentMax_pu = _IQ(gUserVar.USER_MOTOR_MAX_CURRENT/gUserVar.USER_IQ_FULL_SCALE_CURRENT_A);
    self->IsRef_pu = 0;
    self->IqRef_pu = 0;
    self->IdRef_pu = 0;

    //filters
    FILTER_FO_setUp(&self->freq_filter, self->runFreq_Hz, 20);
    FILTER_FO_setUp(&self->dcV_filter, gUserVar.USER_ISR_FREQ_Hz, 60);
#if SYS_IDCMEASURE_EXIST == 1
    FILTER_FO_setUp(&self->dcA_filter, gUserVar.USER_ISR_FREQ_Hz, 60);
#endif

    //data
#if SYS_IDCMEASURE_EXIST == 1
    self->dcA_peak = 0;
#endif

    //regen control
    COUNTER_init(&self->directionCheckCount,self->runFreq_Hz,200);
    self->motorDirection = 0;
    self->directionCheckState = DIRECTIONCHECK_NONE;
#if REGENVOLTCTRL_ENABLE == 1
    //regen pid
    self->regenVoltageTarget_V = REGEN_VOLTAGE_TARGET;
    TRAJ_setMaxDelta(&self->regenVoltageTarget_traj, _IQ((200.0/USER_IQ_FULL_SCALE_VOLTAGE_V)/self->runFreq_Hz));
    TRAJ_setMaxValue(&self->regenVoltageTarget_traj, _IQ(REGEN_VOLTAGE_MAX/USER_IQ_FULL_SCALE_VOLTAGE_V));
    TRAJ_setMinValue(&self->regenVoltageTarget_traj, _IQ(REGEN_VOLTAGE_MIN/USER_IQ_FULL_SCALE_VOLTAGE_V));//todo: set it after precharge
    TRAJ_setTargetValue(&self->regenVoltageTarget_traj, _IQ(self->regenVoltageTarget_V/USER_IQ_FULL_SCALE_VOLTAGE_V));
    TRAJ_setIntValue(&self->regenVoltageTarget_traj, _IQ(self->regenVoltageTarget_V/USER_IQ_FULL_SCALE_VOLTAGE_V));

    self->regenVoltagePIDout = _IQ(0.0);
    PID_init(&self->pid_dcv, sizeof(PID_Obj));
    PID_setGains((PID_Handle)&self->pid_dcv,_IQ(0.5),_IQ(0.0015),_IQ(0.00));//i 0.00002
    PID_setMinMax((PID_Handle)&self->pid_dcv,_IQ(-1.0),_IQ(1.0));
    PID_setRefValue((PID_Handle)&self->pid_dcv, TRAJ_getIntValue(&self->regenVoltageTarget_traj));
#endif

#if INTTEMP_ENABLE == 1
    user_IntTemp_init();
#endif

#if MODBUS_ENABLE == 1
    mb = construct_ModbusSlave(halHandle);
    serial_init(halHandle);
    COUNTER_init(&self->mbWatchdog,self->runFreq_Hz,MODBUS_WDPERIOD_MS);
    self->mbLastTS = 0;
#endif

    //IO
    DIGITALINPUT_init(&self->di1, self->runFreq_Hz, 200, FallingEdge);
    LINEARINPUT_init(&self->userADC1, 1, MODE2, 0, 2000, 4000, 0.01);
    LINEARINPUT_filter_init(&self->userADC1, self->runFreq_Hz, 100);
    LINEARINPUT_setFilterEnable(&self->userADC1, true);
}



void sys_loop(SYS_Obj* const self)
{
    //task scheduling
    if(self->runTick)
    {
        if(self->runTick >= 2)
        {
            self->processBandwidthError = 1;
        }
        --self->runTick;
        sys_run(self);//timed function
    }

    //protection
    if(self->powerOnErrorFreePassed)
    {
        bool stop = false;
        self->error.bit.ctrl |= (self->pMotorVars->CtrlState == CTRL_State_Error);
        self->error.bit.est |= (self->pMotorVars->EstState == EST_State_Error);
        if(self->protect.bit.dcov && self->error.bit.dcov)
        {
            stop |= true;
        }
        if(self->protect.bit.dcuv && self->error.bit.dcuv)
        {
            stop |= true;
        }
        if(self->protect.bit.dcoc && self->error.bit.dcoc)
        {
            stop |= true;
        }
        if(self->protect.bit.ooc_a && self->error.bit.ooc_a)
        {
            stop |= true;
        }
        if(self->protect.bit.ooc_b && self->error.bit.ooc_b)
        {
            stop |= true;
        }
        if(self->protect.bit.ooc_c && self->error.bit.ooc_c)
        {
            stop |= true;
        }
        if(self->protect.bit.gateFault_a && self->error.bit.gateFault_a)
        {
            stop |= true;
        }
        if(self->protect.bit.gateFault_b && self->error.bit.gateFault_b)
        {
            stop |= true;
        }
        if(self->protect.bit.gateFault_c && self->error.bit.gateFault_c)
        {
            stop |= true;
        }
        if(self->protect.bit.ctrl && self->error.bit.ctrl)
        {
            stop |= true;
        }
        if(self->protect.bit.est && self->error.bit.est)
        {
            stop |= true;
        }
        if(self->protect.bit.com && self->error.bit.com)
        {
            stop |= true;
        }
    #if PROTECT_GFA_ENABLE == 1
        if(GPIO_read(self->halHandle->gpioHandle, PROTECT_GFA_GPIO))
        {
            self->error.bit.gateFault_a = true;
            stop |= true;
        }
    #endif
    #if PROTECT_GFB_ENABLE == 1
        if(GPIO_read(self->halHandle->gpioHandle, PROTECT_GFB_GPIO))
        {
            self->error.bit.gateFault_b = true;
            stop |= true;
        }
    #endif
    #if PROTECT_GFC_ENABLE == 1
        if(GPIO_read(self->halHandle->gpioHandle, PROTECT_GFC_GPIO))
        {
            self->error.bit.gateFault_c = true;
            stop |= true;
        }
    #endif
        if(stop)
        {
            sys_deenergize(self);
        }
    }//if(self->powerOnErrorFreePassed)

    //relay output
#if SYS_PRECHARGE_PRECHARGERELAY_ENABLE == 1
    if(self->relay_Precharge){
        GPIO_setHigh(self->halHandle->gpioHandle,SYS_PRECHARGE_PRECHARGERELAY_GPIO);
    }else{
        GPIO_setLow(self->halHandle->gpioHandle,SYS_PRECHARGE_PRECHARGERELAY_GPIO);
    }
#endif
#if SYS_PRECHARGE_MAINRELAY_ENABLE == 1
    if(self->relay_Contactor){
        GPIO_setHigh(self->halHandle->gpioHandle,SYS_PRECHARGE_MAINRELAY_GPIO);
    }else{
        GPIO_setLow(self->halHandle->gpioHandle,SYS_PRECHARGE_MAINRELAY_GPIO);
    }
#endif

#if FAULT_CASCADE == 1
    //fault cascade
    if(HAL_readGpio(self->halHandle, GPIO_Number_15)){
        HAL_setGpioHigh(self->halHandle, GPIO_Number_58);
    }else{
        HAL_setGpioLow(self->halHandle, GPIO_Number_58);
    }
#endif

#if MODBUS_ENABLE == 1
    serial_loop();
    mb.loopStates(&mb);
    debugcount++;
#endif
}



void sys_isrFollower(SYS_Obj* const self)
{
    //filters
    FILTER_FO_run(&self->dcV_filter, self->pAdcData->dcBus);
#if SYS_IDCMEASURE_EXIST == 1
    FILTER_FO_run(&self->dcA_filter, _IQ12mpy((_iq)ADC_readResult(halHandle->adcHandle,ADC_ResultNumber_8), _IQ(USER_CURRENT_SF)) - _IQ(I_DC_offset));
#endif

    //protection
    if(self->powerOnErrorFreePassed)
    {
        const HAL_AdcData_t* pAdcData = self->pAdcData;
        _iq valueTemp = pAdcData->I.value[0];
        if(valueTemp > self->thresholdOOC_pu || valueTemp < -self->thresholdOOC_pu)
        {
            if(COUNTER_count_CTC(&self->errorCountOOC[0]))
            {
                self->error.bit.ooc_a |= true;
            }
        }
        else
        {
            COUNTER_reset(&self->errorCountOOC[0]);
        }

        valueTemp = pAdcData->I.value[1];
        if(valueTemp > self->thresholdOOC_pu || valueTemp < -self->thresholdOOC_pu)
        {
            if(COUNTER_count_CTC(&self->errorCountOOC[1]))
            {
                self->error.bit.ooc_b |= true;
            }
        }
        else
        {
            COUNTER_reset(&self->errorCountOOC[1]);
        }

        valueTemp = pAdcData->I.value[2];
        if(valueTemp > self->thresholdOOC_pu || valueTemp < -self->thresholdOOC_pu)
        {
            if(COUNTER_count_CTC(&self->errorCountOOC[2]))
            {
                self->error.bit.ooc_c |= true;
            }
        }
        else
        {
            COUNTER_reset(&self->errorCountOOC[2]);
        }
        //dc current protection
#if (SYS_IDCMEASURE_EXIST == 1 && PROTECT_DCOC_ENABLE == 1)
        valueTemp = FILTER_FO_get_y1(&self->dcA_filter);
        if(valueTemp > self->thresholdDCOC_pu || valueTemp < -self->thresholdDCOC_pu)
        {
            self->error.bit.dcoc |= true;
        }
        //log peak current
        if(valueTemp > self->dcA_peak){
            self->dcA_peak = valueTemp;
        }
#endif


#if SYS_AUTOONOFFBYVOLTAGE_ENABLE == 0
//        if((self->pwmEnergized || self->relay_Precharge || self->relay_Contactor) && (self->pAdcData->dcBus < self->thresholdDCUV_pu)){
//            self->error.bit.dcuv |= true;
//        }
#endif
    }//if(self->powerOnErrorFreePassed)

//    if(_IQabs(EST_getAngle_pu(self->ctrlHandle->estHandle)) > _IQ(0.5))
//    {
//        HAL_setGpioHigh(self->halHandle, GPIO_Number_34);
//    }
//    else
//    {
//        HAL_setGpioLow(self->halHandle, GPIO_Number_34);
//    }
}



void sys_isr(SYS_Obj* const self)
{
    _iq isTemp = 0;

    if(self->pwmFire){
        self->pwmFire = false;
        sys_energize(self);
    }

#if REGENVOLTCTRL_ENABLE == 1
    //regen pid
    if((self->directionCheckState == DIRECTIONCHECK_OK) && (self->pwmEnergized))
    {
        PID_run((PID_Handle)&self->pid_dcv, TRAJ_getIntValue(&self->regenVoltageTarget_traj), self->pAdcData->dcBus, &self->regenVoltagePIDout);
        if(self->motorDirection){//positive rpm
            isTemp = _IQmpy(self->regenVoltagePIDout, self->currentMax_pu);//- _IQ(0.005)
        }
        else{//negative rpm
            isTemp = _IQmpy(-self->regenVoltagePIDout, self->currentMax_pu);// + _IQ(0.005)
        }
    }
    else
    {
        isTemp = _IQ(0.0);
        //reset integration error
        PID_setUi((PID_Handle)&self->pid_dcv, _IQ(0.0));
    }
#endif

    if(self->pMotorVars->Flag_enableSpeedCtrl)
    {
        if(self->pMotorVars->CtrlState == CTRL_State_OnLine)
        {
            self->pMotorVars->SpeedRef_krpm = _IQmpy(LINEARINPUT_getOutput(&self->userADC1), _IQ(0.4));
        }
        else
        {
            self->pMotorVars->SpeedRef_krpm = 0;
        }
        CTRL_setSpd_ref_krpm(self->ctrlHandle,self->pMotorVars->SpeedRef_krpm);
        CTRL_setId_ref_pu(self->ctrlHandle, self->pMotorVars->IdRef_pu);
    }
    else
    {
        if(self->pMotorVars->CtrlState == CTRL_State_OnLine)
        {
            isTemp = _IQmpy(LINEARINPUT_getOutput(&self->userADC1), _IQ(gUserVar.USER_MOTOR_MAX_CURRENT/gUserVar.USER_IQ_FULL_SCALE_CURRENT_A));
        }
        else
        {
            isTemp = 0;
        }
        self->pMotorVars->IqRef_pu = isTemp;
//        self->pMotorVars->IdRef_pu = 0;

        // set the speed reference so that the forced angle rotates in the correct direction for startup
        if(_IQabs(self->pMotorVars->Speed_krpm) < _IQ(0.01))
          {
            if(self->pMotorVars->IqRef_pu < _IQ(0.0))
              {
                CTRL_setSpd_ref_krpm(self->ctrlHandle,_IQ(-0.01));
              }
            else if(self->pMotorVars->IqRef_pu > _IQ(0.0))
              {
                CTRL_setSpd_ref_krpm(self->ctrlHandle,_IQ(0.01));
              }
          }

        // Set the Iq reference that use to come out of the PI speed control
        CTRL_setIq_ref_pu(self->ctrlHandle, self->pMotorVars->IqRef_pu);
        CTRL_setId_ref_pu(self->ctrlHandle, self->pMotorVars->IdRef_pu);
    }

}



void sys_run(SYS_Obj* const self)
{
    //protection error reset at power on
    if(self->powerOnErrorFreePassed)
    {//energisation procedure
        bool condition = false;
        START_UP_CHECK_Obj* suc = &self->startUpCheck;
        condition = suc->state == STARTUPCHECK_NONE && !self->pwmEnergized && !sys_getError(self) && self->onOffSwitch;
        DIGITALINPUT_run(&suc->none, condition);
        condition = suc->state == STARTUPCHECK_INITIAL && (FILTER_FO_get_y1(&self->dcV_filter) > self->systemOnVoltage_pu) && !sys_getError(self);
        DIGITALINPUT_run(&suc->initial, condition);
#if SYS_FAST_ENABLE == 1
        condition = suc->state == STARTUPCHECK_FLYINGSTART && (self->pMotorVars->CtrlState == CTRL_State_OnLine) && (self->pMotorVars->EstState == EST_State_OnLine);
#else
        condition = suc->state == STARTUPCHECK_FLYINGSTART;
#endif
        DIGITALINPUT_run(&suc->flyingStart, condition);
        condition = suc->state == STARTUPCHECK_CONTROL && !sys_getError(self);
        DIGITALINPUT_run(&suc->control, condition);
        START_UP_CHECK_run(self);
    }
    else
    {
        self->powerOnErrorFreePassed = COUNTER_count_SAT(&self->powerOnErrorFreeCount);
    }//if(self->powerOnErrorFreePassed)

#if SYS_AUTOONOFFBYVOLTAGE_ENABLE == 1
    if(!self->onOffSwitch && (FILTER_FO_get_y1(&self->dcV_filter) > self->systemOnVoltage_pu))
    {
        self->onOffSwitch = true;
    }
    else if(self->onOffSwitch && (FILTER_FO_get_y1(&self->dcV_filter) < self->systemOffVoltage_pu))
    {
        self->onOffSwitch = false;
    }
#endif

    //filters
    if((self->pMotorVars->CtrlState == CTRL_State_OnLine) && (self->pMotorVars->EstState == EST_State_OnLine))
    {
        FILTER_FO_run(&self->freq_filter, EST_getFm_pu(self->ctrlHandle->estHandle));
    }
    else
    {
        FILTER_FO_set_x1(&self->freq_filter, 0);
        FILTER_FO_set_y1(&self->freq_filter, 0);
        FILTER_FO_run(&self->freq_filter, 0);
    }

    //motor direction check
    {
        _iq freqTemp = FILTER_FO_get_y1(&self->freq_filter);
        if((self->pMotorVars->CtrlState == CTRL_State_OnLine) && (self->pMotorVars->EstState == EST_State_OnLine))
        {
            if (self->directionCheckState == DIRECTIONCHECK_NONE)
            {
                if(_IQabs(freqTemp) > DIRECTION_THRESHOLD_PU){
                    COUNTER_reset(&self->directionCheckCount);
                    self->directionCheckState = DIRECTIONCHECK_CHECKING;
                    if(freqTemp > 0){
                        self->motorDirection = 0;//positive rpm
                    }
                    else{
                        self->motorDirection = 1;//negative rpm
                    }
                }
            }
            else if (self->directionCheckState == DIRECTIONCHECK_CHECKING)
            {
                if(((self->motorDirection == 0) && (freqTemp > DIRECTION_THRESHOLD_PU)) || ((self->motorDirection == 1) && (freqTemp < DIRECTION_THRESHOLD_PU))){//see if hold the rpm long enough
                    if(COUNTER_count_CTC(&self->directionCheckCount)){
                        self->directionCheckState = DIRECTIONCHECK_OK;
                    }
                }
                else{//cannot hold rpm
                    self->directionCheckState = DIRECTIONCHECK_NONE;
                }
            }
            else if (self->directionCheckState == DIRECTIONCHECK_OK)
            {//cannot hold rpm
                if(_IQabs(freqTemp) < (DIRECTION_THRESHOLD_LOW_PU)){
                    self->directionCheckState = DIRECTIONCHECK_NONE;
                }
            }
        }
        else
        {
            self->directionCheckState = DIRECTIONCHECK_NONE;
        }//if((self->pMotorVars->CtrlState == CTRL_State_OnLine) && (self->pMotorVars->EstState == EST_State_OnLine))

    }

#if REGENVOLTCTRL_ENABLE == 1
    TRAJ_setTargetValue(&self->regenVoltageTarget_traj, _IQ(self->regenVoltageTarget_V/USER_IQ_FULL_SCALE_VOLTAGE_V));
    TRAJ_run(&self->regenVoltageTarget_traj);//Id Iq transition ramp
#endif

#if INTTEMP_ENABLE == 1
    user_IntTemp_run(halHandle);
#endif

#if MODBUS_ENABLE == 1
    //watchdog
    if(self->powerOnErrorFreePassed)
    {
        if(!self->error.bit.com && COUNTER_count_SAT(&self->mbWatchdog))
        {
            if(self->protect.bit.com)
            {
                mb.holdingRegisters.reg0 = 0;//reset onOffSwitch
            }
            self->error.bit.com = true;//flag a communication fault
        }
        if((mb.holdingRegisters.reg19 != self->mbLastTS) && (mb.holdingRegisters.reg19 - self->mbLastTS <= 3))//reset watchdog counter if there is incremental heart beat
        {
            COUNTER_reset(&self->mbWatchdog);
            self->error.bit.com = false;
        }
        self->mbLastTS = mb.holdingRegisters.reg19;
    }

    //on off command by remote host
    if(self->onOffSwitch != mb.holdingRegisters.reg0)
    {
        self->onOffSwitch = mb.holdingRegisters.reg0;
        if(self->onOffSwitch)
        {//turn on, try to clear fault
            sys_clearError(self);
            COUNTER_reset(&self->mbWatchdog);
        }
    }
//    mb.holdingRegisters.reg1 = ;//Control Voltage
//    mb.holdingRegisters.reg2 = ;//Max Current
    mb.holdingRegisters.reg3 = _IQtoF(self->pAdcData->V.value[0]) * USER_IQ_FULL_SCALE_VOLTAGE_V * 10 + 0.5;//AC Input Voltage I1
    mb.holdingRegisters.reg4 = _IQtoF(self->pAdcData->V.value[1]) * USER_IQ_FULL_SCALE_VOLTAGE_V * 10 + 0.5;//AC Input Voltage I2
    mb.holdingRegisters.reg5 = _IQtoF(self->pAdcData->V.value[2]) * USER_IQ_FULL_SCALE_VOLTAGE_V * 10 + 0.5;//AC Input Voltage I3
//    mb.holdingRegisters.reg6 = _IQtoF(self->pAdcData->I.value[0]) * USER_IQ_FULL_SCALE_CURRENT_A * 10 + 0.5;//AC Input Current I1
//    mb.holdingRegisters.reg7 = _IQtoF(self->pAdcData->I.value[1]) * USER_IQ_FULL_SCALE_CURRENT_A * 10 + 0.5;//AC Input Current I2
//    mb.holdingRegisters.reg8 = _IQtoF(self->pAdcData->I.value[2]) * USER_IQ_FULL_SCALE_CURRENT_A * 10 + 0.5;//AC Input Current I3
    mb.holdingRegisters.reg6 = _IQtoF(self->dcA_peak) * USER_IQ_FULL_SCALE_CURRENT_A * 10 + 0.5;//peak DC current
    mb.holdingRegisters.reg7 = _IQtoF(self->thresholdDCOC_pu) * USER_IQ_FULL_SCALE_CURRENT_A * 10 + 0.5;//DCOC threshold
    mb.holdingRegisters.reg8 = 0;
    mb.holdingRegisters.reg9 = _IQtoF(FILTER_FO_get_y1(&self->dcV_filter)) * USER_IQ_FULL_SCALE_VOLTAGE_V * 10 + 0.5;//DC Output Voltage
    mb.holdingRegisters.reg10 = _IQtoF(FILTER_FO_get_y1(&self->dcA_filter)) * USER_IQ_FULL_SCALE_CURRENT_A * 10 + 0.5;//DC Output Current
    mb.holdingRegisters.reg11 = _IQtoF(FILTER_FO_get_y1(&self->dcV_filter)) * _IQtoF(FILTER_FO_get_y1(&self->dcA_filter)) * USER_IQ_FULL_SCALE_VOLTAGE_V * USER_IQ_FULL_SCALE_CURRENT_A / 100 + 0.5;//Input Power
    mb.holdingRegisters.reg12 = _IQtoF(FILTER_FO_get_y1(&self->dcV_filter)) * _IQtoF(FILTER_FO_get_y1(&self->dcA_filter)) * USER_IQ_FULL_SCALE_VOLTAGE_V * USER_IQ_FULL_SCALE_CURRENT_A / 100 + 0.5;//Output Power: actual kW x 10
    mb.holdingRegisters.reg13 = 95 * 100;//Power Factor 1
    mb.holdingRegisters.reg14 = 95 * 100;//Power Factor 2
    mb.holdingRegisters.reg15 = 95 * 100;//Power Factor 3
    mb.holdingRegisters.reg16 = self->relay_Contactor << 1;//Status 1: 0=none, 1=precharging, 2=precharge finish
    mb.holdingRegisters.reg17 = (1<<15 | 1<<5 | 1<<1 | 1<<0);//Status 2
    mb.holdingRegisters.reg18 = sys_getError(self);
//    mb.holdingRegisters.reg19 = ;//Communication Watchdog from remote host
    if(++mbCounter > self->runFreq_Hz)
    {
        mbCounter = 0;
        mb.holdingRegisters.reg20 += 1;//Communication Watchdog from rectifier
    }

    //update voltage target
    //update current limit when operating
#if SYS_PASSIVEMODE_ENABLE == 1
    if(self->startUpCheck.state >= STARTUPCHECK_FLYINGSTART)
    {
        self->thresholdDCOC_pu = _IQabs(_IQ(mb.holdingRegisters.reg2 / 10.0 / USER_IQ_FULL_SCALE_CURRENT_A));
    }
    else
    {
        self->thresholdDCOC_pu = _IQ(ERROR_THRESHOLD_DCOC_A / USER_IQ_FULL_SCALE_CURRENT_A);
    }
#endif

#endif

    DIGITALINPUT_run(&self->di1, HAL_readGpio(self->halHandle, GPIO_Number_16));
    LINEARINPUT_run(&self->userADC1, ADC_readResult(self->halHandle->adcHandle,ADC_ResultNumber_9));
    if(self->onOffSwitch != DIGITALINPUT_get_output(&self->di1))
    {
        self->onOffSwitch = DIGITALINPUT_get_output(&self->di1);
        if(self->onOffSwitch)
        {//turn on, try to clear fault
            sys_clearError(self);
        }
    }

}



//*****************************************************************************
//Timer 1
void sys_timer_init(SYS_Obj* const self){
    self->runFreq_Hz = SYS_RUNFREQ_HZ;
    self->runTick = 0;
    self->processBandwidthError = 0;
    TIMER_setDecimationFactor(self->halHandle->timerHandle[1],0);//TPR
    TIMER_setEmulationMode(self->halHandle->timerHandle[1],TIMER_EmulationMode_RunFree);//TCR
    TIMER_setPeriod(self->halHandle->timerHandle[1],((uint32_t)(self->pUserParams->systemFreq_MHz * 1000000.0 / self->runFreq_Hz) - 1));//PRD
    TIMER_setPreScaler(self->halHandle->timerHandle[1],0);//TIMERxTPR[TDDR]
    //set interrupt vector
    PIE_registerSystemIntHandler(self->halHandle->pieHandle, PIE_SystemInterrupts_TINT1, &TINT1ISR);
    //enable timer interrupt
    HAL_enableTimer1Int(self->halHandle);
    return;
}



interrupt void TINT1ISR(void) {
    // clear the Timer 1 interrupt flag
    TIMER_clearFlag(sys.halHandle->timerHandle[1]);
    sys.runTick++;

    return;
}// end of TINT1ISR() function



//*****************************************************************************
//Protection
void sys_protect_init(SYS_Obj* const self)
{
    self->powerOnErrorFreePassed = false;
    COUNTER_init(&self->powerOnErrorFreeCount,self->runFreq_Hz,5000);
    self->error.all = 0;
    self->protect.bit.dcov = PROTECT_DCOV_ENABLE;
    self->protect.bit.dcuv = PROTECT_DCUV_ENABLE;
    self->protect.bit.dcoc = PROTECT_DCOC_ENABLE;
    self->protect.bit.ooc_a = PROTECT_OOCA_ENABLE;
    self->protect.bit.ooc_b = PROTECT_OOCB_ENABLE;
    self->protect.bit.ooc_c = PROTECT_OOCC_ENABLE;
    self->protect.bit.gateFault_a = PROTECT_GFA_ENABLE;
    self->protect.bit.gateFault_b = PROTECT_GFB_ENABLE;
    self->protect.bit.gateFault_c = PROTECT_GFC_ENABLE;
    self->protect.bit.ctrl = PROTECT_CTRL_ENABLE;
    self->protect.bit.est = PROTECT_EST_ENABLE;
    self->protect.bit.com = PROTECT_COM_ENABLE;
    self->thresholdOOC_pu = _IQ(ERROR_THRESHOLD_OC_A / USER_IQ_FULL_SCALE_CURRENT_A);
#if SYS_IDCMEASURE_EXIST == 1
    self->thresholdDCOC_pu = _IQ(ERROR_THRESHOLD_DCOC_A / USER_IQ_FULL_SCALE_CURRENT_A);
#endif
    self->thresholdDCUV_pu = _IQ(ERROR_THRESHOLD_UV_V / USER_IQ_FULL_SCALE_VOLTAGE_V);
    COUNTER_init(&self->errorCountOOC[0],USER_ISR_FREQ_Hz,1);
    COUNTER_init(&self->errorCountOOC[1],USER_ISR_FREQ_Hz,1);
    COUNTER_init(&self->errorCountOOC[2],USER_ISR_FREQ_Hz,1);
    self->compHandle[0] = COMP_init((void *)COMP1_BASE_ADDR,sizeof(COMP_Obj));//point to the object
    self->compHandle[1] = COMP_init((void *)COMP2_BASE_ADDR,sizeof(COMP_Obj));//point to the object
#ifndef F2802xF
    Dself->compHandle[2] = COMP_init((void *)COMP3_BASE_ADDR,sizeof(COMP_Obj));//point to the object
#endif

#if PROTECT_COMP2_ENABLE == 1
    //overvoltage protection
    COMP_enable(self->compHandle[1]);
    COMP_setQualsel(self->compHandle[1], COMP_QualSel_Qual_5);
    COMP_enableSync(self->compHandle[1]);//choose qualification output
    COMP_setDacValue(self->compHandle[1], PROTECT_COMP2_DACVAL);//range 0 - 1023
#endif

#if PROTECT_DCAEVT1_ENABLE || PROTECT_DCBEVT1_ENABLE
    //Tripzone interrupt
    //refer to HAL_setupFaults for setting up tripzone
    //pwmHandle[0] is which ePWMx, check hal_init(). we will use that pwm channel to setup interrupt
    //1. Disable related interrupt
    //core level interrupt (INTM, IER, IFR) are clear in HAL_setParams.
    //PIE level interrupt (PIEIER, PIEIFR) are clear in HAL_setupPie.
    //disable peripheral level interrupt
    PWM_disableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_DCAEVT1);
    PWM_disableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_DCBEVT1);
    PWM_disableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_CBC);

    //2. clear peripheral level interrupt flags
    PWM_clearTripZone(halHandle->pwmHandle[0],PWM_TripZoneFlag_DCAEVT1);
    PWM_clearTripZone(halHandle->pwmHandle[0],PWM_TripZoneFlag_DCBEVT1);
    PWM_clearTripZone(halHandle->pwmHandle[0],PWM_TripZoneFlag_CBC);
    PWM_clearTripZone(halHandle->pwmHandle[0],PWM_TripZoneFlag_Global);

    //3. enable peripheral level interrupt
#if PROTECT_DCAEVT1_ENABLE == 1
    PWM_enableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_DCAEVT1);//configure TZEINT to generate a EPWM4_TZINT interrupt
#endif
#if PROTECT_DCBEVT1_ENABLE == 1
    PWM_enableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_DCBEVT1);//configure TZEINT to generate a EPWM4_TZINT interrupt
#endif
    PWM_enableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_CBC);

    //4. configure isr vector in PIE level
    //5. enable interrupt in pie level (PIEIER)
#if HAL_PWMA_BASE_ADDR == PWM_ePWM1_BASE_ADDR
    PIE_registerPieIntHandler(halHandle->pieHandle, PIE_GroupNumber_2, PIE_SubGroupNumber_1, &EPWM_TZINTISR);
    PIE_enablePwmTzInt(halHandle->pieHandle, PWM_Number_1);
#elif HAL_PWMA_BASE_ADDR == PWM_ePWM2_BASE_ADDR
    PIE_registerPieIntHandler(halHandle->pieHandle, PIE_GroupNumber_2, PIE_SubGroupNumber_2, &EPWM_TZINTISR);
    PIE_enablePwmTzInt(halHandle->pieHandle, PWM_Number_2);
#elif HAL_PWMA_BASE_ADDR == PWM_ePWM3_BASE_ADDR
    PIE_registerPieIntHandler(halHandle->pieHandle, PIE_GroupNumber_2, PIE_SubGroupNumber_3, &EPWM_TZINTISR);
    PIE_enablePwmTzInt(halHandle->pieHandle, PWM_Number_3);
#elif HAL_PWMA_BASE_ADDR == PWM_ePWM4_BASE_ADDR
    PIE_registerPieIntHandler(halHandle->pieHandle, PIE_GroupNumber_2, PIE_SubGroupNumber_4, &EPWM_TZINTISR);
    PIE_enablePwmTzInt(halHandle->pieHandle, PWM_Number_4);
#elif HAL_PWMA_BASE_ADDR == PWM_ePWM5_BASE_ADDR
    PIE_registerPieIntHandler(halHandle->pieHandle, PIE_GroupNumber_2, PIE_SubGroupNumber_5, &EPWM_TZINTISR);
    PIE_enablePwmTzInt(halHandle->pieHandle, PWM_Number_5);
#elif HAL_PWMA_BASE_ADDR == PWM_ePWM6_BASE_ADDR
    PIE_registerPieIntHandler(halHandle->pieHandle, PIE_GroupNumber_2, PIE_SubGroupNumber_6, &EPWM_TZINTISR);
    PIE_enablePwmTzInt(halHandle->pieHandle, PWM_Number_6);
#elif HAL_PWMA_BASE_ADDR == PWM_ePWM7_BASE_ADDR
    PIE_registerPieIntHandler(halHandle->pieHandle, PIE_GroupNumber_2, PIE_SubGroupNumber_7, &EPWM_TZINTISR);
    PIE_enablePwmTzInt(halHandle->pieHandle, PWM_Number_7);
#elif HAL_PWMA_BASE_ADDR == PWM_ePWM8_BASE_ADDR
    PIE_registerPieIntHandler(halHandle->pieHandle, PIE_GroupNumber_2, PIE_SubGroupNumber_8, &EPWM_TZINTISR);
    PIE_enablePwmTzInt(halHandle->pieHandle, PWM_Number_8);
#endif

    //6. enable interrupt in core level (IER)
    CPU_enableInt(halHandle->cpuHandle,CPU_IntNumber_2);//INT2
#endif
}



void sys_clearError(SYS_Obj* const self)
{
    self->error.all = 0;
    COUNTER_reset(&self->errorCountOOC[0]);
    COUNTER_reset(&self->errorCountOOC[1]);
    COUNTER_reset(&self->errorCountOOC[2]);
    sys_TZresume(self);
}



unsigned long sys_getError(SYS_Obj* const obj)
{
    return (obj->error.all & obj->protect.all);
}



void sys_TZresume(SYS_Obj* const self)
{
    //clear peripheral level interrupt flags
    int i = 0;
    HAL_Handle halHandle = self->halHandle;
    for(i=0;i<3;i++){
        PWM_clearTripZone(halHandle->pwmHandle[i],PWM_TripZoneFlag_DCAEVT1);
        PWM_clearTripZone(halHandle->pwmHandle[i],PWM_TripZoneFlag_DCBEVT1);
        PWM_clearTripZone(halHandle->pwmHandle[i],PWM_TripZoneFlag_CBC);
        PWM_clearTripZone(halHandle->pwmHandle[i],PWM_TripZoneFlag_Global);
    }
    //enable peripheral level interrupt
    PWM_enableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_DCAEVT1);//configure TZEINT to generate a EPWM4_TZINT interrupt
    PWM_enableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_DCBEVT1);//configure TZEINT to generate a EPWM4_TZINT interrupt
    PWM_enableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_CBC);
}



interrupt void EPWM_TZINTISR()
{
    HAL_Handle halHandle = sys.halHandle;
    //1. disable specific peripheral level interrupt
    if(halHandle->pwmHandle[0]->TZFLG & PWM_TripZoneFlag_DCAEVT1){
        PWM_disableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_DCAEVT1);
        sys.error.bit.dcov = true;
    }
    if(halHandle->pwmHandle[0]->TZFLG & PWM_TripZoneFlag_DCBEVT1){
        PWM_disableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_DCBEVT1);
        sys.error.bit.ooc_a = true;
        sys.error.bit.ooc_b = true;
        sys.error.bit.ooc_c = true;
    }
    if(halHandle->pwmHandle[0]->TZFLG & PWM_TripZoneFlag_CBC){
        PWM_disableTripZoneInt(halHandle->pwmHandle[0], PWM_TripZoneFlag_CBC);

    }
    //2. clear peripheral level, the global interrupt flag
    PWM_clearTripZone(halHandle->pwmHandle[0],PWM_TripZoneFlag_Global);
    //3. clear core level interrupt flag by PIEACK
    PIE_clearInt(halHandle->pieHandle,PIE_GroupNumber_2);//clear INT2 flag, the whole INT2 is TZ interrupt flag
}



//*****************************************************************************
//energisation
void sys_energize(SYS_Obj* const self)
{
    //run the PARK transform once in mainISR, it calculate Vd and Vq from the voltage feedback

    self->pwmEnergized = true;
//    PID_setUi(ctrlHandle->pidHandle_spd, _IQ(0.0));
    CTRL_setId_ref_pu(self->ctrlHandle, _IQ(0.0));
    CTRL_setIq_ref_pu(self->ctrlHandle, _IQ(0.0));
//    CTRL_setSpd_out_pu(ctrlHandle, _IQ(0.0));

    // seed Id controller prior to enabling the inverter
    PID_setUi(self->ctrlHandle->pidHandle_Id,gVdq_in.value[0]);

    //delete error
//        PID_setFbackValue(ctrlHandle->pidHandle_Id,PID_getRefValue(ctrlHandle->pidHandle_Id));

    // seed Iq controller prior to enabling the inverter
    PID_setUi(self->ctrlHandle->pidHandle_Iq,gVdq_in.value[1]);

    //delete error
//        PID_setFbackValue(ctrlHandle->pidHandle_Iq,PID_getRefValue(ctrlHandle->pidHandle_Iq));
//        CTRL_setIdq_in_pu(ctrlHandle,&Idq_out_pu_zero);

    //set output vector
    CTRL_setVdq_out_pu(self->ctrlHandle,&gVdq_in);

    //run the PARK transform second time, update pwm data
    CTRL_runOnLine_User(self->ctrlHandle,self->pAdcData,self->pPwmData);

    HAL_writePwmData(self->halHandle,self->pPwmData);// write the PWM compare values
    HAL_enablePwm(self->halHandle);
}



void sys_deenergize(SYS_Obj* const self)
{
    self->pwmEnergized = false;
    HAL_disablePwm(self->halHandle);
    self->pMotorVars->Flag_Run_Identify = false;
    START_UP_CHECK_set_NONE(&self->startUpCheck);

    //disconnect relay
    self->relay_Contactor = self->relay_Precharge = false;
#if SYS_PRECHARGE_PRECHARGERELAY_ENABLE == 1
    GPIO_setLow(self->halHandle->gpioHandle,SYS_PRECHARGE_PRECHARGERELAY_GPIO);
#endif
#if SYS_PRECHARGE_MAINRELAY_ENABLE == 1
    GPIO_setLow(self->halHandle->gpioHandle,SYS_PRECHARGE_MAINRELAY_GPIO);
#endif
}



void START_UP_CHECK_run(SYS_Obj* const self){
    START_UP_CHECK_Obj* suc = &self->startUpCheck;
    switch(suc->state){
    case STARTUPCHECK_NONE://0
         //open precharge relay if onoffswitch ON
        if(DIGITALINPUT_get_output(&suc->none)){
            self->relay_Precharge = true;
            suc->state = STARTUPCHECK_INITIAL;
        }
        break;
    case STARTUPCHECK_INITIAL://1
        //wait 3sec
        //check current. if dc bus voltage normal, no error, set run flag to true(but keep pwm off)
        // set voltage target to DC bus * 1.1
        if(DIGITALINPUT_get_output(&suc->initial)){
#if SYS_PASSIVEMODE_ENABLE == 1
            self->relay_Contactor = true;//rectifier without boost, turn on main relay if voltage is correct.
#endif
#if FLYING_START_ENABLE == 1
#if SYS_FAST_ENABLE == 1
            self->pMotorVars->Flag_Run_Identify = true;//it will compute offset calibration value.
#endif
            suc->state = STARTUPCHECK_FLYINGSTART;//change state
#else
            self->pMotorVars->Flag_Run_Identify = true;
#endif
        }
        break;
    case STARTUPCHECK_FLYINGSTART://2
        //wait for CTRL_State_OnLine and EST_State_OnLine, wait 5000ms, fire pwm
        if(DIGITALINPUT_get_output(&suc->flyingStart)){
#if SYS_PASSIVEMODE_ENABLE == 1
            self->relay_Precharge = false;
#endif
#if REGENVOLTCTRL_ENABLE == 1
            self->regenVoltageTarget_V = _IQtoF(FILTER_FO_get_y1(&self->dcV_filter)) * USER_IQ_FULL_SCALE_VOLTAGE_V * 1.01;
            TRAJ_setIntValue(&self->regenVoltageTarget_traj, _IQ(self->regenVoltageTarget_V/USER_IQ_FULL_SCALE_VOLTAGE_V));
#endif
#if FLYING_START_ENABLE && SYS_PWMFIRE_ENABLE
            self->pwmFire = true;
#endif
            //set current control kp ki to low value, then ramp it to normal value within 200ms
            suc->state = STARTUPCHECK_CONTROL;//change state
        }
        break;
      case STARTUPCHECK_CONTROL://3
          //wait 1 sec, open contactor
        if(DIGITALINPUT_get_output(&suc->control)){//(mechanicalFreq_pu_Abs < _IQ(50.0/60.0*USER_MOTOR_NUM_POLE_PAIRS/USER_IQ_FULL_SCALE_FREQ_Hz)
#if REGENVOLTCTRL_ENABLE == 1
            self->relay_Contactor = true;
            self->regenVoltageTarget_V = REGEN_VOLTAGE_TARGET;//slightly higher than idle DC voltage
#endif
            suc->state = STARTUPCHECK_FINISH;//change state
        }
        break;
    }

    if(!self->onOffSwitch){
        sys_deenergize(self);
    }
}



//*****************************************************************************
//*****************************************************************************
//Internal Temperature Sensor
#if INTTEMP_ENABLE == 1
#ifdef F2802xF
//Slope of temperature sensor (deg. C / ADC code, fixed pt Q15 format)
#define getTempSlope() (*(int (*)(void))0x3D7E80)()
//ADC code corresponding to temperature sensor output at 0-degreesC
#define getTempOffset() (*(int (*)(void))0x3D7E83)()
#else
//Slope of temperature sensor (deg. C / ADC code, fixed pt Q15 format)
#define getTempSlope() (*(int (*)(void))0x3D7E82)()
//ADC code corresponding to temperature sensor output at 0-degreesC
#define getTempOffset() (*(int (*)(void))0x3D7E85)()
#endif

void user_IntTemp_init(){
    self->intTempScale = _IQ15toF((_iq)getTempSlope());
    self->intTempOffset = getTempOffset();
    self->intTemp = 0;

    self->heatsinkTemp = 0;
}


//! \brief Reads the internal temperature sensor
//! \param[in] handle The hardware abstraction layer (HAL) handle
//! \return The sensor value from _IQ(-1.0) to _IQ(1.0)
void user_IntTemp_run(HAL_Handle handle)
{
    float ntcResistance;
    float steinhart;
    HAL_Obj *obj = (HAL_Obj *)handle;
    //Convert raw temperature sensor output to a temperature (degC)
    self->intTemp = (ADC_readResult(obj->adcHandle,ADC_ResultNumber_8) - self->intTempOffset) * self->intTempScale;

    //heat sink temperature
    ntcResistance = SERIESRESISTOR / ( 4095.0 / ADC_readResult(obj->adcHandle,ADC_ResultNumber_9) - 1);// 10k / (4095 / ADC -1)
    steinhart = (1.0 / (TEMPERATURENOMINAL + 273.15)) + (log(ntcResistance / THERMISTORNOMINAL) / BCOEFFICIENT);     // (1/To) + ln(R/Ro)/B
    steinhart = 1.0 / steinhart;//invert to find T
    steinhart -= 273.15;// convert K to C
    self->ntcResistance = ntcResistance;
    self->steinhart = steinhart;

    self->heatsinkTemp = steinhart;

//    if(self->heatsinkTemp > 70.0){
//        set_onOffSwitch(&self->protection, false);
//    }

} // end of HAL_readPotentiometerData() function


float user_IntTemp_get_intTemp(){
    return self->intTemp;
}
#endif

//*****************************************************************************
//*****************************************************************************
//*****************************************************************************
