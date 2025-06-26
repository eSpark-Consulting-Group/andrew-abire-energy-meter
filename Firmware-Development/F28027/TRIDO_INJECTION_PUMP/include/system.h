#ifndef _SYSTEM_H_
#define _SYSTEM_H_
/*
 * 20220307
 * sync with 27&69
 */
//*****************************************************************************
// the includes
//*****************************************************************************
#include "../include/comp.h"//COMP_Handle
#include "../include/config.h"
#include "../include/counter.h"//poweronErrorClearCount
#include "../include/digitalInput.h"
#include "../include/limiter.h"
#include "../include/linearInput.h"
#include "../include/main.h"//HAL_Handle, MOTOR_Vars_t, iqmath, USER_Params
#include "../include/traj.h"
#include "../include/traj_heterogeneous.h"



//*****************************************************************************
// #defines
//*****************************************************************************



//*****************************************************************************
// typedefs
//*****************************************************************************
struct error_bits
{//1 = error, 0 = normal
//  unsigned char esd:1;//emergency stop
  unsigned char dcov:1;//DC bus over voltage
  unsigned char dcuv:1;//DC bus under voltage
  unsigned char dcoc:1;//DC bus over current
  unsigned char ooc_a:1;//phase A
  unsigned char ooc_b:1;//phase B
  unsigned char ooc_c:1;//phase C
  unsigned char gateFault_a:1;//phase A gate driver fault
  unsigned char gateFault_b:1;//phase B gate driver fault
  unsigned char gateFault_c:1;//phase C gate driver fault
  unsigned char ctrl:1;
  unsigned char est:1;
  unsigned char com:1;//communication fault
};



typedef union error_union
{
    unsigned long all;
    struct error_bits bit;
}error_union;



typedef enum
{
  STARTUPCHECK_NONE=0,//disable force angle, set current control kp ki to low value
  STARTUPCHECK_INITIAL,//check dcbus,and errors
  STARTUPCHECK_FLYINGSTART,//enable pwm, set current control kp ki to normal value within 200ms
  STARTUPCHECK_CONTROL,//check regen, throttle
  STARTUPCHECK_FINISH//everything ok. enable throttle, enable force angle
} StartUpCheck_State_e;



typedef struct _START_UP_CHECK_Obj_{
    StartUpCheck_State_e state;
    DIGITALINPUT_Obj none;
    DIGITALINPUT_Obj initial;
    DIGITALINPUT_Obj flyingStart;
    DIGITALINPUT_Obj control;
}START_UP_CHECK_Obj;



typedef enum
{
  DIRECTIONCHECK_NONE=0,
  DIRECTIONCHECK_CHECKING,
  DIRECTIONCHECK_OK
} directionCheck_State_e;



typedef struct _SYS_Obj_
{
    //external object pointer
    HAL_Handle halHandle;
    CTRL_Handle ctrlHandle;
    USER_Params* pUserParams;
    volatile MOTOR_Vars_t* pMotorVars;
    const HAL_AdcData_t* pAdcData;
    HAL_PwmData_t* pPwmData;

    //task scheduling
    unsigned int runFreq_Hz;//timer 1 frequency
    unsigned char runTick;//timer 1 task scheduling, do task when timerTick > 1, after that --timerTick
    bool processBandwidthError;//indicate mcu processing bandwidth is not enough

    //protection
    error_union protect;//switch protection on/off
    error_union error;
    bool powerOnErrorFreePassed;//true = passed the error free period. false = disable error detection at power on
    COUNTER_Obj powerOnErrorFreeCount;
    _iq thresholdOOC_pu;//output over current threshold
#if SYS_IDCMEASURE_EXIST == 1
    _iq thresholdDCOC_pu;//DC over current threshold
#endif
    _iq thresholdDCUV_pu;
    COUNTER_Obj errorCountOOC[3];
    COMP_Handle compHandle[3];//f28069 have 3 comp, f28027 have 2 comp

    //energisation
    bool pwmFire;//set 1 to fire pwm, it will set to 0 automatically
    bool pwmEnergized;//indicate whether pwm is energized
    bool relay_Precharge;
    bool relay_Contactor;
    START_UP_CHECK_Obj startUpCheck;
    bool onOffSwitch;
    _iq systemOnVoltage_pu;
    _iq systemOffVoltage_pu;

    //Id Iq control
    _iq currentMax_pu;
    _iq IsRef_pu;
    _iq IqRef_pu;
    _iq IdRef_pu;

    //filter objects
    FILTER_FO_Obj freq_filter;
    FILTER_FO_Obj dcV_filter;
#if SYS_IDCMEASURE_EXIST == 1
    FILTER_FO_Obj dcA_filter;
#endif

    //data
#if SYS_IDCMEASURE_EXIST == 1
    _iq dcA_peak;
#endif

    //regen control
    directionCheck_State_e directionCheckState;
    COUNTER_Obj directionCheckCount;
    bool motorDirection;//determine which direction to apply reverse torque for regeneration

#if REGENVOLTCTRL_ENABLE == 1
    //regen pid
    float regenVoltageTarget_V;
    TRAJ_Obj regenVoltageTarget_traj;
    _iq regenVoltagePIDout;
    PID_Obj pid_dcv;
#endif

#if INTTEMP_ENABLE == 1
    //internal temperature sensor
    float intTempScale;
    float intTempOffset;
    float intTemp;
#endif
#if EXTTEMP_ENABLE == 1
    //external temperature sensor
    float heatsinkTemp;
    float ntcResistance;
    float steinhart;
#endif

    //IO
    DIGITALINPUT_Obj di1;
    LINEARINPUT_Obj userADC1;

#if MODBUS_ENABLE == 1
    unsigned int mbLastTS;//last Timestamp from remote host
    COUNTER_Obj mbWatchdog;//communication watchdog counter
#endif
}SYS_Obj;



//*****************************************************************************
// globals
//*****************************************************************************
extern SYS_Obj sys;



//*****************************************************************************
// the function prototypes
//*****************************************************************************
void sys_init(SYS_Obj* const self, HAL_Handle halHandle, CTRL_Handle ctrlHandle, USER_Params *pUserParams, volatile MOTOR_Vars_t *pMotorVars, const HAL_AdcData_t *pAdcData, HAL_PwmData_t *pPwmData);//put it before HAL_enableGlobalInts
void sys_loop(SYS_Obj* const self);//background function
void sys_isrFollower(SYS_Obj* const self);//
void sys_isr(SYS_Obj* const self);//put in main isr
void sys_run(SYS_Obj* const self);//timed function

//Timer 1
void sys_timer_init(SYS_Obj* const self);
interrupt void TINT1ISR(void);



//protection
void sys_protect_init(SYS_Obj* const self);
void sys_clearError(SYS_Obj* const self);
unsigned long sys_getError(SYS_Obj* const self);
void sys_TZresume(SYS_Obj* const self);
interrupt void EPWM_TZINTISR();



//energisation
void sys_energize(SYS_Obj* const self);
void sys_deenergize(SYS_Obj* const self);

inline void START_UP_CHECK_init(START_UP_CHECK_Obj* obj, unsigned int taskFreq_Hz, float none_ms, float initial_ms, float flyingStart_ms, float control_ms){
    obj->state = STARTUPCHECK_NONE;
    DIGITALINPUT_init(&obj->none, taskFreq_Hz, none_ms, RisingEdge);
    DIGITALINPUT_init(&obj->initial, taskFreq_Hz, initial_ms, RisingEdge);
    DIGITALINPUT_init(&obj->flyingStart, taskFreq_Hz, flyingStart_ms, RisingEdge);
    DIGITALINPUT_init(&obj->control, taskFreq_Hz, control_ms, RisingEdge);

}

inline void START_UP_CHECK_set_NONE(START_UP_CHECK_Obj* obj){
    obj->state = STARTUPCHECK_NONE;
}

inline StartUpCheck_State_e START_UP_CHECK_get_state(START_UP_CHECK_Obj* obj){
    return obj->state;
}

void START_UP_CHECK_run(SYS_Obj* const self);


//Internal temperature sensor
#if INTTEMP_ENABLE == 1
void user_IntTemp_init();
void user_IntTemp_run(HAL_Handle handle);
float user_IntTemp_get_intTemp();
#endif

#endif//_SYSTEM_H_
