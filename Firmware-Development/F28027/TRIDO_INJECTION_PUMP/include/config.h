//20220305
#ifndef _CONFIG_H_
#define _CONFIG_H_

//HAL
#define HAL_PWMA_BASE_ADDR PWM_ePWM1_BASE_ADDR

//main.c
#define SYSTEMTASK_ENABLE 0
#define GUICONTROL_ENABLE 1
#define SVGENCURRENT_ENABLE 1
#define FLYING_START_ENABLE 0

//user.c
#define CURRENT_KP_STIFFNESS_FACTOR (0.25)

//system.c
#define SYS_RUNFREQ_HZ (500.0)
#define SYS_AUTOONOFFBYVOLTAGE_ENABLE 0
#define SYS_PWMFIRE_ENABLE 1
#define SYS_FAST_ENABLE 1
#define SYS_PASSIVEMODE_ENABLE 0
#define ERROR_THRESHOLD_OV_V (40.0)
#define ERROR_THRESHOLD_UV_V (2.0)
#define ERROR_THRESHOLD_DCOC_A (23.0)
#define ERROR_THRESHOLD_OC_A (23.0)
#define PROTECT_COMP2_ENABLE 0
#define PROTECT_COMP2_DACVAL (ERROR_THRESHOLD_OV_V / USER_ADC_FULL_SCALE_VOLTAGE_V * 1023 + 0.5)
#define PROTECT_DCAEVT1_ENABLE 0//digital comparator
#define PROTECT_DCBEVT1_ENABLE 0
#define PROTECT_DCOV_ENABLE 1//DC bus over voltage
#define PROTECT_DCUV_ENABLE 0//DC bus under voltage
#define PROTECT_DCOC_ENABLE 0//DC bus over current
#define PROTECT_OOCA_ENABLE 0//phase A output over current
#define PROTECT_OOCB_ENABLE 0//phase B output over current
#define PROTECT_OOCC_ENABLE 0//phase C output over current
#define PROTECT_GFA_ENABLE 0//phase A gate fault
#define PROTECT_GFB_ENABLE 0//phase B gate fault
#define PROTECT_GFC_ENABLE 0//phase C gate fault
#define PROTECT_GFA_GPIO GPIO_Number_52
#define PROTECT_GFB_GPIO GPIO_Number_51
#define PROTECT_GFC_GPIO GPIO_Number_50
#define PROTECT_CTRL_ENABLE 0
#define PROTECT_EST_ENABLE 0
#define PROTECT_COM_ENABLE 0

//precharge
#define SYS_PRECHARGE_PRECHARGERELAY_ENABLE 0
#define SYS_PRECHARGE_MAINRELAY_ENABLE 0
#define SYS_PRECHARGE_PRECHARGERELAY_GPIO GPIO_Number_11
#define SYS_PRECHARGE_MAINRELAY_GPIO GPIO_Number_10

#define SYSTEM_ON_VOLTAGE (10.0)//auto on off based on voltage
#define SYSTEM_OFF_VOLTAGE (2.0)
#define DIRECTION_THRESHOLD_PU _IQ(3000.0/60.0*USER_MOTOR_NUM_POLE_PAIRS/USER_IQ_FULL_SCALE_FREQ_Hz)
#define DIRECTION_THRESHOLD_LOW_PU _IQ(1800.0/60.0*USER_MOTOR_NUM_POLE_PAIRS/USER_IQ_FULL_SCALE_FREQ_Hz)
#define SYS_START_NONE_MS (100)
#define SYS_START_INITIAL_MS (UNIT*100)
#define SYS_START_FS_MS (100)
#define SYS_START_CTRL_MS (100)

//data
#define SYS_IDCMEASURE_EXIST 0

//regen control
#define REGENVOLTCTRL_ENABLE 0
#define REGEN_VOLTAGE_TARGET (850.0)
#define REGEN_VOLTAGE_MAX (1000.0)
#define REGEN_VOLTAGE_MIN (850.0)

//modbus
#define MODBUS_ENABLE 0
#define MODBUS_WDPERIOD_MS (15000.0)

//temperature sensors
#define INTTEMP_ENABLE 0
#define EXTTEMP_ENABLE 0
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000

//setting we do not touch
#if SYS_FAST_ENABLE == 0
#define SYS_PWMFIRE_ENABLE 0
#endif

//temporary
#define FAULT_CASCADE 0
#define UNIT 1

//#ifdef F2802xF
//
//#else
//
//#endif

#endif
