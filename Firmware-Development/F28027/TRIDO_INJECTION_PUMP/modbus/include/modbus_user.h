#ifndef _MODBUS_USER_H_
#define _MODBUS_USER_H_

#include "hal_obj.h" // System Include
#include "userio.h"

#define MODBUS_BAUDRATE            SCI_BaudRate_115_2_kBaud
#define TOTAL_PACKETS              24 // 0 - 23
#define HOLDREG_IN_BASE_ADDRESS    0
#define HOLDREG_OUT_BASE_ADDRESS   14

#define AIN1                       hal.adcHandle->ADCRESULT[10]
#define AIN2                       hal.adcHandle->ADCRESULT[11]
#define AIN3                       hal.adcHandle->ADCRESULT[12]
#define AIN4                       hal.adcHandle->ADCRESULT[13]

#define IDC_ADC_RESULT             hal.adcHandle->ADCRESULT[9]

#define VOLTAGE_CORRECTION_FACTOR  1.0L

struct UINT16_T_BIT_T
{
  uint16_t BYTE0:8;
  uint16_t BYTE1:8;
};

union UINT16_T
{
  struct UINT16_T_BIT_T bit;
  uint16_t all;
};

struct TEMP_CTRL_BIT_T
{
  bool unit:1;  // 0 = deg C, 1 = F
  bool onoff:1; // 0 = ON, 1 = OFF
  uint16_t rsvd:14;
};

union TEMP_CTRL_T
{
  struct TEMP_CTRL_BIT_T bit;
  uint16_t all;
};

struct ERROR_DATA_BIT_T
{
  bool slowdown:1;
  bool ocpwrn:1;
  bool ocperr:1;
  bool ovperr:1;
  bool uvperr:1;
  bool rpmerr:1;
  bool settingerr:1;
  bool esterr:1;
  uint16_t rsvd:8;
};

union ERROR_DATA_T
{
  struct ERROR_DATA_BIT_T bit;
  uint16_t all;
};

extern union ERROR_DATA_T error;

struct FAULT_BIT_T
{
  bool ocp:1;
  bool ovp:1;
  bool uvp:1;
  bool comfail:1;
  bool others:1;
  uint16_t rsvd:11;
};

union FAULT_T
{
  struct FAULT_BIT_T bit;
  uint16_t all;
};

extern union FAULT_T fault;

struct STATUS_BIT_T
{
  bool onoff:1;
  bool dir:1;
  uint16_t rsvd:14;
};

union STATUS_T
{
  struct STATUS_BIT_T bit;
  uint16_t all;
};

extern union STATUS_T stat;

// Control Parameters Send To VFD from ESP32
typedef struct
{
  int16_t  speed_in;           // Set speed +/- 1000rpm
  int16_t  speed_out;          // Running speed +/- 1000rpm
  int16_t  dcvoltage_out;      // Controller DC voltage
  int16_t  dccurrent_out;      // Controller DC current
  float dccurrent_out_f;
  int16_t  motor_current_out;  // Motor ph-ph current converted to rms
  uint16_t onoff_in;           // 1 = ON, 0 = OFF
  uint16_t onoff_out;          // 1 = ON, 0 = OFF
  uint16_t mode;               // 2 = DIO, 1 = LOCAL, 0 = REMOTE
  uint16_t communication_fail_action;
  uint16_t prev_heartbeat_in;  // Previous heartbeat
  uint16_t heartbeat_in;       // ESP32 heartbeat
  uint16_t heartbeat_out;      // Controller heartbeat increment at 1000ms interval
  uint16_t min_dcv_input;
  uint16_t max_dcv_input;
  uint16_t slow_down_dcv;
  uint16_t ocp_dcamp_set;
  int16_t off_temperature;     // Off temperature
  int16_t on_temperature;      // On temperature
  int16_t temperature_now;     // Current temperature
  union TEMP_CTRL_T tempctrl;  // Temperature On/Off, unit
  float motor_current_running;
  float ocp_dcamp_set_f;
  union IO_T io;               // Digital input/output states
  union UINT16_T motor_model;  // 0-3 Model model 0 = 25, 1 = 35, 2 = 50, 3 = 70, 4 = GMI
  uint16_t reset;              // controller reset
  uint16_t reset_ok;           // controller reset_ok
} MODBUS_MSG_T;

extern MODBUS_MSG_T mbmsg;

// Timeout Variables
typedef struct
{
  // MODBUS T15 timeout
  uint32_t t15Count;
  bool t15TimeoutFlag;
  // MODBUS T35 timeout
  uint32_t t35Count;
  // heartbeat 1s out
  uint32_t hearbeatCount;
  // communication timeout
  uint32_t commCount;
  bool commTimeoutFlag;
  // warning current timeout
  uint32_t timer2minCount;
  // over current timeout
  uint32_t timer15sCount;
} TIMEOUT_T;

extern TIMEOUT_T to;
extern uint16_t mtrModel;
extern uint16_t brdSeries;

void modbus_setup(void);
void modbus_loop(void);

#endif /* _MODBUS_USER_H_ */
