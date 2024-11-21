#ifndef _MODBUS_USER_H_
#define _MODBUS_USER_H_

#include "hal_obj.h" // System Include
#include "userio.h"

#define MODBUS_BAUDRATE            SCI_BaudRate_115_2_kBaud
#define TOTAL_PACKETS              25
#define HOLDREG_IN_BASE_ADDRESS    0
#define HOLDREG_OUT_BASE_ADDRESS   16

#define AIN1                       hal.adcHandle->ADCRESULT[10]
#define AIN2                       hal.adcHandle->ADCRESULT[11]
#define AIN3                       hal.adcHandle->ADCRESULT[12]
#define AIN4                       hal.adcHandle->ADCRESULT[13]

#define IDC_ADC_RESULT             hal.adcHandle->ADCRESULT[9]
#define TANK_PRESSURE_ADC_RESULT   AIN2

#define PRESSURE_SENSOR_OFFSET     5
#define MAXIMUM_RPM                400.0L

#define VOLTAGE_CORRECTION_FACTOR  1.0L

struct ERROR_DATA_BIT_T
{
  bool slowdown:1;
  bool ocpwrn:1;
  bool ocperr:1;
  bool ovperr:1;
  bool uvperr:1;
  bool snserr:1;
  bool rpmerr:1;
  bool settingerr:1;
  bool esterr:1;
  bool rsvd:7;
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
  bool sensor:1;
  bool comfail:1;
  bool others:1;
  bool rsvd:10;
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
  bool rsvd:14;
};

union STATUS_T
{
  struct STATUS_BIT_T bit;
  uint16_t all;
};

extern union STATUS_T stat;

//Pressure sensor settings
typedef struct
{
  uint16_t max_psi_sensor;
  uint16_t zero_psi_adc_reading;
  uint16_t max_psi_adc_reading;
  uint16_t err_psi_adc_reading_low;
  uint16_t err_psi_adc_reading_high;
  uint16_t max_start_psi;
  uint16_t max_stop_psi;
} SENSOR_T;

extern SENSOR_T sensor;

// Control Parameters Send To VFD from ESP32
typedef struct
{
  int16_t  speed_in;       // Set speed +/- 1000rpm
  int16_t  speed_out;      // Running speed +/- 1000rpm
  int16_t  dcvoltage_out;  // Controller DC voltage
  int16_t  dccurrent_out;  // Controller DC current
  float dccurrent_out_f;
  int16_t  motor_current_out;  // Motor ph-ph current converted to rms
  uint16_t onoff_in;       // 1 = ON, 0 = OFF
  uint16_t onoff_out;      // 1 = ON, 0 = OFF
  uint16_t mode;           // 2 = DIO, 1 = LOCAL, 0 = REMOTE
  uint16_t communication_fail_action;
  uint16_t prev_heartbeat_in;  // Previous heartbeat
  uint16_t heartbeat_in;      // ESP32 heartbeat
  uint16_t heartbeat_out; // Controller heartbeat increment at 1000ms interval
  uint16_t min_dcv_input; // Minimum DCV input set
  uint16_t max_dcv_input; // Maximum DCV input set
  uint16_t slow_down_dcv; // Slow down DCV
  uint16_t ocp_dcamp_set;     // Current Limit Ratio
  float motor_current_running;
  float ocp_dcamp_set_f;
  uint16_t start_psi;     // Start pressure
  uint16_t stop_psi;      // Stop pressure
  uint16_t tank_pressure; // Tank pressure
  uint16_t zero_psi_offset; // Pressure sensor zero PSI offset, 9 = -1psi, 10 = 0psi, 11 = 1psi...
  uint16_t pressure_control_method; //[0=STOP MOTOR, 1=DOUT1 OPEN, 2=DOUT1 CLOSE]
  uint16_t max_psi_sensor;
  uint16_t zero_psi_sensor_volt;
  uint16_t max_psi_sensor_volt;
  union IO_T io;          // Digital input/output states
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

void modbus_setup(void);
void modbus_loop(void);

#endif /* _MODBUS_USER_H_ */
