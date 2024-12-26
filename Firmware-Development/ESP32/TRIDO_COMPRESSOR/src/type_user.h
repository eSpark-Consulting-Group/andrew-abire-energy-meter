#ifndef TYPE_USER_H
#define TYPE_USER_H

#include <Arduino.h>

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t

#include "define_user.h"

typedef union {
  int all;
  u8 byte[2];
} INT16_T;

typedef union {
  u16 all;
  u8 byte[2];
} UINT16_T;

typedef union {
  u32 all;
  u8 byte[4];
} UINT32_T;

// MENU
typedef struct
{
  u8 this_item;
  char (*pFunc)(char input);
} MENU_FUNC_T;

typedef struct
{
  u8 this_item;
  u8 input;
  u8 next_item;
} MENU_T;

// Date & Time
typedef struct
{
  u16 year;
  u16 month;
  u16 day;
  u16 hour;
  u16 minute;
  u16 second;
} DATETIME_T;

struct IO_BIT_T
{
  bool DI1 : 1;
  bool DI2 : 1;
  bool DI3 : 1;
  bool DI4 : 1;
  bool DO1 : 1;
  bool DO2 : 1;
  bool DO3 : 1;
  bool DO4 : 1;
  bool rsvd : 8;
} __attribute__((packed));

union IO_T
{
  struct IO_BIT_T bit;
  uint16_t all;
};

// Control Parameters Send To VFD
typedef struct
{
  INT16_T rpm;               //Set rpm +/- 1000rpm
  bool onoff = OFF;          //ON/OFF
  bool reset = false;        //Controller reset request
  bool reset_ok = false;     //Controller reset response
  u8 mode = LOCAL_MODE;      //DIO/LOCAL/REMOTE
  UINT16_T motor_model;      // byte[0] Motor model number 0 = 25, 1 = 35, 2 = 50, 3 = 70, 4 = GMI
                             // byte[1] Board number 0 = A(0.002), 1 = D(0.005)
  bool communication_fail_action = 0;  //0 = Keep motor's last state, 1 = Stop motor
  u16 count = 0;             //Heartbeat out increment at 1000ms interval
  u16 prev_count = 0;        //Last heartbeat count
  UINT16_T min_dcv_input;    //Minimum DC input voltage
  UINT16_T max_dcv_input;    //Maximum DC input voltage
  UINT16_T slow_down_dcv;    //Motor slow down belows/equals to this DC input voltage
  double min_dcv_input_f = 21.0;    //
  double max_dcv_input_f = 32.0;    //
  double slow_down_dcv_f = 23.0;    //
  UINT16_T ocp_dcamp;        //Over current ratio in DC amp of full load currrent
  double ocp_dcamp_f = 8.0;
  IO_T gpio;
} CONTROL_T;

// Compressor
typedef struct
{
  INT16_T tank_pressure;     //Tank pressure
  UINT16_T start_pressure;     //Start pressure
  UINT16_T stop_pressure;      //Stop pressure
  u32 runtime_total = 0;     //Total on time in terms of number of cycle time
  u32 cycletime_total = 0;   //Total period of in terms of number of ycle time
  UINT16_T runtime_day{.all = 0};  // Total runtime in days
  u8 runtime_hour = 0;       //Total runtime in hours
  u8 runtime_min = 0;        //Total runtime in mins
  u8 runtime_sec = 0;        //Total runtime in secs
  UINT16_T cycletime_day{.all = 0}; // Total cycletime in days
  u8 cycletime_hour = 0;     //Total cycletime hours in days
  u8 cycletime_min = 0;      //Total cycletime mins in days
  u8 cycletime_sec = 0;      //Total cycletime secs in days
  u8  duty = 0;              //Duty of motor = on time / (on time + off time)
  UINT16_T set_cycletime;    //Scaled up x10 e.g. 5 = 0.5hour
  double set_cycletime_f = 0.5; 
  u8 ram_record[NO_OF_RAM_RECORD]; //NO_OF_SAVED_RECORD records per set_cycletime
  u8 saved_record[NO_OF_SAVED_RECORD]; //NO_OF_SAVED_RECORD records per set_cycletime
  u8 ram_record_index = 0;
  u8 saved_record_index = 0;
  bool duty_record_reset = 0; //duty records reset bit
  u8 sensor_psi_offset = 5;  //set the pressure sensor PSI offset value, 5 = 0 PSI offset, 4 = -1 PSI, 6 = 1 PSI...
  UINT16_T single_duty_alarm_percent;
  UINT16_T average_duty_alarm_percent;
  bool autoreset_motorturns = 0;  // 1 = reset at 00:00:00, 0 = manual reset
  bool disable_dio_mode = 1; //1 = disable
  u8 pressure_control_method = 0; //[0=STOP MOTOR, 1=DOUT3 OPEN, 2=DOUT3 CLOSE]
  bool lock_factory_mode = 1;
  u8 unlock_factory_times_left = 10;
  UINT16_T max_psi_sensor;
  UINT16_T zero_psi_sensor_volt;
  UINT16_T max_psi_sensor_volt;  
} COMPRESSOR_T;

// Status Read from VFD
typedef struct
{
  INT16_T rpm;                //Read rpm
  INT16_T rpm_min;            //min rpm for keypad input
  INT16_T rpm_max;            //max rpm for keypad input
  bool onoff = 0;             //Read 1 = ON, 0 = OFF
  INT16_T dcv;                //Read DC Voltage
  INT16_T dca;                //Read DC Current
  INT16_T motora;             //Motor phase-phase current converted to rms
  double dcv_f = 0.0;          //
  double dca_f = 0.0;          //
  double motora_f = 0.0;       //
  u16 count = 0;              //Heartbeat in at 1000ms interval
  u16 totalturns_lsb = 0;     //lower 16 bits of 32-bit totalturns
  u16 totalturns_msb = 0;     //higher 16 bits of 32-bit totalturns
  COMPRESSOR_T motor;         
} STATUS_T;

// Temporary variables for send/receive
typedef struct
{
  INT16_T rpm_in;             //Read rpm
  INT16_T rpm_out;            //Set rpm
} TEMP_T;

// SCADA Modbus Settings
typedef struct
{
  u8 protocol = 0;            //0=RTU, 1=ASCII, 2=TCP
  u8 server_id = 1;           //Server ID (i.e. Slave Address)
  UINT16_T timeout;           //Timeout in ms [100 to 10000 ms], wait before generating an error for an unanswered Modbus request.
  UINT16_T coil_addr;         //Set the Output Coil Start Address
  UINT16_T discrete_addr;     //Set the Discrete Input Start Address
  UINT16_T inputreg_addr;     //Set the Input Register Start Address
  UINT16_T holdreg_addr;      //Set the Holding Register Start Address
  u16 max_coil = 9;           //For calculation of maxmimum user defined address
  u16 max_discrete = 0;       //For calculation of maxmimum user defined address
  u16 max_inputreg = 0;       //For calculation of maxmimum user defined address
  u16 max_holdreg = 127;      //For calculation of maxmimum user defined address
} SCADA_SETTING_T;

// SCADA Serial Port Config
struct COM_CONFIG_BIT_T
{ 
  u8 parity:2;
  u8 databits:2;
  u8 stopbits:2;
  u8 rsvd:2;  
};

union COM_CONFIG_T
{
  struct COM_CONFIG_BIT_T bit;
  u8 all;
};

// SCADA Serial Port Settings
typedef struct
{
  COM_CONFIG_T config;
  UINT32_T baud;             //Baudrate [9600, 19200, 38400, 57600, and 115200]
  u8 databits = 8;           //Data bits [RTU=8, ASCII=(default7)/8]
  u8 stopbits = 1;           //No of Stop bits [1,2]
  u8 parity = 0;             //Parity [none = 0, odd = 1, even = 2]
} COM_T;

// System Error bits (set = error)
struct ERROR_BIT_T
{ 
  bool over_voltage_err:1;       //0
  bool under_voltage_err:1;      //1
  bool over_current_err:1;       //2
  bool setting_value_err:1;      //3
  bool data_range_err:1;         //4
  bool vfd_heartbeat_err:1;      //5
  bool vfd_est_err:1;            //6
  bool rpm_set_low_err:1;        //7
  bool pressure_sensor_error:1;  //8 
  bool rsvd1:1;                  //9
  bool rsvd2:1;                  //10
  bool rsvd3:1;                  //11
  bool rsvd4:1;                  //12
  bool rsvd5:1;                  //13
  bool rsvd6:1;                  //14
  bool rsvd7:1;                  //15
};

union SYSTEM_ERR_T
{
  struct ERROR_BIT_T bit;
  u16 all;
  u8 byte[2];
};

// System status/warning bits
struct STATUS_BIT_T
{ 
  bool system_onoff_status:1;      //0
  bool motor_onoff_status:1;       //1
  bool low_voltage_warning:1;      //2
  bool over_current_warning:1;     //3
  bool single_duty_high_warning:1; //4
  bool average_duty_high_warning:1;//5
  bool reboot_pending:1;           //6
  bool rsvd1:1;                    //7
  bool rsvd2:1;                    //8
  bool rsvd3:1;                    //9
  bool rsvd4:1;                    //10
  bool rsvd5:1;                    //11
  bool rsvd6:1;                    //12
  bool rsvd7:1;                    //13
  bool rsvd8:1;                    //14
  bool rsvd9:1;                    //15
};

union SYSTEM_STATUS_T
{
  struct STATUS_BIT_T bit;
  u16 all;
  u8 byte[2];
};

typedef struct
{
  DATETIME_T dnt;              //Date & Time that the fault occured
  SYSTEM_ERR_T err;            //error bits
  SYSTEM_STATUS_T status;      //status bits
  //INT16_T rpm;               //Read rpm
  //INT16_T dcv;               //Read DC Voltage
  //INT16_T dca;               //Read DC Current
  //INT16_T motora;            //Motor phase-phase current converted to rms
} HISTORY_T;

// EEPROM
typedef struct
{
  DATETIME_T dnt;            //Date & Time
  CONTROL_T ctrl;            //VFD
  COMPRESSOR_T motor;        //Compressor
  UINT32_T totalturns;       //Accumulated motor turns for the day
  SCADA_SETTING_T scada_setting;     //SCADA setting
  COM_T comport;             //COM 
  UINT16_T crc;              //CRC
  HISTORY_T history[5];      //Fault history
  u8 history_index = 0;      //Fault history memory index
} EEPROM_T;

#endif

