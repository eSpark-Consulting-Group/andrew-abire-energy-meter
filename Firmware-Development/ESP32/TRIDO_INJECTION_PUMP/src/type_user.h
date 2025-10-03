#ifndef TYPE_USER_H
#define TYPE_USER_H

#include <Arduino.h>

#define u8  uint8_t
#define u16 uint16_t
#define u32 uint32_t
#define u64 uint64_t

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

// Menu Type
typedef enum{
  MENU_TYPE_NULL = 0,
  MENU_TYPE_FUNC,
  MENU_TYPE_LIST,
  MENU_TYPE_TEXT
} MENU_TYPE_e;

// Menu Layer
typedef enum{
  MENU_LAYER_0 = 0,
  MENU_LAYER_1,
  MENU_LAYER_2,
  MENU_LAYER_3,
  MENU_LAYER_MAX
} MENU_LAYER_e;

// Menu Item
typedef struct
{
  MENU_LAYER_e layer;             // Layer
  MENU_TYPE_e type;               // Type
  char *ptext;                    // Display Text
  void (*pfunc)(void);            // Callback Function
  u32 id;                         // Menu ID
  bool state;                     // Menu Active or not
} MENU_T;

// Menu History
typedef struct
{
  MENU_T *history[MENU_LAYER_MAX * 2];
  u32 menu_offset[MENU_LAYER_MAX];
} MENU_HISTORY_T;

// Menu Control
typedef struct
{
  MENU_T *parent;                 // Parent Menu
  MENU_T *display;                // Current Display Menu
  MENU_T *selected;               // Selected Menu
  u16 lcd_height;                 // LCD height
  u16 lcd_width;                  // LCD width
  u16 font_height;                // Font Height
  u16 font_width;                 // Font Width
  u32 menu_offset;                // Menu Offset = selected - display
  u16 parent_text_ypos;           // Title Text Y Position
  u16 text_xpos;                  // Item Text X Position
  u16 text_ypos;                  // Item Text Y Position
  u32 item_offset;                // Item <-> Item offset
  bool initialized = false;       // Menu Initializated or not
  bool enable = false;            // Menu enable or not
  bool update = false;            // Menu update or not
  bool runfunc = false;           // Menu Function running or not
  u8 max_items[MENU_LAYER_MAX];   // Maximum items in each layer
  u8 lcd_max_items;               // No. of printable items on LCD
} MENU_CTRL_T;





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

// Date (U16)
typedef union
{
  u16 all;
  u8 byte[2]; // [0] = year, [1] = month
} DATETIME_U16_T;

// Control Parameters Send To VFD
typedef struct
{
  UINT16_T rpm;              //Set rpm
  bool onoff = OFF;          //ON/OFF
  bool reset = false;        //Controller reset request
  bool reset_ok = false;     //Controller reset response
  u8 mode = LOCAL_MODE;      //DIO/LOCAL/REMOTE
  UINT16_T motor_model;      // byte[0] Motor model number 0 = 25, 1 = 35, 2 = 50, 3 = 70, 4 = GMI, 5 = 70LS
                             // byte[1] Board number 0 = A(0.002), 1 = D(0.005)
  bool communication_fail_action = 0;  //0 = Keep motor's last state, 1 = Stop motor
  bool startup_setting_completed = false;
  u16 count = 0;             //Heartbeat out increment at 1000ms interval
  u16 prev_count = 0;        //Last heartbeat count
  UINT16_T min_dcv_input;    //Minimum DC input voltage
  UINT16_T max_dcv_input;    //Maximum DC input voltage
  UINT16_T slow_down_dcv;    //Motor slow down belows/equals to this DC input voltage
  double min_dcv_input_f = (double)MINIMUM_DCV_INPUT / 10.0;
  double max_dcv_input_f = (double)MAXIMUM_DCV_INPUT / 10.0;
  double slow_down_dcv_f = (double)SLOW_DOWN_DCV / 10.0;
  UINT16_T ocp_dcamp;         //Over current ratio in DC amp of full load currrent
  double ocp_dcamp_f = 8.0;
} CONTROL_T;

// Calibration data for each pump
typedef struct
{
  UINT16_T correction_factor;
  double correction_factor_f = 1.0;
  DATETIME_T calibration_dnt;       // Calibration date and time
  UINT16_T stroke_length;           // 16 bit represents 0.0 - 1.0"
} PUMP_CALIBRATION_T;

// Record data for each pump
typedef struct
{
  DATETIME_U16_T record_date[24];      //Stroke count record date  
  u8 record_index = 0;                 //Holds the record number from 0-23
  UINT32_T month_stroke_count[24];     //Month stroke count
  UINT32_T accumulated_stroke_count;   //Accumulated stroke count
  UINT32_T day_stroke_count;           //Day stroke count
  UINT32_T hour_stroke_count;          //Hour stroke count
  double stroke_count_f = 0.0;          //For temp. calculation
} PUMP_RECORD_T;

// Injection Pump
typedef struct
{
  //u32 runtime_total = 0;            // Total run time in minutes
  u16 runtime_day = 0;                // Total run time in days
  u8 runtime_hour = 0;                // Total run time in hours
  u8 runtime_min = 0;                 // Total run time in mins
  u8 runtime_sec = 0;                 // Total run time in secs
  bool reset_runtime = false;         // 0 = no action, 1 = reset now
  bool dio_mode = true;               // 1 = disable
  UINT16_T stroke_length[NO_OF_PUMPS]; // typical 1/3",1/2",3/4",1",1.5",2"
  u8 piston_diameter[NO_OF_PUMPS] = {2,2,2,2};    // 0 = 3/16", 1 = 1/4", 2 = 3/8", 3 = 1/2", 4 = 3/4"
  UINT16_T pump_rate[NO_OF_PUMPS];    // Litres per day, for screen display 
  UINT16_T stroke_rate;               // Litres per day
  UINT16_T desired_rate[NO_OF_PUMPS]; // Litres per day, for calibration
  UINT16_T actual_rate[NO_OF_PUMPS];  // Litres per day
  UINT16_T gear_ratio;                // [n] : 1
  u16 min_pump_rate = 1;
  double min_pump_rate_f = 0.1;
  u16 max_pump_rate = 9999;
  double max_pump_rate_f = 999.9;
  double pump_rate_f[NO_OF_PUMPS] = {0.0,0.0,0.0,0.0};
  double desired_rate_f[NO_OF_PUMPS] = {0.0,0.0,0.0,0.0};
  double actual_rate_f[NO_OF_PUMPS] = {0.0,0.0,0.0,0.0};
  double cal_stroke_rate_f = 0.0;      // calculated stroke count per minute
  double gear_ratio_f = 10.0;
  double volume_per_stroke_f[NO_OF_PUMPS] = {0.0,0.0,0.0,0.0};
  double piston_diameter_m_f[NO_OF_PUMPS] = {0.0,0.0,0.0,0.0};
  double stroke_length_inch_f[NO_OF_PUMPS] = {1.0,1.0,1.0,1.0};
  PUMP_RECORD_T pump_record;               //Hold the record the pump
  PUMP_CALIBRATION_T pump[NO_OF_PUMPS];    //Hold the calibration for each pump
  u8 total_pump_head = 1;              //1 / 4             
  u8 pump_number = 0;                  //Pump Number 0, 1, 2, 3
  u8 volunit = 0;                      //0 = Litre, 1 = Gallon
  double conversion_factor_f = 1000.0;  //m3 to L/gal mul. factor for volume
  u8 tempunit = 0;                     //0 = Deg.C, 1 = F
  u8 temp_ctrl = 0;                    //Temperature control on/off
  INT16_T off_temp;                    //Temperature off threshold
  INT16_T on_temp;                     //Temperature on threshold
  INT16_T temp_now;                    //Current temperature
} INJPUMP_T;

// Status Read from VFD
typedef struct
{
  INT16_T rpm;                //Read rpm
  bool onoff = false;         //Read 1 = ON, 0 = OFF
  INT16_T dcv;                //Read DC Voltage
  INT16_T dca;                //Read DC Current
  INT16_T motora;             //Motor phase-phase current converted to rms
  double dcv_f = 0.0;          //
  double dca_f = 0.0;          //
  double motora_f = 0.0;       //
  u16 count = 0;              //Heartbeat in at 1000ms interval
  INJPUMP_T motor;
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
  u16 max_coil = 3;           //For calculation of maxmimum user defined address
  u16 max_discrete = 0;       //For calculation of maxmimum user defined address
  u16 max_inputreg = 0;       //For calculation of maxmimum user defined address
  u16 max_holdreg = 87;       //For calculation of maxmimum user defined address
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
  bool rsvd0:1;                  //4
  bool vfd_heartbeat_err:1;      //5
  bool vfd_est_err:1;            //6
  bool rpm_set_low_err:1;        //7
  bool rsvd1:1;                  //8
  bool rsvd2:1;                  //9
  bool rsvd3:1;                  //10
  bool rsvd4:1;                  //11
  bool rsvd5:1;                  //12
  bool rsvd6:1;                  //13
  bool rsvd7:1;                  //14
  bool rsvd8:1;                  //15
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
  bool reboot_pending:1;           //4
  bool rsvd1:1;                    //5
  bool rsvd2:1;                    //6
  bool rsvd3:1;                    //7
  bool rsvd4:1;                    //8
  bool rsvd5:1;                    //9
  bool rsvd6:1;                    //10
  bool rsvd7:1;                    //11
  bool rsvd8:1;                    //12
  bool rsvd9:1;                    //13
  bool rsvd10:1;                   //14
  bool rsvd11:1;                   //15
};

union SYSTEM_STATUS_T
{
  struct STATUS_BIT_T bit;
  u16 all;
  u8 byte[2];
};

struct IO_BIT_T
{
  bool DI1:1;
  bool DI2:1;
  bool DI3:1;
  bool DI4:1;
  bool DO1:1;
  bool DO2:1;
  bool DO3:1;
  bool DO4:1;
  bool rsvd:8;
};

union IO_T
{
  struct IO_BIT_T bit;
  uint16_t all;
};

typedef struct
{
  DATETIME_T dnt;            //Date & Time that the fault occured
  SYSTEM_ERR_T err;          //error bits
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
  INJPUMP_T motor;           //PUMP
  SCADA_SETTING_T scada_setting; //SCADA setting
  COM_T comport;             //COM
  UINT16_T crc;              //CRC
  HISTORY_T history[5];      //Fault history
  u8 history_index = 0;      //Fault history memory index
} EEPROM_T;
#endif
