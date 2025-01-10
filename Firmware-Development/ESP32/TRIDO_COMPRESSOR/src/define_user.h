#ifndef DEFINE_USER_H
#define DEFINE_USER_H

// SETTING LIMIT
#define MAXIMUM_RPM                     400  //400 RPM
//#define MINIMUM_OCP_DCAMP               25   //2.5A
//#define MAXIMUM_OCP_DCAMP               250  //25.0A

#define MINIMUM_DCV_INPUT               210  //21.0V
#define MAXIMUM_DCV_INPUT               360  //36.0V
#define SLOW_DOWN_DCV                   230  //23.0V

#define MAXIMUM_TANK_PRESSURE           999  //999 PSI

// DISPLAY LIMIT
#define MAXIMUM_RPM_DISP                999  //999 RPM
#define MAXIMUM_DCV_DISP                999  //99.9V
#define MAXIMUM_DCA_DISP                999  //99.9A
#define MAXIMUM_MOTORA_DISP             999  //99.9A
#define MAXIMUM_TANK_PRESSURE_DISP      999  //999 PSI

// DEFAULT VALUE
#define SENSOR_MAXIMUM_PSI              100  //100PSI
#define SENSOR_ZERO_PSI_MV              1000 //1V
#define SENSOR_MAXIMUM_PSI_MV           5000 //5V
#define SINGLE_DUTY_ALARM_PERCENT       100  //100%
#define AVERAGE_DUTY_ALARM_PERCENT      100  //100%

// SCREEN SAVER
#define SCREEN_SAVER_TIMEOUT            300  //5min

// HISTORY FAULT RECORD
#define MAX_HISTORY_FAULT_RECORD               5 
#define TOTAL_BYTES_IN_A_HISTORY_FAULT_RECORD  10

// COMPRESSOR
#define NO_OF_RAM_RECORD      24
#define NO_OF_SAVED_RECORD    24

// VFD  
#define VFD_COMMUNICATION_TIMEOUT              30    //1=100ms

#define SERIAL0_BAUD          115200L
#define SERIAL1_BAUD          115200L
#define SERIAL2_BAUD          115200L

// EEPROM (EPS32 flash simulated)
#define EEPROM_SIZE           512  // This value must fit with no. of bytes to be saved + 2 CRC bytes, ESP32 supports up to 512 bytes

//<!Pin Definition------------------------------------------------------------
// KEYPAD PIN DEFINITION
#define KEYPAD_R1_PIN         27
#define KEYPAD_R2_PIN         34
#define KEYPAD_R3_PIN         35
#define KEYPAD_C1_PIN         4
#define KEYPAD_C2_PIN         14
#define KEYPAD_C3_PIN         25

// MODBUS CHANNEL 1(Serial2) PIN DEFINITION
#define MODBUS_CH1_EN_PIN     32 //EN
#define MODBUS_CH1_TX_PIN     16 //RX2
#define MODBUS_CH1_RX_PIN     17 //TX2 

// MODBUS CHANNEL 2(Serial1) PIN DEFINITION
#define MODBUS_CH2_EN_PIN     33 //EN
#define MODBUS_CH2_TX_PIN     13 //U1TXD mirrored to this pin
#define MODBUS_CH2_RX_PIN     12 //U1RXD mirrored to this pin

// LCD PIN DEFINITION
#define LCD_CLK_PIN           18 //Shared with SD  
#define LCD_DIN_PIN           23 //Shared with SD  
#define LCD_RST_PIN           2
#define LCD_DC_PIN            26
#define LCD_CS_PIN            15

// RTC PIN DEFINITION
#define RTC_SDA_PIN           21
#define RTC_SCL_PIN           22

// SDCARD PIN DEFINITION
#define SD_CLK_PIN            18 //Shared with LCD  
#define SD_DIN_PIN            23 //Shared with LCD  
#define SD_DOUT_PIN           19
#define SD_CS_PIN             5

//<! Global Define-----------------------------------------------------------
// Keypad
#define KEY_ROWS               3
#define KEY_COLS               3

#define KEY_ESC               '1'
#define KEY_UP                '4'
#define KEY_MODE              '7'
#define KEY_LEFT              '2'
#define KEY_DOWN              '5'
#define KEY_RIGHT             '8'
#define KEY_ON                '3'
#define KEY_ENTER             '6'
#define KEY_OFF               '9'

//
// MENU unique identifier
//
//PAGE 1
#define MENU_PAGE1             10
#define PAGE1_FUNC1            11
//PAGE 2
#define MENU_PAGE2             20
#define PAGE2_FUNC1            21
//PAGE 3
#define MENU_PAGE3             30
#define PAGE3_FUNC1            31
//PAGE 4
#define MENU_PAGE4             40
#define PAGE4_FUNC1            41
//PAGE 5
#define MENU_PAGE5             50
#define PAGE5_FUNC1            51
//PAGE 6
#define MENU_PAGE6             60
#define PAGE6_FUNC1            61
//PAGE 7
#define MENU_PAGE7             70
#define PAGE7_FUNC1            71
//PAGE 8
#define MENU_PAGE8             80
#define PAGE8_FUNC1            81
//PAGE 9
#define MENU_PAGE9             90
#define PAGE9_FUNC1            91
//PAGE 10
#define MENU_PAGE10            100
#define PAGE10_FUNC1           101
//PAGE 11
#define MENU_PAGE11            110
#define PAGE11_FUNC1           111
//PAGE 12
#define MENU_PAGE12            120
#define PAGE12_FUNC1           121
//PAGE 13
#define MENU_PAGE13            130
#define PAGE13_FUNC1           131
//PAGE 14
#define MENU_PAGE14            140
#define PAGE14_FUNC1           141

// OTHERS
#define MAX_MENU_PAGE          14

//
// Screen pixel locations
//
#define MENU_TITLE_YPOS        10
#define MENU_LINE1_YPOS        23
#define MENU_LINE2_YPOS        36
#define MENU_LINE3_YPOS        49
#define MENU_LINE4_YPOS        62

//
// Indent from LHS of screen for presentation
//
#define MENU_COL1_XPOS         0
#define MENU_COL2_XPOS         136


// 

//
// Display parameters
//
#define MAX_LCD_WIDTH          256
#define MAX_LCD_HEIGHT         64
#define TEXT_COLOR_INVERTED    0
#define TEXT_COLOR_NORMAL      1

typedef enum
{
  MENU_FUNC_IDLE               = 0x00,
  MENU_FUNC_EXIT_SAVE          = 0x01,
  MENU_FUNC_EXIT_NO_SAVE       = 0x02,
  MENU_FUNC_CONFIRM            = 0x03,
  MENU_FUNC_WAIT_CONTROLLER    = 0x04
} FUNCSTATE_E;

enum
{
  LOCAL_MODE                   = 0x00,
  REMOTE_MODE                  = 0x01,
  DIO_MODE                     = 0x02
};

enum
{
  sysHOLD                      = 0x02,
  ON                           = 0x01,
  OFF                          = 0x00
};

enum
{
  MOTOR_STATE_UNDEFINED        = 0x00,
  MOTOR_STATE_OFF,
  MOTOR_STATE_RUN,
  MOTOR_STATE_HOLD,
};

#endif
