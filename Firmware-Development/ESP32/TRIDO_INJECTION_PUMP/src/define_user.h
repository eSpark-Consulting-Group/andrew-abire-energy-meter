#ifndef DEFINE_USER_H
#define DEFINE_USER_H

// NO. OF PUMPS
#define NO_OF_PUMPS           4    // 4 units

// SETTING LIMIT
#define MAXIMUM_RPM           400  // 400RPM, max. stable rpm at 24V is 350
#define MINIMUM_RPM           20   // 20RPM, sensorless field oriented control limit
//#define MINIMUM_OCP_DCAMP     25   // 2.5A
//#define MAXIMUM_OCP_DCAMP     250  // 25.0A

#define MINIMUM_DCV_INPUT     210  // 21.0V
#define MAXIMUM_DCV_INPUT     360  // 36.0V
#define SLOW_DOWN_DCV         230  // 23.0V

// DISPLAY LIMIT
#define MAXIMUM_RPM_DISP      999  // 999RPM
#define MAXIMUM_DCV_DISP      999  // 99.9V
#define MAXIMUM_DCA_DISP      999  // 99.9A
#define MAXIMUM_MOTORA_DISP   999  // 99.9A

// SCREEN SAVER
#define SCREEN_SAVER_TIMEOUT  300  // 1=1s

// HISTORY FAULT RECORD
#define MAX_HISTORY_FAULT_RECORD               5
#define TOTAL_BYTES_IN_A_HISTORY_FAULT_RECORD  10    

// VFD  
#define VFD_COMMUNICATION_TIMEOUT              30    //1=100ms

#define SERIAL0_BAUD          115200L
#define SERIAL1_BAUD          115200L
#define SERIAL2_BAUD          115200L

// EEPROM (EPS32 flash simulated)
#define EEPROM_SIZE           1024  // This value must fit with no. of bytes to be saved + 2 CRC bytes
                                    // ESP32 supports up to 1024 bytes 
									// (4 bytes for each value saved x 1024 = 4096 = 1 sector)

#define PI_VALUE              (float)3.1415926535897932384626433832795

//<!Pin Definition------------------------------------------------------------
// KEYPAD PIN DEFINITION
//<!IMPORTANT NOTE: IO34 and IO35 should not be used as output
//The keypad library uses column pins as outputs and row pins as inputs
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
#define KEY_ROWS              3
#define KEY_COLS              3

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
// MENU
//

#define MENU_STRING_BUFSIZE    42  // each character 6 dots
//PAGE 1
#define MENU_PAGE1             10
#define PAGE1_FUNC1            11
//PAGE 2:
#define MENU_PAGE2             20
#define PAGE2_FUNC1            21
//PAGE 3:
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

// OTHERS
#define MAX_MENU_PAGE          12
#define MENU_TITLE_YPOS        10
#define MENU_LINE1_YPOS        23
#define MENU_LINE2_YPOS        36
#define MENU_LINE3_YPOS        49
#define MENU_LINE4_YPOS        62

// LCD
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
  ON                           = 0x01,
  OFF                          = 0x00
};

#define findmin(a,b) (((a) < (b)) ? (a) : (b))
#define findmax(a,b) (((a) > (b)) ? (a) : (b))

#endif
