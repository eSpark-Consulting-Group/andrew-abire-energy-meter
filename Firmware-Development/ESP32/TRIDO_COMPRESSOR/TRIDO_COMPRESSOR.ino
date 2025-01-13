/******************************************************************************************************
     Project: Compressor Application
     Software Version: see SOFTWARE_VERSION_STRING[] below
     Hardware Version: see HARDWARE_VERSION_STRING[] below
     Modified: 22/05/2023

     Release: 2024.06.03 - jkrzywicki
     - removed "lock factory mode" lock-out which could cause the feature to be locked-out forever
     - added low-voltage restart
     - subtle changes to support v5 of the ESP32 API
     - updated eModbus to release 1.7.2
     - updated VERSION_STRING macros for less ambiguity, the POWERCTL will be read from the
    actual board in a future release

    Release 2024.06.11 - jkrzywicki
    - rolled back ESP32 API to v4 
  
j:\Projects\Trido\Townes Controller\Software\2024.06.03\ESP32\TRIDO_COMPRESSOR\src\modbus_user.h
 *******************************************************************************************************/
#include <EEPROM.h>
#include <esp_task_wdt.h>

//<!User Include: inside the src folder
#include "src\RTC\RTClib.h"
#include "src\U8g2\src\U8g2lib.h"
#include "src\Keypad\keypad.h"
#include "src\CRC\crc.h"
#include "src\XConvert\xconvert.h"
#include "src\type_user.h"
#include "src\define_user.h"
#include "src\modbus_user.h"
#include "src\func_prototype_user.h"

//<!Modbus Include
#include "src\eModbus\src\ModbusClientRTU.h"
#include "src\eModbus\src\ModbusServerRTU.h"
#include "src\eModbus\src\CoilData.h"

const char POWERCTL_VERSION_STRING[] = "2025.01.10";
//const char HARDWARE_VERSION_STRING[] = "YH_PB01_V5.001";
const char SYSTEMCTL_VERSION_STRING[] = "2025.01.10";

//<! Set to 0 for manufacturing
#define DEBUG_MODBUS_ERROR 0
#define DEBUG_PRINT_RESPONSE 0
#define DEBUG_FUNCTION 0
#define DEBUG_WRITEFLASH_FUNCTION 0

// 5 seconds Watchdog Timeout
#define WDT_TIMEOUT 5000
#define BATTERY_STABLE_TIME 120000

//<! Global Variables / Objects--------------------------------------------------------/
// General
char gStr1[42] = "";
u32 ms100Counter = 0;
bool clockTicking = false;
bool sysStable = false;
long batStableTime = 0u;
// Watchdog
int wdtResetCount = millis();

SYSTEM_ERR_T syserr;      // System Error bits
SYSTEM_STATUS_T sysstat;  // System Status/Warning bits
bool faultRecord = false;
u32 vfdErrCounter = 0;
u32 vfdFaultRecordCounter = 0;
u32 vfdHeartbeatCounter = 0;
u32 espResetCounter = 0;

//DIO
IO_T io;
#define ACTIVE_IO_BITS 8 // Active bits we want to count

// EEPROM
EEPROM_T eep;
u32 eepCounter = 0;
bool updateEEPROM = false;
unsigned int rbuf[EEPROM_SIZE];
unsigned int wbuf[EEPROM_SIZE];

// Motor
CONTROL_T ctrl;  // Control Parameters Send To VFD
bool startUpReset = true;
u16 MINIMUM_OCP_DCAMP = 10;   // 1.0A
u16 MAXIMUM_OCP_DCAMP = 100;  // 10.0A
int currentIndexRam = 0;
int currentIndexSaved = 0;
int startIndexSaved = 0;

// Status
STATUS_T status;  // Status Read from VFD
double rpm_filt = 0.0;
double totalturns = 0.0;

// 
u32 nPreviousMotorState = MOTOR_STATE_UNDEFINED;

TEMP_T temp;  // Modbus Parameters Send/Receive

// Keypad
const char keys[KEY_ROWS][KEY_COLS] = {
  { '1', '2', '3' },  // ESC  | UP    | Local/Remote
  { '4', '5', '6' },  // LEFT | DOWN  | RIGHT
  { '7', '8', '9' }   // ON   | ENTER | OFF
};
byte pinRows[KEY_ROWS] = { KEYPAD_R1_PIN, KEYPAD_R2_PIN, KEYPAD_R3_PIN };
byte pinCols[KEY_COLS] = { KEYPAD_C1_PIN, KEYPAD_C2_PIN, KEYPAD_C3_PIN };
Keypad kp = Keypad(makeKeymap(keys), pinRows, pinCols, sizeof(pinRows), sizeof(pinCols));
static byte kpState;
bool keyUphold = false;
bool keyOnhold = false;
bool keyDownhold = false;
bool keyLefthold = false;
bool keyRighthold = false;
bool keyEnterhold = false;
bool keyEschold = false;
bool keyLRhold = false;

//Modbus RTU client instance on Serial2
ModbusClientRTU mbClient(Serial2, MODBUS_CH1_EN_PIN, 10 /*queue_limit*/);
//Modbus RTU server instance listening on Serial1 with 2000ms timeout
ModbusServerRTU mbServer(Serial1, 2000, MODBUS_CH2_EN_PIN);
uint32_t mbTokenCounter = 0;

//SCADA
SCADA_SETTING_T scada_setting;
COM_T comport;
CoilData mycoils(scada_setting.max_coil, false);

const char protocol_arr[3][16] = {
  "RTU  ",
  "ASCII",
  "TCP  "
};

const char parity_arr[3][16] = {
  "NONE",
  "ODD ",
  "EVEN"
};

const char onoff_arr[2][16] = {
  "SysOff",
  "SysOn "
};

const u32 baudRate[12] = {
  1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 256000, 512000, 921600
};

const char motorturnreset_arr[2][16] = {
  "MANUAL  ",
  "00:00:00"
};

const char disable_dio_mode_arr[2][16] = {
  "ENABLE ",
  "DISABLE"
};

const char comfailaction_arr[2][16] = {
  "NO ACTION ",
  "STOP MOTOR"
};

const char pressure_control_arr[3][16] = {
  "STOP MOTOR ",
  "DOUT3 OPEN ",
  "DOUT3 CLOSE"
};

const char yesno_arr[2][16] = {
  "NO ",
  "YES"
};

const char brdSeries_arr[2][16] = {
  "A",
  "D"
};

const char mtrModel_arr[5][16] = {
  "TRIDO_25MM",
  "TRIDO_35MM",
  "TRIDO_50MM",
  "TRIDO_75MM",
  "TRIDO_GMI "
};

const char mode_arr[3][16] = {
  "Local ",
  "Remote",
  "DIO   "
};

// RTC
RTC_DS3231 rtc;
DateTime now;
DATETIME_T dnt;   // for display use
DATETIME_T dnt1;  // for setting use
bool rtcFail = false;
u8 rtcRetry = 0;
u16 lastSec = 0;

// LCD
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R2, LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN);

// MENU
char Key;
u8 thisItem, nextItem;
char (*pFunc)(char);
bool copyValue = true;
bool funcUpdate = true;
bool protectScreen = false;
bool scanMenu = true;
bool toggleDutyRecord = false;
bool toggleClearDuty = false;
bool showFactoryMenu = false;
FUNCSTATE_E funcResponse = MENU_FUNC_IDLE;
u8 selectedItem = 0;
u8 menuPage = 1;
u8 menuTotalItem = 0;
u8 toggleHistory = 0;
u8 numHistoryRecords = 0;
char errorDisplayCount = 0;
char warnDisplayCount = 0;
u16 screenSaverCount = 0;
char userMsg[3][52] = {
  { '\0' },
  { '\0' },
  { '\0' },
};
char errorMsg[8] = { '\0' };
//const u8 menuBarWidth = (u8)round((double)MAX_LCD_HEIGHT / (double)MAX_MENU_PAGE);
const u8 menuBarWidth = (u8)(MAX_LCD_HEIGHT / MAX_MENU_PAGE);

const u8 menuItemPos1[10][2] = {
  { 75 - 6, MENU_TITLE_YPOS },
  { 75 - 6, MENU_LINE4_YPOS },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 }
};

const u8 menuItemPos2[10][2] = {
  { 58 - 6, MENU_TITLE_YPOS },
  { 58 - 6, MENU_LINE1_YPOS },
  { 210 - 6, MENU_LINE2_YPOS },
  { 210 - 6, MENU_LINE3_YPOS },
  { 58 - 6, MENU_LINE4_YPOS },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 },
  { 0, 0 }
};

MENU_T next_menu[] = {
  // MENU      KEY INPUT   NEXT MENU
  /******************************************************************************************************/
  { MENU_PAGE1, KEY_ENTER, PAGE1_FUNC1 },
  { MENU_PAGE1, KEY_UP, MENU_PAGE14 },
  { MENU_PAGE1, KEY_DOWN, MENU_PAGE2 },
  /******************************************************************************************************/
  { MENU_PAGE2, KEY_ESC, MENU_PAGE1 },
  { MENU_PAGE2, KEY_ENTER, PAGE2_FUNC1 },
  { MENU_PAGE2, KEY_UP, MENU_PAGE1 },
  { MENU_PAGE2, KEY_DOWN, MENU_PAGE3 },
  /******************************************************************************************************/
  { MENU_PAGE3, KEY_ESC, MENU_PAGE1 },
  //{ MENU_PAGE3, KEY_ENTER, PAGE3_FUNC1 },
  { MENU_PAGE3, KEY_UP, MENU_PAGE2 },
  { MENU_PAGE3, KEY_DOWN, MENU_PAGE4 },
  /******************************************************************************************************/
  { MENU_PAGE4, KEY_ESC, MENU_PAGE1 },
  //{MENU_PAGE4,KEY_ENTER,PAGE4_FUNC1},
  { MENU_PAGE4, KEY_UP, MENU_PAGE3 },
  { MENU_PAGE4, KEY_DOWN, MENU_PAGE5 },
  /******************************************************************************************************/
  { MENU_PAGE5, KEY_ESC, MENU_PAGE1 },
  { MENU_PAGE5, KEY_ENTER, PAGE5_FUNC1 },
  { MENU_PAGE5, KEY_UP, MENU_PAGE4 },
  { MENU_PAGE5, KEY_DOWN, MENU_PAGE6 },
  /******************************************************************************************************/
  { MENU_PAGE6, KEY_ESC, MENU_PAGE1 },
  { MENU_PAGE6, KEY_ENTER, PAGE6_FUNC1 },
  { MENU_PAGE6, KEY_UP, MENU_PAGE5 },
  { MENU_PAGE6, KEY_DOWN, MENU_PAGE7 },
  /******************************************************************************************************/
  { MENU_PAGE7, KEY_ESC, MENU_PAGE1 },
  { MENU_PAGE7, KEY_ENTER, PAGE7_FUNC1 },
  { MENU_PAGE7, KEY_UP, MENU_PAGE6 },
  { MENU_PAGE7, KEY_DOWN, MENU_PAGE8 },
  /******************************************************************************************************/
  { MENU_PAGE8, KEY_ESC, MENU_PAGE1 },
  { MENU_PAGE8, KEY_ENTER, PAGE8_FUNC1 },
  { MENU_PAGE8, KEY_UP, MENU_PAGE7 },
  { MENU_PAGE8, KEY_DOWN, MENU_PAGE9 },
  /******************************************************************************************************/
  { MENU_PAGE9, KEY_ESC, MENU_PAGE1 },
  { MENU_PAGE9, KEY_ENTER, PAGE9_FUNC1 },
  { MENU_PAGE9, KEY_UP, MENU_PAGE8 },
  { MENU_PAGE9, KEY_DOWN, MENU_PAGE10 },
  /******************************************************************************************************/
  { MENU_PAGE10, KEY_ESC, MENU_PAGE1 },
  { MENU_PAGE10, KEY_ENTER, PAGE10_FUNC1 },
  { MENU_PAGE10, KEY_UP, MENU_PAGE9 },
  { MENU_PAGE10, KEY_DOWN, MENU_PAGE11 },
  /******************************************************************************************************/
  { MENU_PAGE11, KEY_ESC, MENU_PAGE1 },
  { MENU_PAGE11, KEY_ENTER, PAGE11_FUNC1 },
  { MENU_PAGE11, KEY_UP, MENU_PAGE10 },
  { MENU_PAGE11, KEY_DOWN, MENU_PAGE12 },
  /******************************************************************************************************/
  { MENU_PAGE12, KEY_ESC, MENU_PAGE1 },
  { MENU_PAGE12, KEY_ENTER, PAGE12_FUNC1 },
  { MENU_PAGE12, KEY_UP, MENU_PAGE11 },
  { MENU_PAGE12, KEY_DOWN, MENU_PAGE13 },
  /******************************************************************************************************/
  { MENU_PAGE13, KEY_ESC, MENU_PAGE1 },
  //{ MENU_PAGE13, KEY_ENTER, PAGE13_FUNC1 },
  { MENU_PAGE13, KEY_UP, MENU_PAGE12 },
  { MENU_PAGE13, KEY_DOWN, MENU_PAGE14 },
  /******************************************************************************************************/
  { MENU_PAGE14, KEY_ESC, MENU_PAGE1 },
  { MENU_PAGE14, KEY_ENTER, PAGE14_FUNC1 },
  { MENU_PAGE14, KEY_UP, MENU_PAGE13 },
  { MENU_PAGE14, KEY_DOWN, MENU_PAGE1 },
  /******************************************************************************************************/
  { 0, 0, 0 }
};

MENU_FUNC_T menu_func[] = {
  // MENU STATE  MENU FUNC CALL
  /******************************************************************************************************/
  { MENU_PAGE1, 0 },
  { PAGE1_FUNC1, page1Func1 },
  /******************************************************************************************************/
  { MENU_PAGE2, 0 },
  { PAGE2_FUNC1, page2Func1 },
  /******************************************************************************************************/
  { MENU_PAGE3, 0 },
  //{ PAGE3_FUNC1, page3Func1 },
  /******************************************************************************************************/
  { MENU_PAGE4, 0 },
  //{PAGE4_FUNC1, page4Func1},
  /******************************************************************************************************/
  { MENU_PAGE5, 0 },
  { PAGE5_FUNC1, page5Func1 },
  /******************************************************************************************************/
  { MENU_PAGE6, 0 },
  { PAGE6_FUNC1, page6Func1 },
  /******************************************************************************************************/
  { MENU_PAGE7, 0 },
  { PAGE7_FUNC1, page7Func1 },
  /******************************************************************************************************/
  { MENU_PAGE8, 0 },
  { PAGE8_FUNC1, page8Func1 },
  /******************************************************************************************************/
  { MENU_PAGE9, 0 },
  { PAGE9_FUNC1, page9Func1 },
  /******************************************************************************************************/
  { MENU_PAGE10, 0 },
  { PAGE10_FUNC1, page10Func1 },
  /******************************************************************************************************/
  { MENU_PAGE11, 0 },
  { PAGE11_FUNC1, page11Func1 },
  /******************************************************************************************************/
  { MENU_PAGE12, 0 },
  { PAGE12_FUNC1, page12Func1},
  /******************************************************************************************************/
  { MENU_PAGE13, 0 },
  //{ PAGE13_FUNC1, page13Func1 },
  /******************************************************************************************************/
  { MENU_PAGE14, 0 },
  { PAGE14_FUNC1, page14Func1 },
  /******************************************************************************************************/
  { 0, 0 }
};

/*
*********************************************************************************************************
	Function: getNextItem
	Description: Get next menu item
	Parameter: thisItem - previous item
             key_input - key
	Return: nextItem
*********************************************************************************************************
*/
u8 getNextItem(char thisItem, u8 key_input) {
  u8 nextItem = thisItem;  //stay in the same item

  if (key_input == NO_KEY) return nextItem;

  //scan the next_menu[]
  for (u8 i = 0; next_menu[i].this_item != 0; i++) {
    if (next_menu[i].this_item == thisItem && next_menu[i].input == key_input) {
      nextItem = next_menu[i].next_item;
      break;
    }
  }

  return nextItem;
}

/*
*********************************************************************************************************
	Function: u8g2_draw_arrow
	Description: Draw an arrow ">" next to a selected item
	Parameter: x - LCD x position
             y - LCD y position
             fontheight - Font Height
             maxrow - Maximum number of rows in the page
	Return: None
*********************************************************************************************************
*/
void u8g2_draw_arrow(u16 x, u8 y, u8 fontheight, u8 maxrow) {
  for (u8 i = 0; i < maxrow; i++) {
    if (i == selectedItem) u8g2.drawStr(x, y + i * fontheight, ">");
    else u8g2.drawStr(x, y + i * fontheight, " ");
  }
}

//---------------------------------------------------------
// u8g2_draw_arrow_at_array
//---------------------------------------------------------
void u8g2_draw_arrow_at_array(const u8 array[10][2], u8 maxitem) {
  for (u8 i = 0; i < maxitem; i++) {
    if (i == selectedItem) u8g2.drawStr(array[i][0], array[i][1], ">");
    else u8g2.drawStr(array[i][0], array[i][1], " ");
  }
}

//---------------------------------------------------------
// u8g2_draw_scrollbar
//---------------------------------------------------------
void u8g2_draw_scrollbar() {
  u8 remain_pixels = MAX_LCD_HEIGHT % menuPage;
  u8g2.drawFrame(253, 0, 3, 64);
  if (MAX_MENU_PAGE == menuPage)
    u8g2.drawBox(254, (menuPage - 1) * menuBarWidth, 1, menuBarWidth + remain_pixels);
  else
    u8g2.drawBox(254, (menuPage - 1) * menuBarWidth, 1, menuBarWidth);
}

//------------------------------------------
// u8g2_move_item_index
// function: set item no pointed by item arrow
//------------------------------------------
void u8g2_move_item_index(u8 *p, u8 maxitem) {
  if (pFunc != NULL) return;  // if we are inside a function, do nothing

  if (Key == KEY_LEFT) {
    if (*p > 0) *p -= 1;
  } else if (Key == KEY_RIGHT) {
    if (*p < maxitem - 1) *p += 1;
  } else if (Key == KEY_UP || Key == KEY_DOWN)
    *p = 0;
}

//-------------------------
// twos2Int
//-------------------------
int twos2Int(u8 msb, u8 lsb) {
  long int val = ((long int)msb << 8) | lsb;
  if ((val & 0x8000)) val -= 65536;
  return (int)val;
}

//-----------------------
// powerU16
//-----------------------
u16 powerU16(u16 x, u8 y) {
  u16 res = x;
  u8 j = y;

  if (y == 0) return 1;
  while (--j) res = res * 10;
  return res;
}

//-----------------------
// powerInt
//-----------------------
int powerInt(int x, int y) {
  int res = x;
  int j = y;

  if (y == 0) return 1;
  while (--j) res = res * 10;
  return res;
}

//------------------------------------------------------------------------
// SetParaFloat (floating point)
//------------------------------------------------------------------------
void SetParaFloat(u16 x, u16 y, u16 *val, u8 maxvlen, u16 minval, u16 maxval) {
  static u16 gVal1 = 0;
  static u8 cusorPos = 0;
  u8 maxindex = maxvlen - 1;
  u16 mul;

  if (copyValue) {
    cusorPos = maxindex;
    gVal1 = *val;
    copyValue = false;
  }

#if DEBUG_FUNCTION
  if (cusorPos > maxindex)
    Serial.println("SetParaFloat: cusorPos > maxindex");
  Serial.print("key:");
  Serial.println(Key);
#endif

  mul = powerU16(10, maxindex - cusorPos);

  if (Key == KEY_UP || keyUphold) {
    if (gVal1 <= (maxval - mul)) gVal1 += mul;
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (gVal1 >= (minval + mul)) gVal1 -= mul;
    funcUpdate = true;
  } else if (Key == KEY_LEFT) {
    if (cusorPos > 0) cusorPos--;
  } else if (Key == KEY_RIGHT) {
    if (cusorPos < maxindex) cusorPos++;
  } else if (Key == KEY_ENTER) {
    *val = gVal1;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    char tmp[10];
    xconvert(tmp, "%%0%dd", maxindex);
    xconvert(gStr1, tmp, (int)((double)gVal1 / 10.0));
    u8g2.drawStr(x, y, gStr1);
    xconvert(gStr1, ".%d", gVal1 % 10);
    u8g2.drawStr(x + maxindex * 6, y, gStr1);
    if (cusorPos == maxindex)
      u8g2.drawHLine(x + maxvlen * 6, y + 1, 6);
    else
      u8g2.drawHLine(x + cusorPos * 6, y + 1, 6);
    u8g2.sendBuffer();  // send to LCD
    funcUpdate = false;
  }
}

//------------------------------------------------------------------------
// SetParaBoolstr (bool->gStr1)
//------------------------------------------------------------------------
void SetParaBoolstr(u16 x, u16 y, bool *val, const char p[2][16]) {
  static bool gVal0 = false;

  if (copyValue) {
    gVal0 = *val;
    copyValue = false;
  }

  if (Key == KEY_UP || Key == KEY_DOWN) {
    gVal0 = !gVal0;
    funcUpdate = true;
  } else if (Key == KEY_ENTER) {
    *val = gVal0;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawStr(x, y, p[gVal0]);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    u8g2.sendBuffer();  // send to LCD
    funcUpdate = false;
  }
}

//--------------------------------------------------------------------------------
// SetParaU8str (u8->gStr1)
//--------------------------------------------------------------------------------
void SetParaU8str(u16 x, u16 y, u8 *val, u8 minval, u8 maxval, const char p[3][16]) {
  static u8 gVal4 = 0;

  if (copyValue) {
    gVal4 = *val;
    copyValue = false;
  }

  if (Key == KEY_UP || keyUphold) {
    if (gVal4 < maxval) gVal4++;
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (gVal4 > minval) gVal4--;
    funcUpdate = true;
  } else if (Key == KEY_ENTER) {
    *val = gVal4;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawStr(x, y, p[gVal4]);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    u8g2.sendBuffer();  // send to LCD
    funcUpdate = false;
  }
}

//------------------------------------------------------------------------
// SetParaU8 (u8)
//------------------------------------------------------------------------
void SetParaU8(u16 x, u16 y, u8 *val, u8 maxvlen, u8 minval, u8 maxval) {
  static u8 gVal4 = 0;
  static u8 cusorPos = 0;
  u8 maxindex = maxvlen - 1;
  u8 mul;

  if (copyValue) {
    cusorPos = maxindex;
    gVal4 = *val;
    copyValue = false;
  }

#if DEBUG_FUNCTION
  if (cusorPos > maxindex)
    Serial.println("SetParaU8: cusorPos > maxindex");
  Serial.print("key:");
  Serial.println(Key);
#endif

  mul = powerU16(10, maxindex - cusorPos);

  if (Key == KEY_UP || keyUphold) {
    if (gVal4 <= (maxval - mul)) gVal4 += mul;
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (gVal4 >= (minval + mul)) gVal4 -= mul;
    funcUpdate = true;
  } else if (Key == KEY_LEFT) {
    if (cusorPos > 0) cusorPos--;
  } else if (Key == KEY_RIGHT) {
    if (cusorPos < maxindex) cusorPos++;
  } else if (Key == KEY_ENTER) {
    *val = gVal4;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    char tmp[10];
    xconvert(tmp, "%%0%dd", maxvlen);
    xconvert(gStr1, tmp, gVal4);
    u8g2.drawStr(x, y, gStr1);
    u8g2.drawHLine(x + cusorPos * 6, y + 1, 6);
    u8g2.sendBuffer();  // send to LCD
    funcUpdate = false;
  }
}

//------------------------------------------------------------------------
// SetParaUint (u16)
//------------------------------------------------------------------------
void SetParaUint(u16 x, u16 y, u16 *val, u8 maxvlen, u16 minval, u16 maxval) {
  static u16 gVal1 = 0;
  static u8 cusorPos = 0;
  u8 maxindex = maxvlen - 1;
  u16 mul;

  if (copyValue) {
    cusorPos = maxindex;
    gVal1 = *val;
    copyValue = false;
  }

#if DEBUG_FUNCTION
  if (cusorPos > maxindex)
    Serial.println("SetParaUint: cusorPos > maxindex");
  Serial.print("key:");
  Serial.println(Key);
#endif

  mul = powerU16(10, maxindex - cusorPos);

  if (Key == KEY_UP || keyUphold) {
    if (gVal1 <= (maxval - mul)) gVal1 += mul;
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (gVal1 >= (minval + mul)) gVal1 -= mul;
    funcUpdate = true;
  } else if (Key == KEY_LEFT) {
    if (cusorPos > 0) cusorPos--;
  } else if (Key == KEY_RIGHT) {
    if (cusorPos < maxindex) cusorPos++;
  } else if (Key == KEY_ENTER) {
    *val = gVal1;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    char tmp[10];
    xconvert(tmp, "%%0%dd", maxvlen);
    xconvert(gStr1, tmp, gVal1);
    u8g2.drawStr(x, y, gStr1);
    u8g2.drawHLine(x + cusorPos * 6, y + 1, 6);
    u8g2.sendBuffer();  // send to LCD
    funcUpdate = false;
  }
}

//------------------------------------------------------------------------
// SetParaInt (int16)
//------------------------------------------------------------------------
void SetParaInt(u16 x, u16 y, int *val, int maxvlen, int minval, int maxval) {
  static int gVal2 = 0;
  static u8 cusorPos = 0;
  static int prevNum = 0;
  u8 maxindex = maxvlen - 1;
  int mul;
  INT16_T tmpval;

  if (copyValue) {
    gVal2 = *val;
    tmpval.all = gVal2;
    gVal2 = twos2Int(tmpval.byte[1], tmpval.byte[0]);
    if (gVal2 >= 0) cusorPos = maxindex;
    else cusorPos = maxvlen;
    copyValue = false;
  }

  tmpval.all = gVal2;
  gVal2 = twos2Int(tmpval.byte[1], tmpval.byte[0]);

#if DEBUG_FUNCTION
  if (cusorPos > maxindex)
    Serial.println("SetParaFloat: cusorPos > maxindex");
  Serial.print("key:");
  Serial.println(Key);
#endif

  if (gVal2 >= 0) mul = powerInt(10, maxindex - cusorPos);
  else mul = powerInt(10, maxvlen - cusorPos);

  if (Key == KEY_UP || keyUphold) {
    prevNum = gVal2;
    if (gVal2 <= (maxval - mul)) gVal2 += mul;
    if ((prevNum < 0) && (gVal2 >= 0)) cusorPos--;
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    prevNum = gVal2;
    if (gVal2 >= (minval + mul)) gVal2 -= mul;
    if ((prevNum >= 0) && (gVal2 < 0)) cusorPos++;
    funcUpdate = true;
  } else if (Key == KEY_LEFT) {
    if (gVal2 >= 0) {
      if (cusorPos > 0) cusorPos--;
    } else {
      if (cusorPos > 1) cusorPos--;
    }
  } else if (Key == KEY_RIGHT) {
    if (gVal2 >= 0) {
      if (cusorPos < maxindex) cusorPos++;
    } else {
      if (cusorPos < maxvlen) cusorPos++;
    }
  } else if (Key == KEY_ENTER) {
    *val = gVal2;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    char tmp[10];
    if (gVal2 >= 0)
      xconvert(tmp, "%%0%dd ", maxvlen);
    else
      xconvert(tmp, "%%0%dd", maxvlen);
    xconvert(gStr1, tmp, gVal2);
    u8g2.drawStr(x, y, gStr1);
    if (gVal2 < 0) {
      if (cusorPos == 0) cusorPos = 1;
    }
    u8g2.drawHLine(x + cusorPos * 6, y + 1, 6);
    u8g2.sendBuffer();  // send to LCD
    funcUpdate = false;
  }
}

//-----------------------------------------------------------------------------
// SelectParaFromList p (u32)
//-----------------------------------------------------------------------------
void SelectParaFromList(u16 x, u16 y, u32 *val, const u32 *p, u8 maxitem, u8 maxclen) {
  static u32 gVal3 = 0;
  static u8 listIndex = 0;
  u8 k = maxitem;
  u8 j = maxitem - 1;

  if (copyValue) {
    gVal3 = *val;
    while (k--) {
      if (listIndex > j) {
        listIndex = j;
        break;
      }
      if (gVal3 != p[listIndex]) {
        listIndex++;
      } else break;
    }
    copyValue = false;
  }

  if (Key == KEY_UP || keyUphold) {
    if (listIndex < j) listIndex++;
    gVal3 = p[listIndex];
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (listIndex > 0) listIndex--;
    gVal3 = p[listIndex];
    funcUpdate = true;
  } else if (Key == KEY_ENTER) {
    *val = gVal3;
    listIndex = 0;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    listIndex = 0;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    char tmp[10];
    xconvert(tmp, "%%0%dd", maxclen);
    xconvert(gStr1, tmp, gVal3);
    u8g2.drawStr(x, y, gStr1);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    u8g2.sendBuffer();  // send to LCD
    funcUpdate = false;
  }
}

//----------------------
// getMaxday
//----------------------
u16 getMaxday(u16 year, u16 month) {
  switch (month) {
    case 1:
    case 3:
    case 5:
    case 7:
    case 8:
    case 10:
    case 12:
      return 31;
    case 2:
      if (((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)) return 29;
      else return 28;
    case 4:
    case 6:
    case 9:
    case 11:
      return 30;
  }
  return 0;
}

//----------------------
// getTime
//----------------------
bool getTime(DateTime dt, DATETIME_T *dt1) {
  bool ret = false;
  if ((dt.year() <= 2150) && (dt.month() > 0) && (dt.month() <= 12)) {
    u16 maxday = getMaxday(dt.year(), dt.month());
    if ((dt.day() > 0) && (dt.day() <= maxday)) {
      if ((dt.hour() < 24) && (dt.minute() < 60) && (dt.second() < 60)) {
        dt1->year = dt.year();
        dt1->month = dt.month();
        dt1->day = dt.day();
        dt1->hour = dt.hour();
        dt1->minute = dt.minute();
        dt1->second = dt.second();
        ret = true;  //RTC ok
      } else {
        //RTC error
#if DEBUG_FUNCTION        
        Serial.println("getTime(): check date and time error!");
#endif        
        //xconvert(gStr1, "%d/%02d/%02d %02d:%02d:%02d", dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
        //Serial.println(gStr1);
        *dt1 = { 2000, 1, 1, 0, 0, 0 };  //yyyymmdd,hhmmss,set to known value when error
      }
    }
  }

  return ret;
}

//------------------------------------------
// Update SCADA COM port Config
//------------------------------------------
void COMport1Config() {
  //<!Calculate the COM port config value
  //Set Data bits
  comport.config.all = 0x00;
  switch (comport.databits) {
    case 7:
      comport.config.bit.databits = 2;
      break;
    case 8:
      comport.config.bit.databits = 3;
      break;
  }
  //Set Stop bits
  switch (comport.stopbits) {
    case 1:
      comport.config.bit.stopbits = 1;
      break;
    case 2:
      comport.config.bit.stopbits = 3;
      break;
  }
  //Set Parity
  switch (comport.parity) {
    case 0:  //NONE
      comport.config.bit.parity = 0;
      break;
    case 1:  //ODD
      comport.config.bit.parity = 3;
      break;
    case 2:  //EVEN
      comport.config.bit.parity = 2;
      break;
  }
  //Configure COM port 1
  Serial1.begin(comport.baud.all, (u32)(0x8000000 + comport.config.all), MODBUS_CH2_RX_PIN /*12*/, MODBUS_CH2_TX_PIN /*13*/);
}

//--------------------------
// RPM Setting
//--------------------------
void setRPMfunc() {
  SetParaInt(75, 10, &ctrl.rpm.all, 3, -MAXIMUM_RPM, MAXIMUM_RPM);

  if (Key == KEY_ON || Key == KEY_OFF) {
    //show status
    if (ctrl.onoff == ON)
      u8g2.drawStr(75, MENU_LINE1_YPOS, "SysOn ");
    else
      u8g2.drawStr(75, MENU_LINE1_YPOS, "SysOff");
  }
}

//--------------------------
// User Setting 1 f5
//--------------------------
void setUserFunc1() {
  switch (selectedItem) {
    case 0:
      SetParaFloat(128, MENU_LINE1_YPOS, &ctrl.min_dcv_input.all, 3, 180, 319);
      ctrl.min_dcv_input_f = (double)ctrl.min_dcv_input.all / 10.0;
      break;
    case 1:
      SetParaFloat(128, MENU_LINE2_YPOS, &ctrl.slow_down_dcv.all, 3, 180, 319);
      ctrl.slow_down_dcv_f = (double)ctrl.slow_down_dcv.all / 10.0;
      break;
    case 2:
      SetParaFloat(128, MENU_LINE3_YPOS, &ctrl.ocp_dcamp.all, 3, MINIMUM_OCP_DCAMP, MAXIMUM_OCP_DCAMP);
      ctrl.ocp_dcamp_f = (double)ctrl.ocp_dcamp.all / 10.0;
      break;
      //case 3:
      //SetParaFloat(128, MENU_LINE4_YPOS, &ctrl.max_dcv_input.all, 3, 150, 430);
      //ctrl.max_dcv_input_f = (double)ctrl.max_dcv_input.all / 10.0;
      //break;
  }
}

//--------------------------
// User Setting 2
//--------------------------
void setUserFunc2() {
  switch (selectedItem) {
    case 0:
      SetParaU8(138 /*lcd xpos*/, MENU_LINE1_YPOS, &status.motor.sensor_psi_offset, 2 /*length*/, 0 /*min*/, 10 /*max*/);
      break;
    case 1:
      SetParaUint(138 /*lcd xpos*/, MENU_LINE2_YPOS, &status.motor.single_duty_alarm_percent.all, 3 /*length*/, 0 /*min*/, 100 /*max*/);
      break;
    case 2:
      SetParaUint(138 /*lcd xpos*/, MENU_LINE3_YPOS, &status.motor.average_duty_alarm_percent.all, 3 /*length*/, 0 /*min*/, 100 /*max*/);
      break;
  }
}

//--------------------------
// User Setting 3
//--------------------------
void setUserFunc3() {
  switch (selectedItem) {
    case 0:
      SetParaU8str(MENU_COL2_XPOS, MENU_LINE1_YPOS, &status.motor.pressure_control_method, 0, 2, pressure_control_arr);  //pressure control method [0=STOP MOTOR, 1=DOUT3 OPEN, 2=DOUT3 CLOSE]
      break;
    case 1:
      SetParaBoolstr(MENU_COL2_XPOS, MENU_LINE2_YPOS, &toggleClearDuty, yesno_arr);  //0=NO ACTION, 1=CLEAR
      if (toggleClearDuty) {
        //reset runtime
        status.motor.runtime_sec = 0;
        status.motor.runtime_min = 0;
        status.motor.runtime_hour = 0;
        status.motor.runtime_day.all = 0;
        status.motor.runtime_total = 0;
        //reset cycletime
        status.motor.cycletime_sec = 0;
        status.motor.cycletime_min = 0;
        status.motor.cycletime_hour = 0;
        status.motor.cycletime_day.all = 0;
        status.motor.cycletime_total = 0;
        //reset duty which is update on LCD every 1 min
        status.motor.duty = 0;
        // reset counters
        status.motor.ram_record_index = 0;
        currentIndexRam = 0;
        currentIndexSaved = 0;
        startIndexSaved = 0;
        //TEST CODE ONLY
        /*for (int k = 0; k < NO_OF_RAM_RECORD; k++) {
          status.motor.ram_record[k] = 99;
          }
          for (int k = 0; k < NO_OF_SAVED_RECORD; k++) {
          status.motor.saved_record[k] = 99;
          }*/
        strcpy(userMsg[1], "CLEAR DUTY!");
        toggleClearDuty = false;
      }
      break;
    case 2:
      SetParaBoolstr(MENU_COL2_XPOS, MENU_LINE3_YPOS, &status.motor.autoreset_motorturns, motorturnreset_arr);  // 1 = reset at 00:00:00, 0 = manual reset
      break;
    case 3:
      SetParaBoolstr(MENU_COL2_XPOS, MENU_LINE4_YPOS, &status.motor.disable_dio_mode, disable_dio_mode_arr);  // 1 = disable
      //switch to local mode if DIO mode was used
      if (status.motor.disable_dio_mode) {
        if (ctrl.mode == DIO_MODE) (ctrl.mode = LOCAL_MODE);
      }
      break;
  }
}

//--------------------------
// Date and Time Setting
//--------------------------
void setDatetimeFunc(u16 x, u16 y) {
  static u16 *pVal1 = NULL;
  static u8 cusorPos = 0;
  u16 mul = 0;
  u16 maxsetlen = 19;
  u16 maxvlen = 1;
  u8 maxindex = maxvlen - 1;
  u16 minval = 0;
  u16 maxval = 0;
  u16 maxday = 0;
  bool validpos = true;

  if (copyValue) {
    copyValue = false;
    //check date and time value
    if (!getTime(now, &dnt1)) {
      return;
    }
    cusorPos = 2;
  }

#if DEBUG_FUNCTION
  if (cusorPos > maxsetlen - 1)
    Serial.println("setDatetimeFunc: cusorPos > maxindex");
  Serial.print("key:");
  Serial.println(Key);
#endif

  switch (cusorPos) {
    case 0:
    case 1:
    case 2:
    case 3:
      maxvlen = 4;
      maxindex = maxvlen - 1;
      minval = 2000;
      maxval = 2150;
      pVal1 = &dnt1.year;
      mul = powerU16(10, maxindex - cusorPos);
      maxday = getMaxday(dnt1.year, dnt1.month);
      if (dnt1.day > maxday) {
        dnt1.day = maxday;
        funcUpdate = true;
      }
      break;
    case 5:
    case 6:
      maxvlen = 2;
      maxindex = maxvlen - 1;
      minval = 1;
      maxval = 12;
      pVal1 = &dnt1.month;
      mul = powerU16(10, maxindex - (cusorPos - 5));
      maxday = getMaxday(dnt1.year, dnt1.month);
      if (dnt1.day > maxday) {
        dnt1.day = maxday;
        funcUpdate = true;
      }
      break;
    case 8:
    case 9:
      maxvlen = 2;
      maxindex = maxvlen - 1;
      minval = 1;
      maxval = getMaxday(dnt1.year, dnt1.month);
      pVal1 = &dnt1.day;
      mul = powerU16(10, maxindex - (cusorPos - 8));
      break;
    case 11:
    case 12:
      maxvlen = 2;
      maxindex = maxvlen - 1;
      pVal1 = &dnt1.hour;
      minval = 0;
      maxval = 23;
      mul = powerU16(10, maxindex - (cusorPos - 11));
      break;
    case 14:
    case 15:
      maxvlen = 2;
      maxindex = maxvlen - 1;
      pVal1 = &dnt1.minute;
      minval = 0;
      maxval = 59;
      mul = powerU16(10, maxindex - (cusorPos - 14));
      break;
    case 17:
    case 18:
      maxvlen = 2;
      maxindex = maxvlen - 1;
      pVal1 = &dnt1.second;
      minval = 0;
      maxval = 59;
      mul = powerU16(10, maxindex - (cusorPos - 17));
      break;
    case 4:
    case 7:
    case 10:
    case 13:
    case 16:
      validpos = false;
      break;
  }

  if (Key == KEY_UP || keyUphold) {
    if (validpos) {
      if (*pVal1 <= (maxval - mul)) *pVal1 += mul;
      funcUpdate = true;
    }
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (validpos) {
      if (*pVal1 >= (minval + mul)) *pVal1 -= mul;
      funcUpdate = true;
    }
  } else if (Key == KEY_LEFT || keyLefthold) {
    if (cusorPos > 0) cusorPos--;
    switch (cusorPos) {
      case 4:
      case 7:
      case 10:
      case 13:
      case 16:
        cusorPos--;
        break;
    }
    funcUpdate = true;
  } else if (Key == KEY_RIGHT || keyRighthold) {
    if (cusorPos < maxsetlen - 1) cusorPos++;
    switch (cusorPos) {
      case 4:
      case 7:
      case 10:
      case 13:
      case 16:
        cusorPos++;
        break;
    }
    funcUpdate = true;
  } else if (Key == KEY_ENTER) {
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    rtc.adjust(DateTime(dnt1.year, dnt1.month, dnt1.day, dnt1.hour, dnt1.minute, dnt1.second));
    // these lines just to update the screen only
    dnt.year = dnt1.year;
    dnt.month = dnt1.month;
    dnt.day = dnt1.day;
    dnt.hour = dnt1.hour;
    dnt.minute = dnt1.minute;
    dnt.second = dnt1.second;
    copyValue = true;
    funcUpdate = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcUpdate = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    xconvert(gStr1, "%d/%02d/%02d %02d:%02d:%02d", dnt1.year, dnt1.month, dnt1.day, dnt1.hour, dnt1.minute, dnt1.second);
    u8g2.drawStr(x, y, gStr1);
    u8g2.drawHLine(x + cusorPos * 6, y + 1, 6);
    u8g2.sendBuffer();  // send to LCD
    funcUpdate = false;
  }
}

//--------------------------
// Cycle Time Setting
//--------------------------
void setCycletimeFunc(u16 x, u16 y) {
  static u16 gVal1 = 0;
  u16 minval = 5;
  u16 maxval = 960;
  u16 val_tmp = 0;

  if (copyValue) {
    gVal1 = status.motor.set_cycletime.all;
    copyValue = false;
  }

  if (Key == KEY_UP || keyUphold || Key == KEY_RIGHT || keyRighthold) {
    if (gVal1 < 240) {
      while (1) {
        if (gVal1 > maxval - 5) break;
        val_tmp = gVal1 + 5;
        if (240 % val_tmp == 0) {
          if (gVal1 <= maxval - 5)
            gVal1 += 5;
          break;
        } else {
          if (gVal1 <= maxval - 5)
            gVal1 += 5;
          else
            break;
        }
      }
    } else  //>=240
    {
      if (gVal1 <= maxval - 240) gVal1 += 240;
    }
    funcUpdate = true;
  }

  if (Key == KEY_DOWN || keyDownhold || Key == KEY_LEFT || keyLefthold) {
    if (gVal1 <= 240) {
      while (1) {
        if (gVal1 < minval + 5) break;
        val_tmp = gVal1 - 5;
        if (240 % val_tmp == 0) {
          if (gVal1 >= minval + 5)
            gVal1 -= 5;
          break;
        } else {
          if (gVal1 >= minval + 5)
            gVal1 -= 5;
          else
            break;
        }
      }
    } else  //>240
    {
      if (gVal1 >= minval + 240) gVal1 -= 240;
    }
    funcUpdate = true;
  }

  if (Key == KEY_ENTER) {
    status.motor.set_cycletime.all = gVal1;
    status.motor.set_cycletime_f = (double)gVal1 / 10.0f;
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    copyValue = true;
    funcUpdate = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  }

  if (Key == KEY_ESC) {
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    copyValue = true;
    funcUpdate = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    if (gVal1 < 240)
      xconvert(gStr1, "%04.1fhour", (double)gVal1 / 10.0f);
    else
      xconvert(gStr1, "%dday     ", gVal1 / 240);
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawStr(x, y, gStr1);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    u8g2.sendBuffer();  // send to LCD
    funcUpdate = false;
  }
}

//---------------------------
//confirmScadaSetting
//---------------------------
void confirmScadaSetting() {
  char usermsg[3][32];
  u8g2.clearBuffer();                // clear lcd ram buffer
  u8g2.setFont(u8g2_font_t0_14_mf);  // set font
  u8g2.setDrawColor(TEXT_COLOR_INVERTED);
  strcpy(usermsg[0], "SYSTEM REBOOT REQUIRED");
  strcpy(usermsg[1], "Do you want to reboot now?");
  strcpy(usermsg[2], "Yes (ENTER) / No (ESC)");
  u8g2.drawStr((MAX_LCD_WIDTH - strlen(usermsg[0]) * 7) / 2, 16, usermsg[0]);
  u8g2.setDrawColor(TEXT_COLOR_NORMAL);
  u8g2.drawStr((MAX_LCD_WIDTH - strlen(usermsg[1]) * 7) / 2, 32, usermsg[1]);
  u8g2.drawStr((MAX_LCD_WIDTH - strlen(usermsg[2]) * 7) / 2, 48, usermsg[2]);
  LcdUpdate();  //Send to display
}

//---------------------------
//Change SCADA Settings #1
//---------------------------
void setScadaFunc1() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    switch (selectedItem) {
      case 0:
        SetParaU8str(128, MENU_LINE1_YPOS, &scada_setting.protocol, 0, 1, protocol_arr);  //0=RTU, 1=ASCII, 2=TCP
        break;
      case 1:
        SetParaU8(128, MENU_LINE2_YPOS, &scada_setting.server_id, 3, 1, 247);  //server_id
        break;
      case 2:
        SetParaUint(128, MENU_LINE3_YPOS, &scada_setting.timeout.all, 5, 100, 65535);  //timeout
        break;
      case 3:
        SetParaUint(128, MENU_LINE4_YPOS, &scada_setting.coil_addr.all, 5, 0, 65535 - scada_setting.max_coil);  //coil_addr
        break;
    }
  }

  if (funcResponse == MENU_FUNC_EXIT_SAVE) {
    if (!sysstat.bit.reboot_pending) {
      confirmScadaSetting();
    }
    funcResponse = MENU_FUNC_CONFIRM;  //wait for user YES/NO confirm
  } else if (funcResponse == MENU_FUNC_CONFIRM) {
    if (!sysstat.bit.reboot_pending) {
      if (Key == KEY_ENTER) {
        writeEEPROMdata();  //save setting
        ESP.restart();
      } else if (Key == KEY_ESC) {
        sysstat.bit.reboot_pending = true;
        funcResponse = MENU_FUNC_EXIT_SAVE;  //return to previous page
      }
    } else {
      funcResponse = MENU_FUNC_EXIT_SAVE;  //return to previous page
    }
  }
}

//---------------------------
//Change SCADA Settings #2
//---------------------------
void setScadaFunc2() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    switch (selectedItem) {
      case 0:
        SetParaUint(MENU_COL2_XPOS, MENU_LINE1_YPOS, &scada_setting.discrete_addr.all, 5, 0, 65535 - scada_setting.max_discrete);  //discrete_addr
        break;
      case 1:
        SetParaUint(MENU_COL2_XPOS, MENU_LINE2_YPOS, &scada_setting.inputreg_addr.all, 5, 0, 65535 - scada_setting.max_inputreg);  //inputreg_addr
        break;
      case 2:
        SetParaUint(MENU_COL2_XPOS, MENU_LINE3_YPOS, &scada_setting.holdreg_addr.all, 5, 0, 65535 - scada_setting.max_holdreg);  //holdreg_addr
        break;
    }
  }

  if (funcResponse == MENU_FUNC_EXIT_SAVE) {
    if (!sysstat.bit.reboot_pending) {
      confirmScadaSetting();
    }
    funcResponse = MENU_FUNC_CONFIRM;  //wait for user YES/NO confirm
  } else if (funcResponse == MENU_FUNC_CONFIRM) {
    if (!sysstat.bit.reboot_pending) {
      if (Key == KEY_ENTER) {
        writeEEPROMdata();  //save setting
        ESP.restart();
      } else if (Key == KEY_ESC) {
        sysstat.bit.reboot_pending = true;
        funcResponse = MENU_FUNC_EXIT_SAVE;  //return to previous page
      }
    } else {
      funcResponse = MENU_FUNC_EXIT_SAVE;  //return to previous page
    }
  }
}

//-------------------------------
//Change SCADA COM port Settings
//-------------------------------
void setComFunc1() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    switch (selectedItem) {
      case 0:
        SelectParaFromList(100, MENU_LINE1_YPOS, &comport.baud.all, baudRate, 12 /*maxitem*/, 6 /*maxclen*/);  //baud 9600-921600
        break;
      case 1:
        SetParaU8(100, MENU_LINE2_YPOS, &comport.databits, 1, 7, 8);  //databits
        break;
      case 2:
        SetParaU8(100, MENU_LINE3_YPOS, &comport.stopbits, 1, 1, 2);  //stopbits
        break;
      case 3:
        SetParaU8str(100, MENU_LINE4_YPOS, &comport.parity, 0, 2, parity_arr);  //parity [0=NONE, 1=ODD, 2=EVEN]
        break;
    }
  }

  if (funcResponse == MENU_FUNC_EXIT_SAVE) {
    if (!sysstat.bit.reboot_pending) {
      confirmScadaSetting();
    }
    funcResponse = MENU_FUNC_CONFIRM;  //wait for user YES/NO confirm
  } else if (funcResponse == MENU_FUNC_CONFIRM) {
    if (!sysstat.bit.reboot_pending) {
      if (Key == KEY_ENTER) {
        writeEEPROMdata();  //save setting
        ESP.restart();
      } else if (Key == KEY_ESC) {
        sysstat.bit.reboot_pending = true;
        funcResponse = MENU_FUNC_EXIT_SAVE;  //return to previous page
      }
    } else {
      funcResponse = MENU_FUNC_EXIT_SAVE;  //return to previous page
    }
  }
}

//-------------------------------
//Compressor Settings
//-------------------------------
//<! Description
//We let the user enter the PSI they want for more user-friendly
//the pressure is checked by the controller to start the motor of not
//wrong setting -> not started
void setCompressorFunc() {
  switch (selectedItem) {
    case 0:
      SetParaInt(58, MENU_TITLE_YPOS, &ctrl.rpm.all, 3, -MAXIMUM_RPM, MAXIMUM_RPM);
      if (Key == KEY_ON || Key == KEY_OFF) {
        //show status
        if (ctrl.onoff == ON) u8g2.drawStr(58, MENU_LINE1_YPOS, "SysOn ");
        else u8g2.drawStr(58, MENU_LINE1_YPOS, "SysOff");
      }
      break;
    case 1:
      SetParaU8str(58, MENU_LINE1_YPOS, &ctrl.onoff, 0, 1, onoff_arr);  //0=Off, 1=On
      break;
    case 2:
      SetParaUint(210, MENU_LINE2_YPOS, &status.motor.start_pressure.all, 3, 1, (status.motor.max_psi_sensor.all - 3));
      break;
    case 3:
      SetParaUint(210, MENU_LINE3_YPOS, &status.motor.stop_pressure.all, 3, 4, status.motor.max_psi_sensor.all);
      break;
    case 4:
      setCycletimeFunc(58, MENU_LINE4_YPOS);
      break;
  }
}

//-------------------------------
// showSavedRecords
//-------------------------------
void showSavedRecords() {
  if (copyValue) {
    u8g2.clearBuffer();  // clear lcd ram buffer
    copyValue = false;
    //scrollbar & title
    u8g2_draw_scrollbar();
    u8g2.setFont(u8g2_font_t0_12_mf);
    u8g2.drawStr(MENU_COL1_XPOS, 10, "Duty Record 2 [Recent->Old]:");
    //Duty record 2 = NO_OF_SAVED_RECORD bytes
    for (int j = 1; j < 5; j++) {
      for (int i = 0; i < 12; i++) {
        xconvert(gStr1, "%03d ", status.motor.saved_record[currentIndexSaved--]);
        if (currentIndexSaved < 0) {
          currentIndexSaved += NO_OF_SAVED_RECORD;
        }
        u8g2.drawStr(i * 20, MENU_TITLE_YPOS + j * 13, gStr1);
      }
    }
  }

  if (Key == KEY_ESC) {
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
    copyValue = true;
  }
}

void checkUserSettings() {
  //warn the user for once when user hits ENTER with wrong psi setting
  if (status.motor.stop_pressure.all - status.motor.start_pressure.all < 3) {
    syserr.bit.setting_value_err = true;
    strcpy(userMsg[0], "WRONG PSI SETTINGS!");
    strcpy(userMsg[1], "THE COMPRESSOR WILL BE");
    strcpy(userMsg[2], "STOPPED ON ERROR!");
  } else {
    syserr.bit.setting_value_err = false;
  }
}

void unlockSetting() {
  static const char pwd_set[] = { "4422586" };  //UP,UP,LEFT,LEFT,DOWN,RIGHT,ENTER
  static const u8 passlen = 7;
  static char pwd_user[passlen + 1] = { "_______" };
  static u8 pwd_count = 0;
  u8 pwdok_count = 0;
  char usermsg[32];

  if (pwd_count < passlen)  //wait for password entry
  {
    u8g2.clearBuffer();                // clear lcd ram buffer
    u8g2.setFont(u8g2_font_t0_12_mf);  // set font
    u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "Enter password:");
    u8g2.drawStr(100, MENU_LINE2_YPOS, pwd_user);
    LcdUpdate();
    if (Key) {
      pwd_user[pwd_count] = Key;
      pwd_count++;
    }
  } else {  //check password
    //verify
    for (int i = 0; i < passlen; i++) {
      if (pwd_user[i] != pwd_set[i]) {
        break;
      } else {
        pwdok_count++;
      }
    }
    if (pwdok_count == passlen) {
      status.motor.unlock_factory_times_left = 10;  //restore the no. of trial
      status.motor.lock_factory_mode = !status.motor.lock_factory_mode;
      u8g2.clearBuffer();                // clear lcd ram buffer
      u8g2.setFont(u8g2_font_t0_14_mf);  // set font
      if (status.motor.lock_factory_mode == false) {
        strcpy(usermsg, "Factory Setting Unlocked!");
      } else {
        strcpy(usermsg, "Factory Setting Locked!");
      }
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(usermsg) * 7) / 2, 32, usermsg);
      LcdUpdate();
      delay(2000);
      funcResponse = MENU_FUNC_EXIT_SAVE;  //leave from here
    } else {
      if (status.motor.unlock_factory_times_left > 0) status.motor.unlock_factory_times_left--;  //decrease the no. of trial by 1
      u8g2.clearBuffer();                                                                        // clear lcd ram buffer
      u8g2.setFont(u8g2_font_t0_14_mf);                                                          // set font
      strcpy(usermsg, "Wrong password!");
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(usermsg) * 7) / 2, 32, usermsg);
      xconvert(usermsg, "You have %02d Chance(s) left.", status.motor.unlock_factory_times_left);
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(usermsg) * 7) / 2, 48, usermsg);
      LcdUpdate();
      delay(2000);
      funcResponse = MENU_FUNC_EXIT_SAVE;  //leave from here
    }
    //clear
    for (int i = 0; i < passlen; i++) {
      pwd_user[i] = '_';
    }
    pwd_count = 0;
  }
  if (Key == KEY_ESC) {
    //clear
    for (int i = 0; i < passlen; i++) {
      pwd_user[i] = '_';
    }
    pwd_count = 0;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;  //return to previous page
  }
}

void factoryFunc1() {
  switch (selectedItem) {
    case 0:
      unlockSetting();
      break;
    case 1:
      SetParaUint(MENU_COL2_XPOS, MENU_LINE2_YPOS, &status.motor.max_psi_sensor.all, 3, 0, MAXIMUM_TANK_PRESSURE);  //maximum psi in psi
      break;
    case 2:
      SetParaUint(MENU_COL2_XPOS, MENU_LINE3_YPOS, &status.motor.zero_psi_sensor_volt.all, 5, 500, 9000);  //zero psi volt in mV
      break;
    case 3:
      SetParaUint(MENU_COL2_XPOS, MENU_LINE4_YPOS, &status.motor.max_psi_sensor_volt.all, 5, 1000, 10000);  //max psi volt in mV
      break;
  }
}

//============================================================================================================================================
// MENU Func MENU Func MENU Func MENU Func MENU Func MENU Func MENU Func MENU Func MENU Func MENU Func MENU Func MENU Func MENU Func MENU Func
//============================================================================================================================================

//-------------------------------------------------------------------
// page1Func1 f1
//-------------------------------------------------------------------
char page1Func1(char input) {
  if (copyValue) {
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawStr(210, MENU_LINE4_YPOS, "SETPAR");
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
  }

  setCompressorFunc();

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE1;
  }

  return PAGE1_FUNC1;
}

//-------------------------------------------------------------------
// page2Func1 f2
//-------------------------------------------------------------------
char page2Func1(char input) {
  if (copyValue) {
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawStr(210, MENU_LINE4_YPOS, "SETPAR");
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
  }

  switch (selectedItem) {
    case 0:
      setRPMfunc();
      break;
    case 1:
      setDatetimeFunc(75, MENU_LINE4_YPOS);
      break;
  }

  if (funcResponse > MENU_FUNC_IDLE) {
    checkUserSettings();
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE2;
  }

  return PAGE2_FUNC1;
}

//-------------------------------------------------------------------
// page3Func1 f3
//-------------------------------------------------------------------
char page3Func1(char input) {
  //No Action

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE3;
  }

  return PAGE3_FUNC1;
}

//-------------------------------------------------------------------
// page4Func1 f4
//-------------------------------------------------------------------
char page4Func1(char input) {
  //No Action

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE4;
  }

  return PAGE4_FUNC1;
}

//-------------------------------------------------------------------
// page5Func1 f5
//-------------------------------------------------------------------
char page5Func1(char input) {
  setUserFunc1();

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE5;
  }

  return PAGE5_FUNC1;
}

//-------------------------------------------------------------------
// page6Func1 f6
//-------------------------------------------------------------------
char page6Func1(char input) {
  setUserFunc2();

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE6;
  }

  return PAGE6_FUNC1;
}

//-------------------------------------------------------------------
// page7Func1 f7
//-------------------------------------------------------------------
char page7Func1(char input) {
  setUserFunc3();

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE7;
  }

  return PAGE7_FUNC1;
}

void updateOCLimitByBoardMotor() {
  if (ctrl.motor_model.byte[1] == 0) {         // Board Series = A
    MINIMUM_OCP_DCAMP = 25;                    // 2.5A
    MAXIMUM_OCP_DCAMP = 250;                   // 25.0A
  } else if (ctrl.motor_model.byte[1] == 1) {  // Board Series = D
    MINIMUM_OCP_DCAMP = 10;                    // 1.0A
    MAXIMUM_OCP_DCAMP = 100;                   // 10.0A
  }
  // set limit
  if (ctrl.ocp_dcamp.all > MAXIMUM_OCP_DCAMP) {
    ctrl.ocp_dcamp.all = MAXIMUM_OCP_DCAMP;
    ctrl.ocp_dcamp_f = (double)ctrl.ocp_dcamp.all / 10.0;
  } else if (ctrl.ocp_dcamp.all < MINIMUM_OCP_DCAMP) {
    ctrl.ocp_dcamp.all = MINIMUM_OCP_DCAMP;
    ctrl.ocp_dcamp_f = (double)ctrl.ocp_dcamp.all / 10.0;
  }
}

//-------------------------------------------------------------------
// page8Func1 f8
//-------------------------------------------------------------------
char page8Func1(char input) {
  static u8 board_series;
  static u8 motor_model;

  switch (selectedItem) {
    case 0:
      SetParaBoolstr(MENU_COL2_XPOS, MENU_LINE1_YPOS, &ctrl.communication_fail_action, comfailaction_arr);
      break;
    case 1:
      if ((funcResponse != MENU_FUNC_CONFIRM) && (funcResponse != MENU_FUNC_WAIT_CONTROLLER)) {
        board_series = ctrl.motor_model.byte[1];
        SetParaU8str(MENU_COL2_XPOS, MENU_LINE2_YPOS, &board_series, 0, 1, brdSeries_arr);
      }
      break;
    case 2:
      if ((funcResponse != MENU_FUNC_CONFIRM) && (funcResponse != MENU_FUNC_WAIT_CONTROLLER)) {
        motor_model = ctrl.motor_model.byte[0];
        SetParaU8str(MENU_COL2_XPOS, MENU_LINE3_YPOS, &motor_model, 0, 4, mtrModel_arr);
      }
      break;
  }

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      if (selectedItem == 1) {
        u8g2.clearBuffer();                // clear lcd ram buffer
        u8g2.setFont(u8g2_font_t0_12_mf);  // set font
        u8g2.setDrawColor(TEXT_COLOR_INVERTED);
        u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS - 6, "WARNING: CHANGE BOARD SERIES");
        u8g2.setDrawColor(TEXT_COLOR_NORMAL);
        u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS - 6, "Warning: controller damage may occur if");
        u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS - 6, "the board and its settings are mismatched.");
        u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS - 6, "Contact Trido Support for assistance.");
        LcdUpdate();                       // Send to display & use size 12 font
        funcResponse = MENU_FUNC_CONFIRM;  //wait for user confirm
      } else if (selectedItem == 2) {
        u8g2.clearBuffer();                // clear lcd ram buffer
        u8g2.setFont(u8g2_font_t0_12_mf);  // set font
        u8g2.setDrawColor(TEXT_COLOR_INVERTED);
        u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS - 6, "WARNING: CHANGE MOTOR MODEL");
        u8g2.setDrawColor(TEXT_COLOR_NORMAL);
        u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS - 6, "Warning: controller damage may occur if");
        u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS - 6, "the connected motor is mismatched!");
        u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS - 6, "Contact Trido Support for assistance.");
        LcdUpdate();                       // Send to display & use size 12 font
        funcResponse = MENU_FUNC_CONFIRM;  //wait for user confirm
      }
      else
      {
        updateEEPROM = true;
        funcResponse = MENU_FUNC_IDLE;
        return MENU_PAGE8;
      }
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      funcResponse = MENU_FUNC_IDLE;
      return MENU_PAGE8;
    } else if (funcResponse == MENU_FUNC_CONFIRM) {
      if (selectedItem == 1) {
        if (Key == KEY_ENTER) {
          ctrl.motor_model.byte[1] = board_series;
          ctrl.reset = true;  // request a reset of controller
          updateEEPROM = true;
          lockKeysExceptEscOff();
          funcResponse = MENU_FUNC_WAIT_CONTROLLER;
        } else if (Key == KEY_ESC) {
          funcResponse = MENU_FUNC_IDLE;
          return MENU_PAGE8;
        }
      } else if (selectedItem == 2) {
        if (Key == KEY_ENTER) {
          ctrl.motor_model.byte[0] = motor_model;
          ctrl.reset = true;  // request a reset of controller
          updateEEPROM = true;
          lockKeysExceptEscOff();
          funcResponse = MENU_FUNC_WAIT_CONTROLLER;
        } else if (Key == KEY_ESC) {
          funcResponse = MENU_FUNC_IDLE;
          return MENU_PAGE8;
        }
      }
    } else if (funcResponse == MENU_FUNC_WAIT_CONTROLLER) {
      if (selectedItem == 1) {
        //wait for controller reset
        if (ctrl.reset_ok == true) {
          updateOCLimitByBoardMotor();
          updateEEPROM = true;
          ctrl.reset_ok = false;
          ctrl.reset = false;         
          lockKeys(0);
          funcResponse = MENU_FUNC_IDLE;
          return MENU_PAGE8;
        } else {
          u8g2.clearBuffer();                // clear lcd ram buffer
          u8g2.setFont(u8g2_font_t0_12_mf);  // set font
          u8g2.setDrawColor(TEXT_COLOR_INVERTED);
          u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS - 6, "SETTING: WAITING FOR CONTROLLER RESPONSE");
          u8g2.setDrawColor(TEXT_COLOR_NORMAL);
          u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS - 6, "Setting: Waiting for controller to apply");
          u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS - 6, "new settings. If there is no response for");
          u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS - 6, "a few seconds, try power cycling the unit.");
          LcdUpdate();  // Send to display & use size 12 font
        }
      } else if (selectedItem == 2) {
        //wait for controller reset
        if (ctrl.reset_ok == true) {
          ctrl.reset_ok = false;
          ctrl.reset = false;
          lockKeys(0);
          funcResponse = MENU_FUNC_IDLE;
          return MENU_PAGE8;
        } else {
          u8g2.clearBuffer();                // clear lcd ram buffer
          u8g2.setFont(u8g2_font_t0_12_mf);  // set font
          u8g2.setDrawColor(TEXT_COLOR_INVERTED);
          u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS - 6, "SETTING: WAITING FOR CONTROLLER RESPONSE");
          u8g2.setDrawColor(TEXT_COLOR_NORMAL);
          u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS - 6, "Setting: Waiting for controller to apply");
          u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS - 6, "new settings. If there is no response for");
          u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS - 6, "a few seconds, try power cycling the unit.");
          LcdUpdate();  // Send to display & use size 12 font
        }
      }
    }
  }
  return PAGE8_FUNC1;
}

//-------------------------------------------------------------------
// page9Func1 f9
//-------------------------------------------------------------------
char page9Func1(char input) {
  setScadaFunc1();

  if ((funcResponse > MENU_FUNC_IDLE) && (funcResponse != MENU_FUNC_CONFIRM)) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE9;
  }

  return PAGE9_FUNC1;
}

//-------------------------------------------------------------------
// page10Func1 f10
//-------------------------------------------------------------------
char page10Func1(char input) {
  setScadaFunc2();

  if ((funcResponse > MENU_FUNC_IDLE) && (funcResponse != MENU_FUNC_CONFIRM)) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE10;
  }

  return PAGE10_FUNC1;
}

//-------------------------------------------------------------------
// page11Func1 f11
//-------------------------------------------------------------------
char page11Func1(char input) {
  setComFunc1();

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE11;
  }

  return PAGE11_FUNC1;
}

//-------------------------------------------------------------------
// page12Func1 f12
//-------------------------------------------------------------------
char page12Func1(char input) {
  if (numHistoryRecords != 0)
  {
    // Serial.println("Inside Page12func1");
    u8g2.clearBuffer(); // clear lcd ram buffer

    // font set to 13b
    u8g2.setFont(u8g2_font_t0_13b_mf); // set font

    // Error and Status Code
    xconvert(gStr1, "ERROR COUNT:%02d", __builtin_popcount(eep.history[toggleHistory].err.all));
    u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, gStr1);
    xconvert(gStr1, "  STAT COUNT:%02d", __builtin_popcount(eep.history[toggleHistory].status.all));
    u8g2.drawStr(MENU_COL2_XPOS, MENU_TITLE_YPOS, gStr1);

    // font set to 12
    u8g2.setFont(u8g2_font_t0_12_mf); // set font

    xconvert(gStr1, "ERR001:%d  ERR002:%d  PSIERR:%d  RPMERR:%d",
             eep.history[toggleHistory].err.bit.data_range_err,
             eep.history[toggleHistory].err.bit.vfd_heartbeat_err,
             eep.history[toggleHistory].err.bit.setting_value_err,
             eep.history[toggleHistory].err.bit.rpm_set_low_err);
    u8g2.drawStr(15, MENU_LINE1_YPOS, gStr1);

    xconvert(gStr1, "SNSERR:%d  OVPERR:%d  UVPERR:%d  BATLOW:%d",
             eep.history[toggleHistory].err.bit.pressure_sensor_error,
             eep.history[toggleHistory].err.bit.over_voltage_err,
             eep.history[toggleHistory].err.bit.under_voltage_err,
             eep.history[toggleHistory].status.bit.low_voltage_warning);
    u8g2.drawStr(15, MENU_LINE2_YPOS, gStr1);

    xconvert(gStr1, "OCPWRN:%d  OCPERR:%d  DHIGH1:%d  DHIGH2:%d",
             eep.history[toggleHistory].status.bit.over_current_warning,
             eep.history[toggleHistory].err.bit.over_current_err,
             eep.history[toggleHistory].status.bit.single_duty_high_warning,
             eep.history[toggleHistory].status.bit.average_duty_high_warning);
    u8g2.drawStr(15, MENU_LINE3_YPOS, gStr1);

    xconvert(gStr1, "REBOOT:%d  ESTERR:%d",
             eep.history[toggleHistory].status.bit.reboot_pending,
             eep.history[toggleHistory].err.bit.vfd_est_err);
    u8g2.drawStr(75, MENU_LINE4_YPOS, gStr1);
    u8g2.sendBuffer();
  }
  else
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;

  if (Key == KEY_ESC) {
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE12;
  }

  return PAGE12_FUNC1;
}

//-------------------------------------------------------------------
// page13Func1 f13
//-------------------------------------------------------------------
char page13Func1(char input) {
  //No Action

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE13;
  }

  return PAGE13_FUNC1;
}

//-------------------------------------------------------------------
// page14Func1 f14
//-------------------------------------------------------------------
char page14Func1(char input) {
  factoryFunc1();

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) updateEEPROM = true;
    funcResponse = MENU_FUNC_IDLE;
    return MENU_PAGE14;
  }

  return PAGE14_FUNC1;
}

//-----------------------------
// getChecksum
//-----------------------------
u16 getChecksum(unsigned int *buf, u16 bytecount) {
  UINT16_T crcval;

  crcval.all = calCrc(buf, bytecount);

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.printf("getChecksum crc:%u%u\n", crcval.byte[0], crcval.byte[1]);
#endif

  return crcval.all;
}

void prepareWrbuf(unsigned int *buf, u16 *bytecount) {
  u16 index = 0;
  UINT16_T crcval;
  
  // VFD = 10 bytes
  buf[index++] = eep.ctrl.rpm.byte[0];
  buf[index++] = eep.ctrl.rpm.byte[1];
  buf[index++] = eep.ctrl.onoff;
  buf[index++] = eep.ctrl.mode;
  buf[index++] = eep.totalturns.byte[0];
  buf[index++] = eep.totalturns.byte[1];
  buf[index++] = eep.totalturns.byte[2];
  buf[index++] = eep.totalturns.byte[3];
  buf[index++] = eep.ctrl.motor_model.byte[0];
  buf[index++] = eep.ctrl.motor_model.byte[1];

  // SCADA settings = 12 bytes
  buf[index++] = eep.scada_setting.protocol;
  buf[index++] = eep.scada_setting.server_id;
  buf[index++] = eep.scada_setting.timeout.byte[0];
  buf[index++] = eep.scada_setting.timeout.byte[1];
  buf[index++] = eep.scada_setting.coil_addr.byte[0];
  buf[index++] = eep.scada_setting.coil_addr.byte[1];
  buf[index++] = eep.scada_setting.discrete_addr.byte[0];
  buf[index++] = eep.scada_setting.discrete_addr.byte[1];
  buf[index++] = eep.scada_setting.inputreg_addr.byte[0];
  buf[index++] = eep.scada_setting.inputreg_addr.byte[1];
  buf[index++] = eep.scada_setting.holdreg_addr.byte[0];
  buf[index++] = eep.scada_setting.holdreg_addr.byte[1];

  // SCADA COM port settings = 8 bytes
  buf[index++] = (u8)eep.comport.config.all;
  buf[index++] = eep.comport.baud.byte[0];
  buf[index++] = eep.comport.baud.byte[1];
  buf[index++] = eep.comport.baud.byte[2];
  buf[index++] = eep.comport.baud.byte[3];
  buf[index++] = eep.comport.databits;
  buf[index++] = eep.comport.stopbits;
  buf[index++] = eep.comport.parity;

  // Compressor = 41 bytes
  buf[index++] = eep.ctrl.min_dcv_input.byte[0];
  buf[index++] = eep.ctrl.min_dcv_input.byte[1];
  buf[index++] = eep.ctrl.max_dcv_input.byte[0];
  buf[index++] = eep.ctrl.max_dcv_input.byte[1];
  buf[index++] = eep.ctrl.slow_down_dcv.byte[0];
  buf[index++] = eep.ctrl.slow_down_dcv.byte[1];
  buf[index++] = eep.ctrl.ocp_dcamp.byte[0];
  buf[index++] = eep.ctrl.ocp_dcamp.byte[1];
  buf[index++] = eep.motor.start_pressure.byte[0];
  buf[index++] = eep.motor.start_pressure.byte[1];
  buf[index++] = eep.motor.stop_pressure.byte[0];
  buf[index++] = eep.motor.stop_pressure.byte[1];
  buf[index++] = eep.motor.set_cycletime.byte[0];
  buf[index++] = eep.motor.set_cycletime.byte[1];
  buf[index++] = eep.motor.runtime_sec;
  buf[index++] = eep.motor.runtime_min;
  buf[index++] = eep.motor.runtime_hour;
  buf[index++] = eep.motor.runtime_day.byte[0];
  buf[index++] = eep.motor.runtime_day.byte[1];
  buf[index++] = eep.motor.cycletime_sec;
  buf[index++] = eep.motor.cycletime_min;
  buf[index++] = eep.motor.cycletime_hour;
  buf[index++] = eep.motor.cycletime_day.byte[0];
  buf[index++] = eep.motor.cycletime_day.byte[1];
  buf[index++] = eep.motor.sensor_psi_offset;
  buf[index++] = eep.motor.single_duty_alarm_percent.byte[0];
  buf[index++] = eep.motor.single_duty_alarm_percent.byte[1];
  buf[index++] = eep.motor.average_duty_alarm_percent.byte[0];
  buf[index++] = eep.motor.average_duty_alarm_percent.byte[1];
  buf[index++] = eep.motor.pressure_control_method;
  buf[index++] = eep.motor.autoreset_motorturns;
  buf[index++] = eep.motor.disable_dio_mode;
  buf[index++] = eep.ctrl.communication_fail_action;
  buf[index++] = eep.motor.lock_factory_mode;
  buf[index++] = eep.motor.unlock_factory_times_left;
  buf[index++] = eep.motor.max_psi_sensor.byte[0];
  buf[index++] = eep.motor.max_psi_sensor.byte[1];
  buf[index++] = eep.motor.zero_psi_sensor_volt.byte[0];
  buf[index++] = eep.motor.zero_psi_sensor_volt.byte[1];
  buf[index++] = eep.motor.max_psi_sensor_volt.byte[0];
  buf[index++] = eep.motor.max_psi_sensor_volt.byte[1];

  //Duty record 2 = 1 + 48 bytes
  buf[index++] = eep.motor.saved_record_index;
  for (int i = 0; i < NO_OF_SAVED_RECORD; i++) {
    buf[index++] = eep.motor.saved_record[i];
  }

  //Fault history = 1 + 50 bytes
  buf[index++] = eep.history_index;
  for (int i = 0; i < MAX_HISTORY_FAULT_RECORD; i++) {
    buf[index++] = (u8)eep.history[i].dnt.year;
    buf[index++] = (u8)eep.history[i].dnt.month;
    buf[index++] = (u8)eep.history[i].dnt.day;
    buf[index++] = (u8)eep.history[i].dnt.hour;
    buf[index++] = (u8)eep.history[i].dnt.minute;
    buf[index++] = (u8)eep.history[i].dnt.second;
    buf[index++] = eep.history[i].err.byte[0];
    buf[index++] = eep.history[i].err.byte[1];
    buf[index++] = eep.history[i].status.byte[0];
    buf[index++] = eep.history[i].status.byte[1];
  }

  crcval.all = getChecksum(buf, index);
  buf[index++] = crcval.byte[0];
  buf[index++] = crcval.byte[1];

  *bytecount = index;

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.printf("prepareWrbuf bytes (plus 2 crc bytes):%u\n", index);  //Monitor via serial port
  Serial.print("");                                                    //Monitor via serial port
  for (int i = 0; i < index; i++) {
    Serial.printf("data:%u", buf[i]);
  }
  Serial.print("\n");
#endif
}

//-----------------------------
// readEEPROMdata
//-----------------------------
void readEEPROMdata(unsigned int *buf, u16 *byteread) {
  u16 index = 0;
  u16 addr = 0;

  // VFD = 10 bytes
  for (int i = 0; i < 10; i++) {
    buf[index++] = EEPROM.read(addr++);
  }

  // SCADA settings = 12 bytes
  for (int i = 0; i < 12; i++) {
    buf[index++] = EEPROM.read(addr++);
  }

  // SCADA COM port = 8 bytes
  for (int i = 0; i < 8; i++) {
    buf[index++] = EEPROM.read(addr++);
  }

  // Compressor = 41 bytes
  for (int i = 0; i < 41; i++) {
    buf[index++] = EEPROM.read(addr++);
  }

  //Duty record 2 = 1 + 48 bytes
  buf[index++] = EEPROM.read(addr++);
  for (int i = 0; i < NO_OF_SAVED_RECORD; i++) {
    buf[index++] = EEPROM.read(addr++);
  }

  //Fault history = 1 + 50 bytes
  buf[index++] = EEPROM.read(addr++);
  for (int i = 0; i < MAX_HISTORY_FAULT_RECORD * TOTAL_BYTES_IN_A_HISTORY_FAULT_RECORD; i++) {
    buf[index++] = EEPROM.read(addr++);
  }

  // CRC = 2 bytes
  buf[index++] = EEPROM.read(addr++);
  buf[index++] = EEPROM.read(addr++);

  *byteread = index;

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.printf("readEEPROM bytes(plus 2 crc bytes):%u\n", index);  //Monitor via serial port
#endif

#if DEBUG_WRITEFLASH_FUNCTION
  for (int i = 0; i < index; i++) {
    Serial.printf("data:%u ", buf[i]);
  }
  Serial.print("\n");
#endif
}

//-----------------------------
// writeEEPROMdata
//-----------------------------
void writeEEPROMdata() {
  bool update_needed = false;
  u16 br;
  u16 bw;

  // if battery voltage is too low probably the pump is powering off, stop recording to avoid data loss, at less than 18V the DC/DC supplying the circuit will not function
  if (status.dcv_f < 20.0) {
    //some errors, we should not be here
    strcpy(userMsg[0], "SAVE SETTINGS FAILED.");
    strcpy(userMsg[1], "Reason: power failure!");
    strcpy(userMsg[2], "Input is less than 20V.");
    return;
  }

  // Prevent too frequent flash write operation
  if (millis() - eepCounter < 100) {
    strcpy(userMsg[1], "SAVE: TOO FREQUENT!");
    Serial.println("writeEEPROM:too frequent, last action is not saved!");  //Monitor via serial port
    eepCounter = millis();
    return;
  }

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.print("writeEEPROM:start\r\n");  //Monitor via serial port
#endif

  eep.ctrl = ctrl;
  eep.scada_setting = scada_setting;
  eep.comport = comport;
  eep.motor = status.motor;

  //read rbuf and get wbuf
  readEEPROMdata(rbuf, &br);  //plus 2 crc bytes
  prepareWrbuf(wbuf, &bw);    //plus 2 crc bytes

  //DEBUG
  if (br != bw) {
#if DEBUG_WRITEFLASH_FUNCTION
    Serial.println("writeEEPROM: write buf index not equal to read buf index!");  //Monitor via serial port
#endif    
    return;
  }

  //compare old data with new data to see if need to write to memory
  //exclude crc bytes
  for (int i = 0; i < bw - 2; i++) {
    if (rbuf[i] != wbuf[i]) {
      update_needed = true;
    }
  }

  if (!update_needed) {
#if DEBUG_WRITEFLASH_FUNCTION
    Serial.println("writeEEPROM: no update needed!");  //Monitor via serial port
    Serial.println("-----------------------------------------------------");
#endif
    return;
  }

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.print("rbuf:");
  for (int i = 0; i < bw - 2; i++) {
    Serial.print(rbuf[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
  Serial.print("wbuf:");
  for (int i = 0; i < bw - 2; i++) {
    Serial.print(wbuf[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
#endif

  // write to EEPROM buffer
  for (int i = 0; i < bw; i++) {
    EEPROM.write(i, wbuf[i]);
  }

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.printf("writeEEPROM bytes (plus 2 crc bytes):%u\n", bw);  //Monitor via serial port
  Serial.print("data:");                                           //Monitor via serial port
  for (int i = 0; i < bw; i++) {
    Serial.print(wbuf[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
#endif

  //check maximum limit
  if (bw > EEPROM_SIZE) {
    strcpy(userMsg[1], "SAVE: OVER LIMIT!");
    return;
  }

  //Program buffer data to hardware flash
  EEPROM.commit();

  //Read back the data written
  readEEPROMdata(rbuf, &br);  //plus 2 crc bytes

  //verify last CRC bytes
  if ((rbuf[br - 2] != wbuf[bw - 2]) || (rbuf[br - 1] != wbuf[bw - 1])) {
    strcpy(userMsg[1], "SAVE: CRC ERROR!");
  }
  //<! DONOT change the above lines
  //------------------------------------------------------

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.println("writeEEPROM:success");  //Monitor flash write via serial port
#endif
  eepCounter = millis();
}

//-----------------------------
// restoreEEPROMdata
//-----------------------------
bool restoreEEPROMdata() {
  bool ret = false;
  UINT16_T crcval;
  u16 index = 0;
  u16 br;

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.print("restoreEEPROM:start\r\n");                                    //Monitor via serial port
  Serial.print("-----------------------------------------------------\r\n");  //Monitor via serial port
#endif

  readEEPROMdata(rbuf, &br);               //plus 2 crc bytes
  crcval.all = getChecksum(rbuf, br - 2);  //calculate CRC

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.printf("br:%u\n", br);
  Serial.printf("restoreEEPROM crc (calculated):%u %u\n", crcval.byte[0], crcval.byte[1]);  //Monitor via serial port
#endif

  // VFD = 10 bytes
  eep.ctrl.rpm.byte[0] = rbuf[index++];
  eep.ctrl.rpm.byte[1] = rbuf[index++];
  eep.ctrl.onoff = rbuf[index++];
  eep.ctrl.mode = rbuf[index++];
  eep.totalturns.byte[0] = rbuf[index++];
  eep.totalturns.byte[1] = rbuf[index++];
  eep.totalturns.byte[2] = rbuf[index++];
  eep.totalturns.byte[3] = rbuf[index++];
  eep.ctrl.motor_model.byte[0] = rbuf[index++];
  eep.ctrl.motor_model.byte[1] = rbuf[index++];

  // SCADA settings = 12 bytes
  eep.scada_setting.protocol = rbuf[index++];
  eep.scada_setting.server_id = rbuf[index++];
  eep.scada_setting.timeout.byte[0] = rbuf[index++];
  eep.scada_setting.timeout.byte[1] = rbuf[index++];
  eep.scada_setting.coil_addr.byte[0] = rbuf[index++];
  eep.scada_setting.coil_addr.byte[1] = rbuf[index++];
  eep.scada_setting.discrete_addr.byte[0] = rbuf[index++];
  eep.scada_setting.discrete_addr.byte[1] = rbuf[index++];
  eep.scada_setting.inputreg_addr.byte[0] = rbuf[index++];
  eep.scada_setting.inputreg_addr.byte[1] = rbuf[index++];
  eep.scada_setting.holdreg_addr.byte[0] = rbuf[index++];
  eep.scada_setting.holdreg_addr.byte[1] = rbuf[index++];

  // SCADA COM port = 8 bytes
  eep.comport.config.all = rbuf[index++];
  eep.comport.baud.byte[0] = rbuf[index++];
  eep.comport.baud.byte[1] = rbuf[index++];
  eep.comport.baud.byte[2] = rbuf[index++];
  eep.comport.baud.byte[3] = rbuf[index++];
  eep.comport.databits = rbuf[index++];
  eep.comport.stopbits = rbuf[index++];
  eep.comport.parity = rbuf[index++];

  // Compressor = 41 bytes
  eep.ctrl.min_dcv_input.byte[0] = rbuf[index++];
  eep.ctrl.min_dcv_input.byte[1] = rbuf[index++];
  eep.ctrl.max_dcv_input.byte[0] = rbuf[index++];
  eep.ctrl.max_dcv_input.byte[1] = rbuf[index++];
  eep.ctrl.slow_down_dcv.byte[0] = rbuf[index++];
  eep.ctrl.slow_down_dcv.byte[1] = rbuf[index++];
  eep.ctrl.ocp_dcamp.byte[0] = rbuf[index++];
  eep.ctrl.ocp_dcamp.byte[1] = rbuf[index++];
  eep.motor.start_pressure.byte[0] = rbuf[index++];
  eep.motor.start_pressure.byte[1] = rbuf[index++];
  eep.motor.stop_pressure.byte[0] = rbuf[index++];
  eep.motor.stop_pressure.byte[1] = rbuf[index++];
  eep.motor.set_cycletime.byte[0] = rbuf[index++];
  eep.motor.set_cycletime.byte[1] = rbuf[index++];
  eep.motor.runtime_sec = rbuf[index++];
  eep.motor.runtime_min = rbuf[index++];
  eep.motor.runtime_hour = rbuf[index++];
  eep.motor.runtime_day.byte[0] = rbuf[index++];
  eep.motor.runtime_day.byte[1] = rbuf[index++];
  eep.motor.cycletime_sec = rbuf[index++];
  eep.motor.cycletime_min = rbuf[index++];
  eep.motor.cycletime_hour = rbuf[index++];
  eep.motor.cycletime_day.byte[0] = rbuf[index++];
  eep.motor.cycletime_day.byte[1] = rbuf[index++];
  eep.motor.sensor_psi_offset = rbuf[index++];
  eep.motor.single_duty_alarm_percent.byte[0] = rbuf[index++];
  eep.motor.single_duty_alarm_percent.byte[1] = rbuf[index++];
  eep.motor.average_duty_alarm_percent.byte[0] = rbuf[index++];
  eep.motor.average_duty_alarm_percent.byte[1] = rbuf[index++];
  eep.motor.pressure_control_method = rbuf[index++];
  eep.motor.autoreset_motorturns = rbuf[index++];
  eep.motor.disable_dio_mode = rbuf[index++];
  eep.ctrl.communication_fail_action = rbuf[index++];
  eep.motor.lock_factory_mode = rbuf[index++];
  eep.motor.unlock_factory_times_left = rbuf[index++];
  eep.motor.max_psi_sensor.byte[0] = rbuf[index++];
  eep.motor.max_psi_sensor.byte[1] = rbuf[index++];
  eep.motor.zero_psi_sensor_volt.byte[0] = rbuf[index++];
  eep.motor.zero_psi_sensor_volt.byte[1] = rbuf[index++];
  eep.motor.max_psi_sensor_volt.byte[0] = rbuf[index++];
  eep.motor.max_psi_sensor_volt.byte[1] = rbuf[index++];

  //Duty record 2 = 1 + 48 bytes
  eep.motor.saved_record_index = rbuf[index++];
  for (int i = 0; i < NO_OF_SAVED_RECORD; i++) {
    eep.motor.saved_record[i] = rbuf[index++];
  }

  //Fault history = 1 + 50 bytes
  eep.history_index = rbuf[index++];
  for (int i = 0; i < MAX_HISTORY_FAULT_RECORD; i++) {
    eep.history[i].dnt.year = rbuf[index++];
    eep.history[i].dnt.month = rbuf[index++];
    eep.history[i].dnt.day = rbuf[index++];
    eep.history[i].dnt.hour = rbuf[index++];
    eep.history[i].dnt.minute = rbuf[index++];
    eep.history[i].dnt.second = rbuf[index++];
    eep.history[i].err.byte[0] = rbuf[index++];
    eep.history[i].err.byte[1] = rbuf[index++];
    eep.history[i].status.byte[0] = rbuf[index++];
    eep.history[i].status.byte[1] = rbuf[index++];
  }

  //verify last CRC bytes
  if ((rbuf[br - 2] != crcval.byte[0])
      || (rbuf[br - 1] != crcval.byte[1])) {
#if DEBUG_WRITEFLASH_FUNCTION
    Serial.println("restoreEEPROM: failed");                            //Monitor via serial port
    Serial.printf("crcval read: %u %u\n", rbuf[br - 2], rbuf[br - 1]);  //Monitor via serial port
#endif
    // Verification Error, set to default
    eepCounter = millis() + 500;
    //write default eep values
    rpm_filt = 0.0;
    totalturns = 0.0;
    eep.totalturns.all = 0;
    // Fault history
    eep.history_index = 0;
    for (int i = 0; i < MAX_HISTORY_FAULT_RECORD; i++) {
      eep.history[i].dnt.year = 0;
      eep.history[i].dnt.month = 0;
      eep.history[i].dnt.day = 0;
      eep.history[i].dnt.hour = 0;
      eep.history[i].dnt.minute = 0;
      eep.history[i].dnt.second = 0;
      eep.history[i].err.all = 0;
      eep.history[i].status.all = 0;
    }
    status.dcv_f = 36.0;  // to skip voltage checking for writeEEPROMdata()
    writeEEPROMdata();
    return ret;
  } else {
    // Verification Succeeded, restore from EEPROM
    ret = true;
  }

  // VFD settings
  ctrl = eep.ctrl;
  ctrl.min_dcv_input_f = (double)ctrl.min_dcv_input.all / 10.0;
  ctrl.max_dcv_input_f = (double)ctrl.max_dcv_input.all / 10.0;
  ctrl.slow_down_dcv_f = (double)ctrl.slow_down_dcv.all / 10.0;
  ctrl.ocp_dcamp_f = (double)ctrl.ocp_dcamp.all / 10.0;
  totalturns = eep.totalturns.all;
  // SCADA settings
  scada_setting = eep.scada_setting;
  // SCADA COM port settings
  comport = eep.comport;
  // Compressor
  status.motor = eep.motor;
  status.motor.set_cycletime_f = (double)status.motor.set_cycletime.all / 10.0f;
  status.motor.cycletime_total = status.motor.cycletime_sec;

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.println("restoreEEPROM: finished");  //Monitor via serial port
#endif
  return ret;
}

//-----------------------------
// taskTimer100ms
//-----------------------------
void taskTimer100ms(void *parameter) {
  for (;;) {
    ms100Counter++;
    vfdErrCounter++;
    vfdFaultRecordCounter++;
    if (syserr.all == 0)
      totalturns += (0.1f * abs(rpm_filt) / 60.0f);
    u32 temp = (u32)totalturns;
    status.totalturns_msb = (u16)(temp >> 16);
    status.totalturns_lsb = (u16)(temp & 0x0000FFFF);
    // VFD heartbeart not received error
    if (vfdErrCounter - vfdHeartbeatCounter >= VFD_COMMUNICATION_TIMEOUT)
      syserr.bit.vfd_heartbeat_err = true;
    else
      syserr.bit.vfd_heartbeat_err = false;
    //Fault history record - give 3 second for the low pass filter to settle
    if (vfdFaultRecordCounter >= 30) {
      if ((syserr.all != 0) && (!faultRecord) && (clockTicking)) {
        faultRecord = true;
        if (eep.history_index > MAX_HISTORY_FAULT_RECORD - 1) eep.history_index = 0;
        eep.history[eep.history_index].dnt = dnt;
        eep.history[eep.history_index].dnt.year = dnt.year - 2000;
        eep.history[eep.history_index].err.all = syserr.all;
        eep.history[eep.history_index].status.all = sysstat.all;
        eep.history_index++;
        if (eep.history_index > MAX_HISTORY_FAULT_RECORD - 1) eep.history_index = 0;
        updateEEPROM = true;  //save setting
      }
      vfdFaultRecordCounter = 0;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);  //100ms
  }
}

//-----------------------------
// kpEvent
//-----------------------------
void kpEvent(KeypadEvent Key) {
  kpState = kp.getState();
  switch (kpState) {
    case PRESSED:
      break;
    case HOLD:
      if (Key == KEY_UP) keyUphold = true;
      else if (Key == KEY_DOWN) keyDownhold = true;
      else if (Key == KEY_LEFT) keyLefthold = true;
      else if (Key == KEY_RIGHT) keyRighthold = true;
      else if (Key == KEY_ENTER) keyEnterhold = true;
      else if (Key == KEY_ESC) keyEschold = true;
      else if (Key == KEY_ON) {
        //Serial.println("ON HOLD");
        // Motor ON/OFF
        if (ctrl.onoff == OFF) {
          if ((ctrl.mode == LOCAL_MODE) || (ctrl.mode == REMOTE_MODE)) {
            ctrl.onoff = ON;
            strcpy(userMsg[1], "SYSTEM ON!");
            if (pFunc == NULL) updateEEPROM = true;  // save the change when we are inside the menu page but not setting function
          } else {
            strcpy(userMsg[0], "DIO MODE CONTROLLED BY DI1");
            strcpy(userMsg[1], "YOU CANNOT PRESS");
            strcpy(userMsg[2], "THE ON KEY!");
          }
        } else {
          strcpy(userMsg[1], "SYSTEM ALREADY ON!");
        }
        keyOnhold = true;
      } else if (Key == KEY_MODE) {
        //Serial.println("L/R HOLD");
        if (ctrl.mode < (2 - status.motor.disable_dio_mode))
          ctrl.mode++;
        else
          ctrl.mode = 0;
        strcpy(userMsg[1], mode_arr[ctrl.mode]);
        updateEEPROM = true;
        if ((ctrl.mode == LOCAL_MODE) || (ctrl.mode == REMOTE_MODE))
          mbServer.start();  // Start ModbusRTU background task
        else
          mbServer.stop();  // Stop ModbusRTU background task
        keyLRhold = true;
      }
      break;
    case RELEASED:
      if (Key == KEY_UP) keyUphold = false;
      else if (Key == KEY_DOWN) keyDownhold = false;
      else if (Key == KEY_LEFT) keyLefthold = false;
      else if (Key == KEY_RIGHT) keyRighthold = false;
      else if (Key == KEY_ENTER) keyEnterhold = false;
      else if (Key == KEY_ESC) keyEschold = false;
      else if (Key == KEY_MODE) {
        //Serial.println("L/R RELEASE");
        keyLRhold = false;
      } else if (Key == KEY_ON) {
        //Serial.println("ON RELEASE");
        keyOnhold = false;
      }
      break;
  }
}

//-------------------------------
// lockKeys
//-------------------------------
void lockKeys(byte lock) {
  kp.lockKey(0, 0, lock);  // lock ESC
  kp.lockKey(0, 1, lock);  // lock LEFT
  kp.lockKey(0, 2, lock);  // lock ON
  kp.lockKey(1, 0, lock);  // lock UP
  kp.lockKey(1, 1, lock);  // lock DOWN
  kp.lockKey(1, 2, lock);  // lock ENTER
  kp.lockKey(2, 0, lock);  // lock L/R
  kp.lockKey(2, 1, lock);  // lock RIGHT
  //kp.lockKey(2, 2, 1); // lock OFF
  //if (Key) Serial.println(Key);  //DEBUG
}

void lockKeysExceptEscOff() {
  //kp.lockKey(0, 0, 1);  // lock ESC
  kp.lockKey(0, 1, 1);  // lock LEFT
  kp.lockKey(0, 2, 1);  // lock ON
  kp.lockKey(1, 0, 1);  // lock UP
  kp.lockKey(1, 1, 1);  // lock DOWN
  kp.lockKey(1, 2, 1);  // lock ENTER
  kp.lockKey(2, 0, 1);  // lock L/R
  kp.lockKey(2, 1, 1);  // lock RIGHT
  //kp.lockKey(2, 2, 1);     // lock OFF
}

//-------------------------------
// taskScankey
//-------------------------------
void taskScankey(void *parameter) {
  for (;;) {
    Key = kp.getKey();  //scan a key
    if (Key) {
      funcUpdate = true;
      switch (Key) {
        case KEY_ESC:
          if (pFunc == NULL)
            selectedItem = 0;
          break;
        case KEY_MODE:
          break;
        case KEY_UP:
          if (pFunc == NULL)
            selectedItem = 0;
          break;
        case KEY_DOWN:
          if (pFunc == NULL)
            selectedItem = 0;
          break;
        case KEY_LEFT:
          if (protectScreen) break;
          u8g2_move_item_index(&selectedItem, menuTotalItem);
          if (nextItem == MENU_PAGE3) {
            toggleDutyRecord = 0;
          }
          if (nextItem == MENU_PAGE12) {
            if (toggleHistory > 0)
              toggleHistory--;
          }
          break;
        case KEY_RIGHT:
          if (protectScreen) break;
          u8g2_move_item_index(&selectedItem, menuTotalItem);
          if (nextItem == MENU_PAGE3) {
            toggleDutyRecord = 1;
          }
          if (nextItem == MENU_PAGE12) {
            if (numHistoryRecords != 0) {
              if ((toggleHistory < 4) && (toggleHistory < numHistoryRecords - 1))
                toggleHistory++;
            }
          }
          break;
        case KEY_ON:
          break;
        case KEY_ENTER:
          break;
        case KEY_OFF:
          if (ctrl.onoff == ON) {
            if (ctrl.mode == DIO_MODE) {
              ctrl.mode = LOCAL_MODE;  //if DIO mode is on, switch back to local mode
              mbServer.start();        // Restart ModbusRTU background task
            }
            strcpy(userMsg[1], "SYSTEM OFF!");
            ctrl.onoff = OFF;
            if (pFunc == NULL) updateEEPROM = true;  //save the change when we are inside the menu page but not setting function
          } else
            strcpy(userMsg[1], "SYSTEM ALREADY OFF!");
          break;
      }

      lockKeys(0);  //release all keys if one of the unlocked keys is pressed

      //reset screen saver state and counter
      screenSaverCount = 0;
      protectScreen = false;
    }

    if (scanMenu) {
      // Menu function call
      if (pFunc) nextItem = pFunc(Key);
      else nextItem = getNextItem(thisItem, Key);

      // Menu scan function
      if (nextItem != thisItem) {
        thisItem = nextItem;
        for (int i = 0; menu_func[i].this_item != 0; i++) {
          if (menu_func[i].this_item == thisItem) {
            pFunc = menu_func[i].pFunc;
            break;
          } else
            pFunc = NULL;
        }
      }
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);  //20ms
  }
}

//-----------------------------
// taskScreenSaver
//-----------------------------
void taskScreenSaver(void *parameter) {
  for (;;) {
    if (protectScreen) {
      int x;
      int y;
      //kp.lockKey(0, 0, 1);  // lock ESC
      kp.lockKey(0, 1, 1);  // lock LEFT
      kp.lockKey(0, 2, 1);  // lock ON
      kp.lockKey(1, 0, 1);  // lock UP
      kp.lockKey(1, 1, 1);  // lock DOWN
      kp.lockKey(1, 2, 1);  // lock ENTER
      kp.lockKey(2, 0, 1);  // lock L/R
      kp.lockKey(2, 1, 1);  // lock RIGHT
      //kp.lockKey(2, 2, 1);     // lock OFF
      u8g2.clearBuffer();                    // clear lcd ram buffer
      u8g2.setFont(u8g2_font_t0_12_mf);      // set font
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);  //set color
      x = random(0, 82);
      y = random(12, 52);
      u8g2.drawStr(x, y, "SCREENSAVER (ESC key to exit)");
      LcdUpdate();
    } else {
      if (screenSaverCount >= SCREEN_SAVER_TIMEOUT) {
        screenSaverCount = 0;
        if (pFunc == NULL) {
          protectScreen = true;
          scanMenu = false;
        }
      } else {
        scanMenu = true;
      }
      screenSaverCount++;
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

//--------------------------------
// debugMbRequest
//--------------------------------
void debugMbRequest(Error err) {
#if DEBUG_MODBUS_ERROR
  if (err != SUCCESS) {
    ModbusError e(err);
    Serial.printf("Error creating request: %02X - %s\n", err, (const char *)e);
  }
#else
  (void)err;
#endif
}

//--------------------------------
// debugMbResponse
//--------------------------------
void debugMbResponse(Error err) {
#if DEBUG_MODBUS_ERROR
  if (err != SUCCESS) {
    ModbusError e(err);
    Serial.printf("Error response: %02X - %s\n", err, (const char *)e);
  }
#else
  (void)err;
#endif
}

//--------------------------------
// taskMbClient
//--------------------------------
void taskMbClient(void *parameter) {
  const u8 WDAT_SIZE = 18;
  u16 wdat[WDAT_SIZE];
  u16 index;

  for (;;) {
    index = 0;
    // Polling address 1, server ID 1, 1 word
    Error err = SUCCESS;
    switch (mbTokenCounter) {
      case FC03_SOMI:
        //reg 18 - 27
        err = mbClient.addRequest(FC03_SOMI /*token*/, 1 /*server_id*/, READ_HOLD_REGISTER /*FC*/, 18 /*reg*/, 10 /*count*/);
#ifdef DEBUG_PRINT_RESPONSE        
        debugMbRequest(err);
#endif        
        break;
      case FC16_MOSI:
        //reg 0 heartbeat out ->
        ctrl.count = ms100Counter;
        wdat[index++] = ctrl.count;
        //reg 1 ON/OFF  out  ->
        wdat[index++] = ctrl.onoff;
        //reg 2 rpm set ->
        temp.rpm_out.all = ctrl.rpm.all;  //int16 rpm
        wdat[index++] = temp.rpm_out.all;
        //reg 3 min dcv input set out ->
        wdat[index++] = ctrl.min_dcv_input.all;
        //reg 4 max dcv input set out ->
        wdat[index++] = ctrl.max_dcv_input.all;
        //reg 5 slow down dcv set out ->
        wdat[index++] = ctrl.slow_down_dcv.all;
        //reg 6 ocp DC amp set out ->
        wdat[index++] = ctrl.ocp_dcamp.all;
        //reg 7 Control mode ->
        wdat[index++] = ctrl.mode;
        //reg 8 Pressure control method ->
        wdat[index++] = status.motor.pressure_control_method;
        //reg 9 start pressure set ->
        wdat[index++] = status.motor.start_pressure.all;
        //reg 10 stop pressure set ->
        wdat[index++] = status.motor.stop_pressure.all;
        //reg 11 Pressure sensor zero psi offset ->
        wdat[index++] = status.motor.sensor_psi_offset;
        //reg 12 Pressure sensor max psi ->
        wdat[index++] = status.motor.max_psi_sensor.all;
        //reg 13 Pressure sensor volt at zero psi ->
        wdat[index++] = status.motor.zero_psi_sensor_volt.all;
        //reg 14 Pressure sensor volt at max psi ->
        wdat[index++] = status.motor.max_psi_sensor_volt.all;
        //reg 15 Communication fail action ->
        wdat[index++] = ctrl.communication_fail_action;
        //reg 16 Motor Model Number
        wdat[index++] = ctrl.motor_model.all;
        //reg 17 Controller reset
        wdat[index++] = ctrl.reset;
        //reg 18 Digital output set by modbus master
        wdat[index++] = ctrl.gpio.all;
        err = mbClient.addRequest(FC16_MOSI /*token*/, 1 /*server_id*/, WRITE_MULT_REGISTERS /*FC*/, 0 /*reg*/, index /*words*/, index * 2 /*bytes*/, wdat);
        //####################################
        //REMEMBER to change the wdat[size]!
        //####################################
        break;
    }
#if DEBUG_MODBUS_ERROR
    if (index > WDAT_SIZE) Serial.println("taskMbClient():index > WDAT_SIZE");
#endif    
    if (err == SUCCESS) {
      mbTokenCounter++;
    }
    if (mbTokenCounter > FC16_MOSI) mbTokenCounter = 0;
    vTaskDelay(50 / portTICK_PERIOD_MS);  //50ms
  }
}

//---------------------------------------------------------------------
//Modbus Client(master) callback function for data responses coming in
//---------------------------------------------------------------------
void handleData(ModbusMessage msg, u32 token) {
#if DEBUG_MODBUS_ERROR
#if DEBUG_PRINT_RESPONSE
  Serial.printf("Response: serverID=%d, FC=%d, Token=%08X, length=%d:\n", msg.getServerID(), msg.getFunctionCode(), token, msg.size());
  for (auto &byte : msg) {
    Serial.printf("%02X ", byte);
  }
  Serial.println("");
#endif
#else
  (void)token;
#endif

  const u8 RDAT_SIZE = 10;
  u16 words = 0;
  u16 rdata[RDAT_SIZE];
  u16 index = 0;

  //FC03
  if (msg.getServerID() == 1 && msg.getFunctionCode() == READ_HOLD_REGISTER) {
    if (token == FC03_SOMI) {
      for (u8 i = 0; i < RDAT_SIZE; i++) {
        msg.get(3 + i * 2, words);
#if DEBUG_MODBUS_ERROR
        Serial.print(words);
        Serial.print(" ");
#endif
        rdata[i] = words;
      }
#if DEBUG_MODBUS_ERROR
      Serial.print("\n");
#endif
      //reg 18 rpm in        <-
      temp.rpm_in.all = rdata[index++];
      temp.rpm_in.all = twos2Int(temp.rpm_in.byte[1], temp.rpm_in.byte[0]);
      status.rpm.all = temp.rpm_in.all;   //int rpm
      rpm_filt = (double)status.rpm.all;  //double val for totalturn counting

      //reg 19 DC bus voltage in       <-
      status.dcv.all = rdata[index++];
      status.dcv.all = twos2Int(status.dcv.byte[1], status.dcv.byte[0]);
      status.dcv_f = (double)status.dcv.all / 10.0f;

      //reg 20 DC bus current in      <-
      status.dca.all = rdata[index++];
      status.dca.all = twos2Int(status.dca.byte[1], status.dca.byte[0]);
      status.dca_f = (double)status.dca.all / 10.0f;

      //reg 21 on/off state in      <-
      status.onoff = rdata[index++];

      //reg 22 error
      sysstat.bit.low_voltage_warning = bitRead(rdata[index], 0);
      sysstat.bit.over_current_warning = bitRead(rdata[index], 1);
      syserr.bit.over_current_err = bitRead(rdata[index], 2);
      syserr.bit.over_voltage_err = bitRead(rdata[index], 3);
      syserr.bit.under_voltage_err = bitRead(rdata[index], 4);
      syserr.bit.pressure_sensor_error = bitRead(rdata[index], 5);
      syserr.bit.rpm_set_low_err = bitRead(rdata[index], 6);  //rpm set low err
      syserr.bit.setting_value_err = bitRead(rdata[index], 7);
      syserr.bit.vfd_est_err = bitRead(rdata[index], 8);  //FOC estimator error
      index++;

      //reg 23 Motor coil current in      <-
      status.motora.all = rdata[index++];
      status.motora.all = twos2Int(status.motora.byte[1], status.motora.byte[0]);
      status.motora_f = (double)status.motora.all / 10.0f;

      //reg 24 heartbeat
      status.count = rdata[index++];

      //reg 25 DIO state
      io.all = rdata[index++];
      //DIO control on/off in <-
      if (io.bit.DI1) {
        if ((ctrl.onoff == OFF) && ((ctrl.mode == DIO_MODE))) {
          ctrl.onoff = ON;
        }
      } else {
        if ((ctrl.onoff == ON) && ((ctrl.mode == DIO_MODE))) {
          ctrl.onoff = OFF;
        }
      }

      //reg 26 Tank Pressure in <-
      status.motor.tank_pressure.all = rdata[index++];  //Tank pressure in     <-
  
      //reg 27
      ctrl.reset_ok = rdata[index++];
      //####################################
      //REMEMBER to change the rdat[size]!
      //####################################

#if DEBUG_MODBUS_ERROR
      if (index > RDAT_SIZE) Serial.println("handleData():index > RDAT_SIZE");
#endif      
      // reset communication counter      
      vfdHeartbeatCounter = vfdErrCounter;
      syserr.bit.vfd_heartbeat_err = false;
    }
  }
}

//---------------------------------------------------------------------
//Modbus Client(master) callback function for error message
//---------------------------------------------------------------------
void handleError(Error err, u32 token) {
  debugMbResponse(err);
}

//---------------------------------------------------------------------
//Modbus Server(slave)
//---------------------------------------------------------------------
//FC03: worker do serve Modbus function code 0x03 (READ_HOLD_REGISTER)
ModbusMessage FC03(ModbusMessage request) {
  ModbusMessage response;  // response message to be sent back
  u16 scada_max_holdregs_addr;
  u16 address;  // requested register address
  u16 words;    // requested number of registers
  u16 reg_address = 0;
  u16 reg_count = 0;

  request.get(2, address);  //get register address from request message
  request.get(4, words);    //get total words from request message

  //TEST CODE
  /*
    for (int i = 0; i < NO_OF_SAVED_RECORD; i++) {
    status.motor.ram_record[i] = i;
    status.motor.saved_record[i] = i + 48;
    }*/

  //scada_setting.max_holdreg = 127; //see type_user.h
  scada_max_holdregs_addr = scada_setting.holdreg_addr.all + scada_setting.max_holdreg - 1;

  // Data address valid?
  if ((address >= scada_setting.holdreg_addr.all) && (address <= scada_max_holdregs_addr)) {
    // Data length valid?
    if ((words > 0) && (words <= (scada_setting.max_holdreg - (address - scada_setting.holdreg_addr.all)))) {
      if (request.getFunctionCode() == READ_HOLD_REGISTER) {
        // Set up message with serverID, FC and length of data
        response.add(request.getServerID(), request.getFunctionCode(), (u8)(words * 2));
        // Fill response with requested data
        reg_address = address;
        if (reg_address == scada_setting.holdreg_addr.all + 0) {
          if (reg_count <= words - 1) {
            response.add((u16)syserr.all);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 1) {
          if (reg_count <= words - 1) {
            response.add((u16)sysstat.all);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 2) {
          if (reg_count <= words - 1) {
            response.add((u16)status.rpm.all);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 3) {
          if (reg_count <= words - 1) {
            response.add((u16)status.dcv.all);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 4) {
          if (reg_count <= words - 1) {
            response.add((u16)status.dca.all);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 5) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motora.all);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 6) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.start_pressure.all);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 7) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.stop_pressure.all);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 8) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.tank_pressure.all);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 9) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.year);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 10) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.month);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 11) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.day);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 12) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.hour);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 13) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.minute);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 14) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.second);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 15) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.cycletime_day.all);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 16) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.cycletime_hour);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 17) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.cycletime_min);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 18) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.cycletime_sec);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 19) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.runtime_day.all);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 20) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.runtime_hour);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 21) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.runtime_min);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 22) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.runtime_sec);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 23) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.duty);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 24) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.single_duty_alarm_percent.all);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 25) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.average_duty_alarm_percent.all);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 26) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.set_cycletime.all);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 27) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.pressure_control_method);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 28) {
          if (reg_count <= words - 1) {
            response.add(status.totalturns_msb);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 29) {
          if (reg_count <= words - 1) {
            response.add(status.totalturns_lsb);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        for (int idx = 0; idx < 48; idx++) {
          if ((reg_address == scada_setting.holdreg_addr.all + 30 + idx)) {
            if (reg_count <= words - 1) {
              response.add((u16)status.motor.ram_record[idx]);  //30-77
              reg_address++;
              reg_count++;
            } else
              return response;
          }
        }
        for (int idx = 0; idx < 48; idx++) {
          if ((reg_address == scada_setting.holdreg_addr.all + 78 + idx)) {
            if (reg_count <= words - 1) {
              response.add((u16)status.motor.saved_record[idx]);  //78-125
              reg_address++;
              reg_count++;
            } else
              return response;
          }
        }
        if (reg_address == scada_setting.holdreg_addr.all + 126) {
          if (reg_count <= words - 1) {
            response.add((u16)ctrl.onoff);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        //<!Maximum support per request is 128 words
      } else  //if the requested function is not supported
        // Set up error response.
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_FUNCTION);
    } else  //if the requested data length is not correct
      // Set up error response.
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
  } else  //if the requested address does not match
    // Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);

  return response;
}

//FC06: worker do serve Modbus function code 0x06 (WRITE SINGLE REGISTER)
ModbusMessage FC06(ModbusMessage request) {
  ModbusMessage response;  // response message to be sent back
  u16 scada_max_holdregs_addr;
  u16 address;  // requested register address
  u16 words;    // requested number of registers

  //if DIO mode is used or local mode, return an error
  if (ctrl.mode == LOCAL_MODE || ctrl.mode == DIO_MODE) {
    response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_FAILURE);
    return response;
  }

  //Exit screensaver
  if (protectScreen == true) {
    screenSaverCount = 0;
    protectScreen = false;
  }

  request.get(2, address);  //get register address from request message
  request.get(4, words);    //get total words from request message

  //scada_setting.max_holdreg = 127; //see type_user.h
  scada_max_holdregs_addr = scada_setting.holdreg_addr.all + scada_setting.max_holdreg - 1;

  // Data address valid?
  if ((address >= scada_setting.holdreg_addr.all) && (address <= scada_max_holdregs_addr)) {
    // Write requested data to requested register
    if (request.getFunctionCode() == WRITE_HOLD_REGISTER) {
      //<!Change RPM
      if (address == scada_setting.holdreg_addr.all + 2) {
        // Data value valid?
        if (((words >= 20) && (words <= MAXIMUM_RPM)) || ((words >= 65536 - MAXIMUM_RPM) && (words <= 65536 - 20))) {
          ctrl.rpm.all = words;
          updateEEPROM = true;  //Save setting
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET RPM!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SYSTEM ON/OFF!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change start pressure
      else if (address == scada_setting.holdreg_addr.all + 6) {
        // Data value valid?
        if ((words >= (status.motor.max_psi_sensor.all - 3)) && (words <= (status.motor.max_psi_sensor.all - 3)) && (words <= status.motor.stop_pressure.all - 3)) {
          status.motor.start_pressure.all = words;
          checkUserSettings();
          updateEEPROM = true;  //Save setting
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET START PRESSURE!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET START PRESSURE!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change stop pressure
      else if (address == scada_setting.holdreg_addr.all + 7) {
        // Data value valid?
        if (((words >= status.motor.max_psi_sensor.all) && (words <= status.motor.max_psi_sensor.all)) && (words >= status.motor.start_pressure.all + 3)) {
          status.motor.stop_pressure.all = words;
          checkUserSettings();
          updateEEPROM = true;  //Save setting
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET STOP PRESSURE!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET STOP PRESSURE!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change Date & Time: year
      else if (address == scada_setting.holdreg_addr.all + 9) {
        // Data value valid?
        if ((words >= 2022) && (words <= 2150)) {
          rtc.adjust(DateTime(words, dnt.month, dnt.day, dnt.hour, dnt.minute, dnt.second));
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change Date & Time: month
      else if (address == scada_setting.holdreg_addr.all + 10) {
        // Data value valid?
        if ((words >= 1) && (words <= 12)) {
          rtc.adjust(DateTime(dnt.year, words, getMaxday(dnt.year, words), dnt.hour, dnt.minute, dnt.second));
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change Date & Time: day
      else if (address == scada_setting.holdreg_addr.all + 11) {
        // Data value valid?
        if ((words >= 1) && (words <= getMaxday(dnt.year, dnt.month))) {
          rtc.adjust(DateTime(dnt.year, dnt.month, words, dnt.hour, dnt.minute, dnt.second));
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change Date & Time: hour
      else if (address == scada_setting.holdreg_addr.all + 12) {
        // Data value valid?
        if (words <= 23) {
          rtc.adjust(DateTime(dnt.year, dnt.month, dnt.day, words, dnt.minute, dnt.second));
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change Date & Time: minute
      else if (address == scada_setting.holdreg_addr.all + 13) {
        // Data value valid?
        if (words <= 59) {
          rtc.adjust(DateTime(dnt.year, dnt.month, dnt.day, dnt.hour, words, dnt.second));
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change Date & Time: second
      else if (address == scada_setting.holdreg_addr.all + 14) {
        // Data value valid?
        if (words <= 59) {
          rtc.adjust(DateTime(dnt.year, dnt.month, dnt.day, dnt.hour, dnt.minute, words));
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET DATE & TIME!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change single_duty_alarm_percent
      else if (address == scada_setting.holdreg_addr.all + 24) {
        // Data value valid?
        if ((words >= 1) && (words <= 100)) {
          status.motor.single_duty_alarm_percent.all = words;
          updateEEPROM = true;  //Save setting
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET SINGLE DUTY ALARM!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET SINGLE DUTY ALARM!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change average_duty_alarm_percent
      else if (address == scada_setting.holdreg_addr.all + 25) {
        // Data value valid?
        if ((words >= 1) && (words <= 100)) {
          status.motor.average_duty_alarm_percent.all = words;
          updateEEPROM = true;  //Save setting
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET AVERAGE DUTY ALARM!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET AVERAGE DUTY ALARM!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change Cycletime (duty cycle average time period)
      else if (address == scada_setting.holdreg_addr.all + 26) {
        // Data value valid?
        if ((words == 5) || (words == 10) || (words == 15) || (words == 20) || (words == 30) || (words == 40) || (words == 60) || (words == 80) || (words == 120) || (words == 240) || (words == 480) || (words == 720) || (words == 960)) {
          status.motor.set_cycletime.all = words;
          updateEEPROM = true;  //Save setting
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET CYCYLE TIME!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET CYCYLE TIME!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change pressure_control_method
      else if (address == scada_setting.holdreg_addr.all + 27) {
        // Data value valid?
        if (words <= 2) {
          status.motor.pressure_control_method = words;
          updateEEPROM = true;  //Save setting
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET PRESSURE CONTROL METHOD!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET PRESSURE CONTROL METHOD!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change system on/off state
      else if (address == scada_setting.holdreg_addr.all + 126) {
        // Data value valid?
        if (words <= 1) {
          ctrl.onoff = (bool)words;
          updateEEPROM = true;  //Save setting
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET SYSTEM ON/OFF!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET SYSTEM ON/OFF!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Any other non-supported functions
      else {
        // Set up error response.
        response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
      }
    } else  //if the requested function is not supported
      // Set up error response.
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_FUNCTION);
  } else  //if the requested address does not match
    // Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);

  return response;
}

//---------------------------------------------------------------------
//Modbus Server(slave) - FC01
//---------------------------------------------------------------------
// FC01: worker do serve Modbus function code 0x01 request (READ SINGLE COIL)
ModbusMessage FC01(ModbusMessage request) {
  ModbusMessage response;
  // Request parameters are first coil and number of coils to read
  u16 output_address = 0;
  u16 output_value = 0;
  u16 coil_addr;

  request.get(2, output_address, output_value);

  // Address valid?
  if (output_address < scada_setting.coil_addr.all) {
    // Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }

  coil_addr = output_address - scada_setting.coil_addr.all;
  mycoils.set(0, ctrl.onoff);
  mycoils.set(1, ACTIVE_IO_BITS, (uint8_t *)&ctrl.gpio.all);

  // Are the parameters valid?
  if (coil_addr < mycoils.coils()) {
    // Get the requested coils from our storage
    vector<uint8_t> coilset = mycoils.slice(coil_addr, output_value);
    // Set up response according to the specs: serverID, function code, number of bytes to follow, packed coils
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)coilset.size(), coilset);
  } else {
    // Something was wrong with the parameters
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
  }
  // Return the response
  return response;
}

//---------------------------------------------------------------------
//Modbus Server(slave)
//---------------------------------------------------------------------
// FC03: worker do serve Modbus function code 0x05 (WRITE SINGLE COIL)
ModbusMessage FC05(ModbusMessage request) {
  ModbusMessage response;
  // Request parameters are coil number and 0x0000 (OFF) or 0xFF00 (ON)
  u16 output_address = 0;
  u16 output_value = 0;
  u16 coil_addr;
  bool value = false;

  //if DIO mode is used or local mode, return an error
  if ((ctrl.mode == LOCAL_MODE) || (ctrl.mode == DIO_MODE)) {
    response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_FAILURE);
    return response;
  }

  //Exit screensaver
  if (protectScreen == true) {
    screenSaverCount = 0;
    protectScreen = false;
  }

  request.get(2, output_address, output_value);

  // Address valid?
  if (output_address < scada_setting.coil_addr.all) {
    // Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    return response;
  }

  coil_addr = output_address - scada_setting.coil_addr.all;
  //scada_setting.max_coil = 2; //see type_user.h
  //scada_max_coil_addr = scada_setting.coil_addr.all + scada_setting.max_coil - 1;

  // Is the coil address valid?
  if (coil_addr < mycoils.coils() && !(coil_addr >= 1 && coil_addr <= 4)) {
    if (output_value == 0x0000 || output_value == 0xFF00) {
        value = (output_value == 0xFF00);
        if (mycoils[coil_addr] == value) // Is the ON/OFF value correct?
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "SET COILS!");
          strcpy(userMsg[2], "RESULT: NO ACTION!");
          response = ECHO_RESPONSE;
      } else {
        // set the coil value
        if (mycoils.set(coil_addr, value)) {
          // Data handling
          if (coil_addr == 0)
          {
            ctrl.onoff = value;
            strcpy(userMsg[0], "REMOTE CONTROL:");
            strcpy(userMsg[1], "SYSTEM ON/OFF!");
            strcpy(userMsg[2], "RESULT: APPLIED!");
          }
          else /// if (coil_addr >= 5 && coil_addr <= 7 )
          {
            ctrl.gpio.all = io.all;
            // value ? ctrl.gpio.all |= (1 <<coil_addr) : ctrl.gpio.all &= ~(1 <<coil_addr);
            ctrl.gpio.all ^= (1 <<coil_addr-1);
            strcpy(userMsg[0], "REMOTE CONTROL:");
            strcpy(userMsg[1], "DO SET/RESET!");
            strcpy(userMsg[2], "RESULT: APPLIED!");
            Serial.print("ctrl.gpio.all: ");
            Serial.println(ctrl.gpio.all,BIN);

            /* test updated values (esp32 to esp32)
            io.bit.DO1 = ctrl.gpio.bit.DO1;
            io.bit.DO2 = ctrl.gpio.bit.DO2;
            io.bit.DO3 = ctrl.gpio.bit.DO3;
            io.bit.DO4 = ctrl.gpio.bit.DO4;
            */
          }
          // All fine, coil was set.
          response = ECHO_RESPONSE;
        } else
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), SERVER_DEVICE_FAILURE);
      }
    } else
      // Set up error response.
      response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
  } else
    // Set up error response.
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);

  // Return the response
  return response;
}

//---------------------------------------------------------------------
// mbServerRegisterWorkers
//---------------------------------------------------------------------
void mbServerRegisterWorkers() {
  mbServer.registerWorker(scada_setting.server_id, READ_HOLD_REGISTER, &FC03);   // Function 03 - READ HOLDING REGISTERS
  mbServer.registerWorker(scada_setting.server_id, WRITE_HOLD_REGISTER, &FC06);  // Function 06 - WRITE HOLDING REGISTERS
  mbServer.registerWorker(scada_setting.server_id, READ_COIL, &FC01);            // Function 01 - READ SINGLE COIL
  mbServer.registerWorker(scada_setting.server_id, WRITE_COIL, &FC05);           // Function 05 - WRITE SINGLE COIL
}

void checkSystemStability()
{
  static long lastSec = 0u;

  if ((ctrl.onoff == sysHOLD) && (millis() - lastSec) >= 5000)
  {
    if (sysStable == false)
    {
      if ((sysstat.all & 6) != 0 || (syserr.all & 7) != 0)
      {
        batStableTime = millis();
        sysStable = false;
      }
      else if ((millis() - batStableTime) > BATTERY_STABLE_TIME)
      {
        batStableTime = millis();
        sysStable = true;
      }
    }
    lastSec = millis();
  }
}

//=========================================================================================================================================================================
// setup()
//=========================================================================================================================================================================
void setup() {
  // Serial
  Serial.begin(SERIAL0_BAUD);  //Serial 0, IO1=TX0, IO3=RX0
  while (!Serial) {};          //Wait for Serial 0
  // Serial2
  Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, MODBUS_CH1_TX_PIN, MODBUS_CH1_RX_PIN); //Serial 2, IO17=TX2, IO16=RX2
  
  while (!Serial2) {};          //Wait for Serial 2
  // LCD
  u8g2.begin();
  u8g2.clearBuffer();                // clear lcd ram buffer
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.setDrawColor(TEXT_COLOR_NORMAL);
  // External RTC
  if (rtc.begin()) {
    //RTC error
#if DEBUG_FUNCTION
    Serial.println("rtc.begin(): Error!");
#endif    
    dnt = { 2000, 1, 1, 0, 0, 0 };  //yymmdd,hhmmss
  } else {
    //RTC ok
    rtc.useBattery();
    checkTime();
  }
  /*
    if (rtc.lostPower()) {
    // When time needs to be set on gStr1 new device, or after power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }*/
  // Keypad
  kp.begin(makeKeymap(keys));
  kp.addEventListener(kpEvent);  // Add an event listener.
  kp.setHoldTime(1000);          // Set minimum hold time
  kp.setScanTime(0);             // Set scan rate in msec, 0 = controlled by task event
  kp.setDebounceTime(80);        // Set debounce time for noise immunity, i.e. the minimum depress period to be treated as PRESSED
  // EEPROM
  EEPROM.begin(EEPROM_SIZE);
  delay(100);
  //<! Initialize unknown variables
  ctrl.rpm.all = MAXIMUM_RPM;
  ctrl.min_dcv_input.all = MINIMUM_DCV_INPUT;
  ctrl.max_dcv_input.all = MAXIMUM_DCV_INPUT;
  ctrl.slow_down_dcv.all = SLOW_DOWN_DCV;
  ctrl.ocp_dcamp.all = MAXIMUM_OCP_DCAMP;
  ctrl.motor_model.byte[0] = 2;  // Motor Model = 50MM
  ctrl.motor_model.byte[1] = 1;  // Controller Series = D
  status.rpm.all = 0;
  status.dcv.all = 0;
  status.dca.all = 0;
  status.motora.all = 0;
  status.motor.tank_pressure.all = 0;
  status.motor.start_pressure.all = 10;
  status.motor.stop_pressure.all = 40;
  temp.rpm_in.all = 0;
  temp.rpm_out.all = 0;
  io.all = 0;
  sysstat.all = 0;
  //compressor only
  status.motor.set_cycletime.all = 5;  //0.5hr
  status.motor.single_duty_alarm_percent.all = SINGLE_DUTY_ALARM_PERCENT;
  status.motor.average_duty_alarm_percent.all = AVERAGE_DUTY_ALARM_PERCENT;
  for (int i = 0; i < NO_OF_SAVED_RECORD; i++) {
    status.motor.saved_record[i] = 0;
    status.motor.ram_record[i] = 0;
  }
  status.motor.max_psi_sensor.all = SENSOR_MAXIMUM_PSI;
  status.motor.zero_psi_sensor_volt.all = SENSOR_ZERO_PSI_MV;    //1V
  status.motor.max_psi_sensor_volt.all = SENSOR_MAXIMUM_PSI_MV;  //5V
  // System Error Bits
  syserr.all = 0;
  // SCADA COM port
  comport.config.all = (u8)SERIAL_8N1;
  comport.baud.all = 115200;
  // SCADA
  scada_setting.timeout.all = 2000;
  scada_setting.coil_addr.all = 0;
  scada_setting.discrete_addr.all = 0;
  scada_setting.inputreg_addr.all = 0;
  scada_setting.holdreg_addr.all = 0;
  // Fault history
  eep.history_index = 0;
  for (int i = 0; i < MAX_HISTORY_FAULT_RECORD; i++) {
    eep.history[i].dnt.year = 0;
    eep.history[i].dnt.month = 0;
    eep.history[i].dnt.day = 0;
    eep.history[i].dnt.hour = 0;
    eep.history[i].dnt.minute = 0;
    eep.history[i].dnt.second = 0;
    eep.history[eep.history_index].err.all = 0;
    eep.history[eep.history_index].status.all = 0;
  }
  // Restore settings
  if (restoreEEPROMdata() == false)  // if last setting is not valid, write the default setting
  {
    if (restoreEEPROMdata() == true)  // write the default setting and read again
    {
      strcpy(userMsg[1], "DEFAULT SETTING APPLIED!");
    } else {
      u8g2.clearBuffer();                // clear lcd ram buffer
      u8g2.setFont(u8g2_font_t0_12_mf);  // set font
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS - 6, "ERROR: MEMORY FAIL");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
      u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS - 6, "Error: Memory failed with retry.");
      u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS - 6, "The user setting data read back");
      u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS - 6, "were not valid.");
      LcdUpdate();  // Send to display & use size 12 font
      while (1)
        ;
    }
  } else {
    strcpy(userMsg[1], "Configuration read successfully.");
  }
  COMport1Config();
  // MENU
  pFunc = NULL;
  thisItem = nextItem = MENU_PAGE1;
  //<! FreeRTOS Task Initialization
  vfdHeartbeatCounter = vfdErrCounter;
  // 100 msec task
  xTaskCreatePinnedToCore(
    taskTimer100ms,    // Task function.
    "taskTimer100ms",  // String with name of task.
    4096,              // Stack size in bytes.
    NULL,              // Parameter passed as input of the task
    1,                 // Priority of the task. Larger is higher
    NULL,              // Task handle.
    1                  // Core 1
  );
  // Scan Key task
  xTaskCreatePinnedToCore(
    taskScankey,    // Task function.
    "taskScankey",  // String with name of task.
    24576,          // Stack size in bytes.
    NULL,           // Parameter passed as input of the task
    1,              // Priority of the task. Larger is higher
    NULL,           // Task handle.
    1               // Core 1
  );
  // Modbus Client Polling task
  xTaskCreatePinnedToCore(
    taskMbClient,    // Task function.
    "taskMbClient",  // String with name of task.
    24576,           // Stack size in bytes.
    NULL,            // Parameter passed as input of the task
    1,               // Priority of the task. Larger is higher
    NULL,            // Task handle.
    0                // Core 0
  );
  // ScreenSaver task
  xTaskCreatePinnedToCore(
    taskScreenSaver,    // Task function.
    "taskScreenSaver",  // String with name of task.
    8192,               // Stack size in bytes.
    NULL,               // Parameter passed as input of the task
    1,                  // Priority of the task. Larger is higher
    NULL,               // Task handle.
    1                   // Core 1
  );
  //Check user settings
  checkUserSettings();
  // MODBUS Server Task (Serial 1), ###initialize Seriall before starting task###
  //Protocol setting
  if (scada_setting.protocol == 0)
    mbServer.useModbusRTU();  //RTU
  else if (scada_setting.protocol == 1)
    mbServer.useModbusASCII(scada_setting.timeout.all);  //ASCII
  mbServerRegisterWorkers();
  if ((ctrl.mode == LOCAL_MODE) || (ctrl.mode == REMOTE_MODE)) {
    mbServer.start();  // Start ModbusRTU background task
  }
  // MODBUS Client Task (Serial 2)
  mbClient.onDataHandler(&handleData);    //Response callback function
  mbClient.onErrorHandler(&handleError);  //Response Error callback function
  mbClient.setTimeout(100);               //Set time out
  mbClient.begin(0);                      // Start ModbusClientRTU background task on core 0
  ctrl.reset = true;                      // request a reset of controller
  checkSystemStability();
                                          /*
    setCpuFrequencyMhz(10);
    int Freq = getCpuFrequencyMhz();
    Serial.print("CPU Freq = ");
    Serial.print(Freq);
    Serial.println(" MHz");
    Freq = getXtalFrequencyMhz();
    Serial.print("XTAL Freq = ");
    Serial.print(Freq);
    Serial.println(" MHz");
    Freq = getApbFrequency();
    Serial.print("APB Freq = ");
    Serial.print(Freq);
    Serial.println(" Hz");
    Serial.println(ESP.getFreeHeap());*/
  // Watchdog
  // Enable panic so that the ESP32 restarts
  // For V5 API
  /*
  const esp_task_wdt_config_t wdtConfig = { WDT_TIMEOUT, 0, false };
  esp_task_wdt_init( &wdtConfig );
  esp_task_wdt_add( NULL );
  wdtResetCount = millis();
  */
  // For V3 API
  wdtResetCount = millis();
  esp_task_wdt_config_t esp_task_wdt = {
    .timeout_ms = WDT_TIMEOUT,
    .idle_core_mask = 0x3,
    .trigger_panic = true
  };
  esp_task_wdt_init(&esp_task_wdt);       //Enable panic so ESP32 restarts
  // esp_task_wdt_init(WDT_TIMEOUT, true);  //Enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                //Add current thread to WDT watch
}

void checkTime() {
  now = rtc.now();  //get time
  if (now.year() == 2000) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    now = rtc.now();  //read adjusted time
#if DEBUG_FUNCTION    
    Serial.println("RTC:adjusting date and time.");
#endif
    //check date and time value
    getTime(now, &dnt);
  }
  //DEBUG
  /*
  Serial.println("Date and Time:");
  Serial.print(now.year());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.print(now.day());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.print(now.second());
  Serial.print("\n");*/
}

//=========================================================================================================================================================================
// loop()
//=========================================================================================================================================================================
void loop() {
  static bool savedToEeprom = true;
  // controller startup reset
  if (startUpReset == true) {
    if (ctrl.reset == true) {
      if (ctrl.reset_ok == true) {
        ctrl.reset_ok = false;
        ctrl.reset = false;
        startUpReset = false;
      }
    }
  }
  // Feed Watchdog every 1s, when program stucks, ESP32 auto-restart
  if (millis() - wdtResetCount >= 1000) {
    if (syserr.bit.vfd_heartbeat_err == true) {
      espResetCounter++;
    } else {
      espResetCounter = 0;
    }
    // reset ESP32 if communication from controller has been lost for 30 seconds
    if (espResetCounter > 30) {
      espResetCounter = 0;
      delay(10000);  // This causes ESP32 reset by watchdog timeout
    } else {
      esp_task_wdt_reset();  //feed watchdog
    }
    //Make sure the below loop still runs when real time clock failed
    if (rtcFail) {
      // dnt.second != lastSec
      dnt.second = 0;
      lastSec++;
    }
    wdtResetCount = millis();
  }
  // Read RTC
  if (rtc.testDev()) {
    clockTicking = false;
    if (rtcRetry++ > 5) {
      rtcFail = true;  //RTC failed
      dnt = { 2000, 0, 0, 0, 0, 0 };
    }
  } else {
    now = rtc.now();  //get time
    //check date and time value
    getTime(now, &dnt);
    clockTicking = true;
    if (rtcRetry > 0) rtcRetry--;
    rtcFail = false;
    //Serial.printf("ms = %lu", millis()); //Watch loop speed
  }

  //
  // Secondly Task
  //
  if (dnt.second != lastSec) {
    //if the motor is actually running we start counting, this flag is reset by a system ON/OFF command
    if (!syserr.bit.vfd_heartbeat_err) {
      if ((status.onoff == ON) && (status.rpm.all > 0))  //if the motor is on and speed is not 0, runtime++
      {
        status.motor.runtime_sec++;  //motor on sec++
      }

      if (ctrl.onoff == ON)  //if the system is on regardless of motor on/off state, time on++
      {
        status.motor.cycletime_sec++;  //motor on+off sec++
      }

      // Check if we had a low voltage condition, and restore the correct state
      if (sysstat.bit.low_voltage_warning)
      {
        if( nPreviousMotorState == MOTOR_STATE_UNDEFINED )
        {
          if ((ctrl.onoff != sysHOLD) && (status.onoff == ON) && (status.rpm.all > 0))
          {
            nPreviousMotorState = MOTOR_STATE_RUN;
          }
          else if ((ctrl.onoff == ON) || (ctrl.onoff == sysHOLD))
          {
            nPreviousMotorState = MOTOR_STATE_HOLD;
            batStableTime = millis();
            sysStable = false;
          }
          else
            nPreviousMotorState = MOTOR_STATE_OFF;
        }
      }
      else
      {
        checkSystemStability();
        // Restore previous state
        if (nPreviousMotorState == MOTOR_STATE_OFF)
        {
          if(( ctrl.mode == LOCAL_MODE ) || ( ctrl.mode == REMOTE_MODE))
          {
            ctrl.onoff = OFF;
          }
        }
        else if (nPreviousMotorState == MOTOR_STATE_RUN || (ctrl.onoff == sysHOLD && sysStable))
        {
          if(( ctrl.mode == LOCAL_MODE ) || ( ctrl.mode == REMOTE_MODE))
          {
            ctrl.onoff = ON;
          }
        }
        else if( nPreviousMotorState == MOTOR_STATE_HOLD )
        {
          if(( ctrl.mode == LOCAL_MODE ) || ( ctrl.mode == REMOTE_MODE))
          {
            ctrl.onoff = sysHOLD;
          }
        }

        // Reset stored state
        nPreviousMotorState = MOTOR_STATE_UNDEFINED;
      }

      // if we use 100*seconds/totalsecs to calculate %, it would overflow in a few days
      // u32 = 2,147,483,647 / 100 = 21,474,836.47 s
      // 21,474,836.47 / 3600 = 5,965 h / 24 = 248 days
      //min
      if (status.motor.runtime_sec >= 60) {
        status.motor.runtime_min++;
        status.motor.runtime_total++;  //++ on 1min base
        status.motor.runtime_sec = 0;
      }
      if (status.motor.cycletime_sec >= 60) {
        status.motor.cycletime_min++;
        status.motor.cycletime_total++;  //++ on 1min base
        status.motor.cycletime_sec = 0;
      }
      //hour
      if (status.motor.runtime_min >= 60) {
        eep.totalturns.all = (u32)totalturns;
        updateEEPROM = true;  //Save total motor turns every hour
        status.motor.runtime_hour++;
        status.motor.runtime_min = 0;
      }
      if (status.motor.cycletime_min >= 60) {
        status.motor.cycletime_hour++;
        status.motor.cycletime_min = 0;
        savedToEeprom = false;
      }
      //day
      if (status.motor.runtime_hour >= 24) {
        status.motor.runtime_day.all++;
        status.motor.runtime_hour = 0;
      }
      if (status.motor.cycletime_hour >= 24) {
        status.motor.cycletime_day.all++;
        status.motor.cycletime_hour = 0;
      }
      // Calculate duty cycle, 0 case is ignored
      if (status.motor.cycletime_total)
        status.motor.duty = 100 * status.motor.runtime_total / status.motor.cycletime_total;
      // set_cycletime 5 = 0.5hr, 10 = 1hr, 240 = 24hr
        // cycletime_total = no. of min, set_cycletime = no. of hour x 10
      if (ctrl.onoff == ON && status.motor.cycletime_total > 0 && status.motor.cycletime_total % 60 == 0 && !savedToEeprom)
      {
        savedToEeprom = true;
        if (eep.motor.saved_record_index >= NO_OF_SAVED_RECORD)
        {
          eep.motor.saved_record_index = 0;
        }
          
        // record the duty for every hour averaged for 24hr, max 24 records fixed
          u16 avg_record_val = 0;
          for (int i = 0; i < NO_OF_RAM_RECORD; i++) {
            avg_record_val += status.motor.ram_record[i];
          }
          avg_record_val /= NO_OF_RAM_RECORD;
          status.motor.saved_record[status.motor.saved_record_index] = avg_record_val;
          // average duty alarm
          if (status.motor.duty >= status.motor.average_duty_alarm_percent.all)
            sysstat.bit.average_duty_high_warning = true;
          else
            sysstat.bit.average_duty_high_warning = false;
          updateEEPROM = true;  // we update the Flash every 48 records averaged only, this action cannot be too frequent
          currentIndexSaved = status.motor.saved_record_index;
          status.motor.saved_record_index++;
          if (startIndexSaved >= NO_OF_RAM_RECORD) {
            status.motor.ram_record_index = 0;
          } else {
            startIndexSaved += NO_OF_RAM_RECORD;
          }

        // record the duty for every cycletime set, max 48 records,
        // minimum interval = set hr, i.e. minimum 1 day records at 1/2 hr interval
        status.motor.ram_record[status.motor.ram_record_index] = status.motor.duty;
        // single duty alarm
        if (status.motor.duty >= status.motor.single_duty_alarm_percent.all)
          sysstat.bit.single_duty_high_warning = true;
        else
          sysstat.bit.single_duty_high_warning = false;
        currentIndexRam = status.motor.ram_record_index;
        status.motor.ram_record_index++;

        // //reset runtime
        // status.motor.runtime_sec = 0;
        // status.motor.runtime_min = 0;
        // status.motor.runtime_hour = 0;
        // status.motor.runtime_day.all = 0;
        // status.motor.runtime_total = 0;
        // //reset cycletime
        // status.motor.cycletime_sec = 0;
        // status.motor.cycletime_min = 0;
        // status.motor.cycletime_hour = 0;
        // status.motor.cycletime_day.all = 0;
        // status.motor.cycletime_total = 0;
        // //reset duty which is update on LCD every 1 min
        // status.motor.duty = 0;
        // //reset counters
        // //status.motor.ram_record_index = 0;
        // //currentIndexRam = 0;
      }
    }
    //
    // Minutely Task
    //
    if (dnt.second == 0) {
      //
      // Hourly Task
      //
      if (dnt.minute == 0) {
        // save data
        //writeEEPROMdata();
        //
        // Daily Task
        //
        if (dnt.hour == 0) {
          // auto-reset at 20xx/xx/xx-00:00:00
          if (status.motor.autoreset_motorturns) {
            if ((dnt.year != 2000) && (dnt.hour == 0) && (dnt.minute == 0) && ((dnt.second == 0) || (dnt.second == 1))) {
              // DAY TURN RESET
              totalturns = 0;
              eep.totalturns.all = 0;
              updateEEPROM = true;
              strcpy(userMsg[0], "AUTORESET:");
              strcpy(userMsg[1], "DAY TURN RESET!");
            }
          }
          //reset every year 20xx/01/01-00:00:00
          if ((dnt.month == 1) && (dnt.day == 1) && (dnt.hour == 0) && (dnt.minute == 0)) {
          }
          //
          //  Monthly Task
          //
          u16 lastday = getMaxday(dnt.year, dnt.month);
          if (dnt.day == lastday) {
            //
            // Yearly Task
            //
            if (dnt.month == 0) {
            }
          }
        }
      }
    }
    lastSec = dnt.second;
  }

  // Update menu
  if ((!protectScreen) && (pFunc == NULL)) {
    if (userMsg[1][0] == '\0')  // If there is no user message, show current menu
    {
      LcdPrepareBuffer();
      LcdUpdate();  //Send to display
    } else          // Display interactive user messages/errors
    {
      //----------------------------------------------------------------
      //User Message User Message User Message User Message User Message
      //----------------------------------------------------------------
      u8g2.clearBuffer();                // clear lcd ram buffer
      u8g2.setFont(u8g2_font_t0_14_mf);  // set font
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[0]) * 7) / 2, 22, userMsg[0]);
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[1]) * 7) / 2, 38, userMsg[1]);
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[2]) * 7) / 2, 54, userMsg[2]);
      LcdUpdate();  //Send to display
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      userMsg[0][0] = '\0';
      userMsg[1][0] = '\0';
      userMsg[2][0] = '\0';
    }
  } else {
    // Cancel message if screensaver is in use
    userMsg[0][0] = '\0';
    userMsg[1][0] = '\0';
    userMsg[2][0] = '\0';
  }

  // Update menu
  if ((!protectScreen) && (pFunc == NULL)) {
    if (userMsg[1][0] == '\0')  // If there is no user message, show current menu
    {
      LcdPrepareBuffer();
      LcdUpdate();  //Send to display
    } else          // Display interactive user messages/errors
    {
      //----------------------------------------------------------------
      //User Message User Message User Message User Message User Message
      //----------------------------------------------------------------
      u8g2.clearBuffer();                // clear lcd ram buffer
      u8g2.setFont(u8g2_font_t0_14_mf);  // set font
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[0]) * 7) / 2, 22, userMsg[0]);
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[1]) * 7) / 2, 38, userMsg[1]);
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[2]) * 7) / 2, 54, userMsg[2]);
      LcdUpdate();  //Send to display
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      userMsg[0][0] = '\0';
      userMsg[1][0] = '\0';
      userMsg[2][0] = '\0';
    }
  } else {
    // Cancel message if screensaver is in use
    userMsg[0][0] = '\0';
    userMsg[1][0] = '\0';
    userMsg[2][0] = '\0';
  }

  sysstat.bit.system_onoff_status = ctrl.onoff;
  sysstat.bit.motor_onoff_status = status.onoff;

  if (updateEEPROM == true) {
    updateEEPROM = false;
    writeEEPROMdata();
  }

}  //loop()

//=========================================================================================================================================================================
// LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate() LcdUpdate()
//=========================================================================================================================================================================
void LcdUpdate() {
  u8g2.setFont(u8g2_font_t0_12_mf);
  u8g2.sendBuffer();  // send to LCD
}

void dispError() {
  //error message
  if (syserr.all != 0) {
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    if (bitRead(syserr.all, errorDisplayCount)) {
      switch (errorDisplayCount)
      {
      case 0:
        if (!syserr.bit.vfd_heartbeat_err)
          strcpy(errorMsg, "DCLERR");  //DC input over-voltage error
        break;
      case 1:
        if (!syserr.bit.vfd_heartbeat_err)
          strcpy(errorMsg, "DCHERR");  //DC input under-voltage error
        break;
      case 2:
        if (!syserr.bit.vfd_heartbeat_err)
          strcpy(errorMsg, "OCPERR");  //Motor over current error
        break;
      case 3:
        strcpy(errorMsg, "PSIERR");  //wrong psi settings set by user
        break;
      case 4:
        strcpy(errorMsg, "ERR001");  //invalid data range read from motor controller
        break;
      case 5:
        strcpy(errorMsg, "ERR002");  //HMI lost communication with motor controller
        break;
      case 6:
        if (!syserr.bit.vfd_heartbeat_err)
          strcpy(errorMsg, "ESTERR");  //FOC estimator control error, reason maybe running current over hardware limit, locked rotor etc.
        break;
      case 7:
        if (!syserr.bit.vfd_heartbeat_err)
          strcpy(errorMsg, "RPMERR");  //wrong RPM setting set by user
        break;
      case 8:
        if (!syserr.bit.vfd_heartbeat_err)
          strcpy(errorMsg, "SNSERR");  //Pressure sensor error
        errorDisplayCount = 0;
        break;
      }
    }
    u8g2.drawStr(210, MENU_LINE4_YPOS, errorMsg);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    errorDisplayCount++;
    warnDisplayCount = 0;
  } else if ((sysstat.all > 3) && (sysstat.all < 128)) {
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    if (bitRead(sysstat.all, warnDisplayCount)) {
      switch (warnDisplayCount) {
        case 2:
          if (!syserr.bit.vfd_heartbeat_err)
            strcpy(errorMsg, "BATLOW");  //DC input low-voltage warning
          break;
        case 3:
          if (!syserr.bit.vfd_heartbeat_err)
            strcpy(errorMsg, "OCPWRN");  //Motor over current warning
          break;
        case 4:
          strcpy(errorMsg, "DHIGH1");  //Single duty high warning
          break;
        case 5:
          strcpy(errorMsg, "DHIGH2");  //average duty high warning
          break;
        case 6:
          strcpy(errorMsg, "REBOOT");  //reboot is pending
          warnDisplayCount = 2;
          break;
      }
    }
    u8g2.drawStr(210, MENU_LINE4_YPOS, errorMsg);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    warnDisplayCount++;
    errorDisplayCount = 0;
  } else  //normal running state
  {
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawStr(210, MENU_LINE4_YPOS, "SYSRUN");
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
  }
}

//----------------------------------------------------------------
//Page 1 [Main Display] p1
//----------------------------------------------------------------
void MenuBufferPage1() {
  INT16_T tmpval;
  int tmpCycleHrs;
  int tmpRuntimeHrs;
  menuPage = 1;
  menuTotalItem = 4;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2_draw_arrow_at_array(menuItemPos2, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_11_mf);  // set font
  //set rpm
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "Set RPM");
  int intvar = twos2Int(ctrl.rpm.byte[1], ctrl.rpm.byte[0]);
  xconvert(gStr1, "%03d", intvar);
  u8g2.drawStr(58, MENU_TITLE_YPOS, gStr1);
  //draw motor direction icon
  u8g2.setFont(u8g2_font_8x13_t_symbols);
  if ((ctrl.mode == LOCAL_MODE) || (ctrl.mode == REMOTE_MODE)) {
    if (ctrl.rpm.all & 0x8000) {
      u8g2.drawGlyph(88, MENU_TITLE_YPOS + 2, 8630);
    } else {
      u8g2.drawGlyph(88, MENU_TITLE_YPOS + 2, 8631);
    }
  } else if (ctrl.mode == DIO_MODE) {
    if (io.bit.DI2) {
      u8g2.drawGlyph(88, MENU_TITLE_YPOS + 2, 8631);
    } else {
      u8g2.drawGlyph(88, MENU_TITLE_YPOS + 2, 8630);
    }
  }
  u8g2.setFont(u8g2_font_t0_11_mf);  // set font
  //end of draw motor direction icon
  //running rpm
  u8g2.drawStr(120, MENU_TITLE_YPOS, "Running RPM");
  intvar = twos2Int(status.rpm.byte[1], status.rpm.byte[0]);
  xconvert(gStr1, "%03d", intvar);
  u8g2.drawStr(210, MENU_TITLE_YPOS, gStr1);
  if (syserr.bit.vfd_heartbeat_err) u8g2.drawStr(210, MENU_TITLE_YPOS, "---");
  //draw motor direction icon
  u8g2.setFont(u8g2_font_8x13_t_symbols);
  if (status.rpm.all & 0x8000) {
    u8g2.drawGlyph(240, MENU_TITLE_YPOS + 2, 8630);
  } else if (status.rpm.all != 0) {
    u8g2.drawGlyph(240, MENU_TITLE_YPOS + 2, 8631);
  }
  u8g2.setFont(u8g2_font_t0_11_mf);  // set font
  //end of draw motor direction icon
  //status
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "Status");
  if (syserr.bit.vfd_heartbeat_err) {
    if (ctrl.onoff == ON) {
      u8g2.drawStr(58, MENU_LINE1_YPOS, "SysOn");
    } else {
      u8g2.drawStr(58, MENU_LINE1_YPOS, "SysOff");
    }
  } else {
    if ((ctrl.onoff == OFF) && (status.onoff == OFF)) {
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(58, MENU_LINE1_YPOS, "SysOff");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    }
    if ((ctrl.onoff == OFF) && (status.onoff == ON)) {
      u8g2.drawStr(58, MENU_LINE1_YPOS, "SysOff");
    }
    if ((ctrl.onoff == sysHOLD)) {
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(58, MENU_LINE1_YPOS, "SysHold");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    }
    if ((ctrl.onoff == ON) && (status.onoff == OFF)) {
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(58, MENU_LINE1_YPOS, "MOff");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    }
    if ((ctrl.onoff == ON) && (status.onoff == ON)) {
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(58, MENU_LINE1_YPOS, "MOn");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    }
  }
  //dc voltage/dc current
  u8g2.drawStr(120, MENU_LINE1_YPOS, "DC IN:");
  xconvert(gStr1, "%04.1fV ", status.dcv_f);
  u8g2.drawStr(168, MENU_LINE1_YPOS, gStr1);
  if (syserr.bit.vfd_heartbeat_err) u8g2.drawStr(168, MENU_LINE1_YPOS, "--.-V");
  xconvert(gStr1, "%04.1fA ", status.dca_f);
  u8g2.drawStr(210, MENU_LINE1_YPOS, gStr1);
  if (syserr.bit.vfd_heartbeat_err) u8g2.drawStr(210, MENU_LINE1_YPOS, "--.-A");
  //tank pressure
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "Pressure");
  tmpval.all = status.motor.tank_pressure.all;
  tmpval.all = twos2Int(tmpval.byte[1], tmpval.byte[0]);
  xconvert(gStr1, "%03dpsi ", tmpval.all);
  u8g2.drawStr(58, MENU_LINE2_YPOS, gStr1);
  //run time
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS, "Run Time");
  if (status.motor.runtime_day.all)
  { tmpRuntimeHrs = (status.motor.runtime_day.all*24) + status.motor.runtime_hour;
    if (tmpRuntimeHrs<1000) xconvert(gStr1, "%03dh-%02dm", tmpRuntimeHrs, status.motor.runtime_min);
    else if (tmpRuntimeHrs<10000) xconvert(gStr1, "%04dh-%02dm", tmpRuntimeHrs, status.motor.runtime_min);
    else if (tmpRuntimeHrs>9999) xconvert(gStr1, "%03dd-%02dh", status.motor.runtime_day.all, status.motor.runtime_hour);}
  else if (status.motor.runtime_hour) xconvert(gStr1, "%02dh-%02dm", status.motor.runtime_hour, status.motor.runtime_min);
  else xconvert(gStr1, "%02dm-%02ds", status.motor.runtime_min, status.motor.runtime_sec);
  u8g2.drawStr(58, MENU_LINE3_YPOS, gStr1);
  //start pressure
  u8g2.drawStr(120, MENU_LINE2_YPOS, "Set Start PSI");
  xconvert(gStr1, "%03dpsi", status.motor.start_pressure.all);
  u8g2.drawStr(210, MENU_LINE2_YPOS, gStr1);
  //stop pressure
  u8g2.drawStr(120, MENU_LINE3_YPOS, "Set Stop PSI");
  xconvert(gStr1, "%03dpsi", status.motor.stop_pressure.all);
  u8g2.drawStr(210, MENU_LINE3_YPOS, gStr1);
  //date and time
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS, "Time On");
  if (status.motor.cycletime_day.all)
  {   tmpCycleHrs = (status.motor.cycletime_day.all*24) + status.motor.cycletime_hour;
    if (tmpCycleHrs<1000) xconvert(gStr1, "%03dh-%02dm", tmpCycleHrs, status.motor.cycletime_min);
    else if (tmpCycleHrs<10000) xconvert(gStr1, "%04dh-%02dm", tmpCycleHrs, status.motor.cycletime_min);
    else if (tmpCycleHrs>9999) xconvert(gStr1, "%03dd-%02dh", status.motor.cycletime_day.all, status.motor.cycletime_hour);}
  else if (status.motor.runtime_hour) xconvert(gStr1, "%02dh-%02dm", status.motor.cycletime_hour, status.motor.cycletime_min);
  else xconvert(gStr1, "%02dm-%02ds", status.motor.cycletime_min, status.motor.cycletime_sec);
  u8g2.drawStr(58, MENU_LINE4_YPOS, gStr1);
  //motor duty
  u8g2.drawStr(120, MENU_LINE4_YPOS, "Duty");
  xconvert(gStr1, "%03d%%", status.motor.duty);
  u8g2.drawStr(150, MENU_LINE4_YPOS, gStr1);
  //error message
  dispError();
}

//----------------------------------------------------------------
//Page 2 [Sub Display] p2
//----------------------------------------------------------------
void MenuBufferPage2() {
  INT16_T tmpval;
  menuPage = 2;
  menuTotalItem = 2;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //u8g2_draw_arrow(75 - 6, MENU_TITLE_YPOS, 13/*font_h*/, menuTotalItem/*totalItem*/);
  u8g2_draw_arrow_at_array(menuItemPos1, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_11_mf);  // set font
  //set rpm
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "Set RPM");
  int intvar = twos2Int(ctrl.rpm.byte[1], ctrl.rpm.byte[0]);
  xconvert(gStr1, "%03d", intvar);
  u8g2.drawStr(75, MENU_TITLE_YPOS, gStr1);
  //draw motor direction icon
  u8g2.setFont(u8g2_font_8x13_t_symbols);
  if ((ctrl.mode == LOCAL_MODE) || (ctrl.mode == REMOTE_MODE)) {
    if (ctrl.rpm.all & 0x8000) {
      u8g2.drawGlyph(105, MENU_TITLE_YPOS + 2, 8630);
    } else {
      u8g2.drawGlyph(105, MENU_TITLE_YPOS + 2, 8631);
    }
  } else if (ctrl.mode == DIO_MODE) {
    if (io.bit.DI2) {
      u8g2.drawGlyph(105, MENU_TITLE_YPOS + 2, 8631);
    } else {
      u8g2.drawGlyph(105, MENU_TITLE_YPOS + 2, 8630);
    }
  }
  u8g2.setFont(u8g2_font_t0_11_mf);  // set font
  //end of draw motor direction icon
  //running rpm
  u8g2.drawStr(120, MENU_TITLE_YPOS, "Running RPM");
  intvar = twos2Int(status.rpm.byte[1], status.rpm.byte[0]);
  xconvert(gStr1, "%03d", intvar);
  u8g2.drawStr(210, MENU_TITLE_YPOS, gStr1);
  if (syserr.bit.vfd_heartbeat_err) u8g2.drawStr(210, MENU_TITLE_YPOS, "---");
  //draw motor direction icon
  u8g2.setFont(u8g2_font_8x13_t_symbols);
  if (status.rpm.all & 0x8000) {
    u8g2.drawGlyph(240, MENU_TITLE_YPOS + 2, 8630);
  } else if (status.rpm.all != 0) {
    u8g2.drawGlyph(240, MENU_TITLE_YPOS + 2, 8631);
  }
  u8g2.setFont(u8g2_font_t0_11_mf);  // set font
  //end of draw motor direction icon
  //status
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "Status");
  if (syserr.bit.vfd_heartbeat_err) {
    if (ctrl.onoff == ON) {
      u8g2.drawStr(75, MENU_LINE1_YPOS, "SysOn");
    } else {
      u8g2.drawStr(75, MENU_LINE1_YPOS, "SysOff");
    }
  } else {
    if ((ctrl.onoff == OFF) && (status.onoff == OFF)) {
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(75, MENU_LINE1_YPOS, "SysOff");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    }
    if ((ctrl.onoff == OFF) && (status.onoff == ON)) {
      u8g2.drawStr(75, MENU_LINE1_YPOS, "SysOff");
    }
    if ((ctrl.onoff == sysHOLD))
    {
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(75, MENU_LINE1_YPOS, "SysHold");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    }
    if ((ctrl.onoff == ON) && (status.onoff == OFF)) {
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(75, MENU_LINE1_YPOS, "MOff");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    }
    if ((ctrl.onoff == ON) && (status.onoff == ON)) {
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(75, MENU_LINE1_YPOS, "MOn");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    }
  }
  //control mode
  u8g2.drawStr(120, MENU_LINE1_YPOS, "Control");
  u8g2.drawStr(210, MENU_LINE1_YPOS, mode_arr[ctrl.mode]);
  //dc voltage
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "DC Voltage");
  xconvert(gStr1, "%04.1fV ", status.dcv_f);
  u8g2.drawStr(75, MENU_LINE2_YPOS, gStr1);
  if (syserr.bit.vfd_heartbeat_err) u8g2.drawStr(75, MENU_LINE2_YPOS, "--.-V");
  //dc current
  u8g2.drawStr(120, MENU_LINE2_YPOS, "DC Current");
  xconvert(gStr1, "%04.1fA ", status.dca_f);
  u8g2.drawStr(210, MENU_LINE2_YPOS, gStr1);
  if (syserr.bit.vfd_heartbeat_err) u8g2.drawStr(210, MENU_LINE2_YPOS, "--.-A");
  //tank pressure
  u8g2.drawStr(120, MENU_LINE3_YPOS, "Tank Pressure");
  tmpval.all = status.motor.tank_pressure.all;
  tmpval.all = twos2Int(tmpval.byte[1], tmpval.byte[0]);
  xconvert(gStr1, "%03dpsi ", tmpval.all);
  u8g2.drawStr(210, MENU_LINE3_YPOS, gStr1);
  //count
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS, "Count");
  xconvert(gStr1, "%05d", status.count);
  u8g2.drawStr(75, MENU_LINE3_YPOS, gStr1);
  if (syserr.bit.vfd_heartbeat_err) u8g2.drawStr(75, MENU_LINE3_YPOS, "-----");
  //date and time
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS, "Date & Time");
  if (!rtcFail) {
    xconvert(gStr1, "%d/%02d/%02d %02d:%02d:%02d", dnt.year, dnt.month, dnt.day, dnt.hour, dnt.minute, dnt.second);
    u8g2.drawStr(75, MENU_LINE4_YPOS, gStr1);
  } else u8g2.drawStr(75, MENU_LINE4_YPOS, "RTC GET DATETIME FAILED.");
  //scrollbar & title
  u8g2_draw_scrollbar();
  //error message
  dispError();
}

//----------------------------------------------------------------
//Page 3 [Week Record] p3
//----------------------------------------------------------------
void MenuBufferPage3() {
  menuPage = 3;
  menuTotalItem = 0;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2.setFont(u8g2_font_t0_12_mf);
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "Duty Record 1 [Recent->Old]:");
  if (!toggleDutyRecord) {
    //RAM stored record = NO_OF_RAM_RECORD bytes
    for (int j = 1; j < 5; j++) {
      for (int i = 0; i < 12; i++) {
        xconvert(gStr1, "%03d ", status.motor.ram_record[currentIndexRam--]);
        if (currentIndexRam < 0) {
          currentIndexRam += NO_OF_RAM_RECORD;
        }
        u8g2.drawStr(i * 20, MENU_TITLE_YPOS + j * 13, gStr1);
      }
    }
  } else {
    u8g2.drawStr(MENU_COL1_XPOS, 10, "Duty Record 2 [Recent->Old]:");
    //FLASH stored record = NO_OF_SAVED_RECORD bytes
    for (int j = 1; j < 5; j++) {
      for (int i = 0; i < 12; i++) {
        xconvert(gStr1, "%03d ", status.motor.saved_record[currentIndexSaved--]);
        if (currentIndexSaved < 0) {
          currentIndexSaved += NO_OF_SAVED_RECORD;
        }
        u8g2.drawStr(i * 20, MENU_TITLE_YPOS + j * 13, gStr1);
      }
    }
  }
}

//----------------------------------------------------------------
//Page 4 Motor Turns p4
//----------------------------------------------------------------
void MenuBufferPage4() {
  menuPage = 4;
  menuTotalItem = 0;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(15, 15, "Accumulated Motor Turns of the day:");
  u8g2.setFont(u8g2_font_t0_22_mf);  // set font
  xconvert(gStr1, "%09d", (u32)totalturns);
  u8g2.drawStr(75, 37, gStr1);
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(15, 58, "Hold ENTER Key for 1s to reset!");
  // Enter Key hold for 1s to reset day turn record to 0
  if (keyEnterhold) {
    totalturns = 0;
    eep.totalturns.all = 0;
    updateEEPROM = true;
    strcpy(userMsg[1], "DAY TURN RESET!");
  }
}

//----------------------------------------------------------------
//Page 5 [COMPRESSOR SETTINGS 1] p5
//----------------------------------------------------------------
void MenuBufferPage5() {
  menuPage = 5;
  menuTotalItem = 3;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2_draw_arrow(MENU_COL2_XPOS - 6, MENU_LINE1_YPOS, 13 /*font_h*/, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "[COMPRESSOR SETTINGS]");
  //1.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "1. Min input voltage");
  xconvert(gStr1, "%04.1fV ", ctrl.min_dcv_input_f);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE1_YPOS, gStr1);
  //2.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "2. Slow-down voltage");
  xconvert(gStr1, "%04.1fV ", ctrl.slow_down_dcv_f);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE2_YPOS, gStr1);
  //3.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS, "3. Over-current limit");
  xconvert(gStr1, "%04.1fA ", ctrl.ocp_dcamp_f);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE3_YPOS, gStr1);
  //4.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS, "4. Max input voltage");
  xconvert(gStr1, "%04.1fV ", ctrl.max_dcv_input_f);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE4_YPOS, gStr1);
  u8g2.drawStr(168, MENU_LINE4_YPOS, "(PRESET)");
}

//----------------------------------------------------------------
//Page 6 [COMPRESSOR SETTINGS 2] p6
//----------------------------------------------------------------
void MenuBufferPage6() {
  menuPage = 6;
  menuTotalItem = 3;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2_draw_arrow(MENU_COL2_XPOS - 6, MENU_LINE1_YPOS, 13 /*font_h*/, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "[COMPRESSOR SETTINGS]");
  //5.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "5. Zero PSI offset");
  xconvert(gStr1, "%02d", status.motor.sensor_psi_offset);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE1_YPOS, gStr1);
  xconvert(gStr1, "[%02d = %d PSI]", status.motor.sensor_psi_offset, status.motor.sensor_psi_offset - 5);
  u8g2.drawStr(168, MENU_LINE1_YPOS, gStr1);  //Hint
  //6.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "6. Single duty alarm");
  xconvert(gStr1, "%03d%%", status.motor.single_duty_alarm_percent);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE2_YPOS, gStr1);
  //7.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS, "7. Average duty alarm");
  xconvert(gStr1, "%03d%%", status.motor.average_duty_alarm_percent);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE3_YPOS, gStr1);
}

//----------------------------------------------------------------
//Page 7 [COMPRESSOR SETTINGS 3] p7
//----------------------------------------------------------------
void MenuBufferPage7() {
  menuPage = 7;
  menuTotalItem = 4;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2_draw_arrow(MENU_COL2_XPOS - 6, MENU_LINE1_YPOS, 13 /*font_h*/, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "[COMPRESSOR SETTINGS]");
  //9.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "9. Pressure Control");
  if (status.motor.pressure_control_method == 0) xconvert(gStr1, "%s", "STOP MOTOR ");
  else if (status.motor.pressure_control_method == 1) xconvert(gStr1, "%s", "DOUT3 OPEN ");
  else if (status.motor.pressure_control_method == 2) xconvert(gStr1, "%s", "DOUT3 CLOSE");
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE1_YPOS, gStr1);
  //10.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "10. Clear duty records");
  if (toggleClearDuty) {
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE2_YPOS, "YES");
  } else {
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE2_YPOS, "NO");
  }
  //11.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS, "11. Reset motor turns");
  if (status.motor.autoreset_motorturns) {
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE3_YPOS, "00:00:00");
  } else {
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE3_YPOS, "MANUAL");
  }
  //12.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS, "12. Disable DIO mode");
  if (status.motor.disable_dio_mode) {
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE4_YPOS, "DISABLE");
  } else {
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE4_YPOS, "ENABLE");
  }
}

//----------------------------------------------------------------
//Page 8 [COMPRESSOR SETTINGS 4] p8
//----------------------------------------------------------------
void MenuBufferPage8() {
  menuPage = 8;
  menuTotalItem = 3;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2_draw_arrow(MENU_COL2_XPOS - 6, MENU_LINE1_YPOS, 13 /*font_h*/, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "[COMPRESSOR SETTINGS]");
  //13.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "13. Comm fail action");
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE1_YPOS, comfailaction_arr[ctrl.communication_fail_action]);
  //14.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "14. Controller Series");
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE2_YPOS, brdSeries_arr[ctrl.motor_model.byte[1]]);
  //15.
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS, "15. Motor model");
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE3_YPOS, mtrModel_arr[ctrl.motor_model.byte[0]]);
}

//----------------------------------------------------------------
//Page 9 [SCADA MODBUS SETTINGS 1] p9
//----------------------------------------------------------------
void MenuBufferPage9() {
  menuPage = 9;
  menuTotalItem = 4;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2_draw_arrow(MENU_COL2_XPOS - 6, MENU_LINE1_YPOS, 13 /*font_h*/, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "[SCADA MODBUS SETTINGS]");
  //protocol
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "1. Protocol");
  if (scada_setting.protocol == 0) xconvert(gStr1, "%s", "RTU  ");
  else if (scada_setting.protocol == 1) xconvert(gStr1, "%s", "ASCII");
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE1_YPOS, gStr1);
  u8g2.drawStr(173, MENU_LINE1_YPOS, "[RTU/ASCII]");  //Hint
  //server_id
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "2. Server ID");
  xconvert(gStr1, "%03d", scada_setting.server_id);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE2_YPOS, gStr1);
  u8g2.drawStr(173, MENU_LINE2_YPOS, "[1-247]");  //Hint
  //timeout
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS, "3. Timeout (ASCII)");
  xconvert(gStr1, "%05d", scada_setting.timeout);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE3_YPOS, gStr1);
  u8g2.drawStr(173, MENU_LINE3_YPOS, "[100-65535]");  //Hint
  //coil_addr
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS, "4. Output Coil Addr");
  xconvert(gStr1, "%05d", scada_setting.coil_addr);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE4_YPOS, gStr1);
  xconvert(gStr1, "[0-%05d]", 65535 - scada_setting.max_coil);
  u8g2.drawStr(173, MENU_LINE4_YPOS, gStr1);  //Hint
}

//----------------------------------------------------------------
//Page 10 [SCADA MODBUS SETTINGS 2] p10
//----------------------------------------------------------------
void MenuBufferPage10() {
  menuPage = 10;
  menuTotalItem = 3;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2_draw_arrow(MENU_COL2_XPOS - 6, MENU_LINE1_YPOS, 13 /*font_h*/, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "[SCADA MODBUS SETTINGS]");
  //discrete_addr
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "5. Discrete Input Addr");
  xconvert(gStr1, "%05d", scada_setting.discrete_addr);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE1_YPOS, gStr1);
  xconvert(gStr1, "[0-%05d]", 65535 - scada_setting.max_discrete);
  u8g2.drawStr(183, MENU_LINE1_YPOS, gStr1);  //Hint
  //inputreg_addr
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "6. Input Register Addr");
  xconvert(gStr1, "%05d", scada_setting.inputreg_addr);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE2_YPOS, gStr1);
  xconvert(gStr1, "[0-%05d]", 65535 - scada_setting.max_inputreg);
  u8g2.drawStr(183, MENU_LINE2_YPOS, gStr1);  //Hint
  //holdreg_addr
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS, "7. Hold Register Addr");
  xconvert(gStr1, "%05d", scada_setting.holdreg_addr);
  u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE3_YPOS, gStr1);
  xconvert(gStr1, "[0-%05d]", 65535 - scada_setting.max_holdreg);
  u8g2.drawStr(183, MENU_LINE3_YPOS, gStr1);  //Hint
}

//----------------------------------------------------------------
//Page 11 [SCADA SERIAL PORT SETTINGS] p11
//----------------------------------------------------------------
void MenuBufferPage11() {
  menuPage = 11;
  menuTotalItem = 4;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2_draw_arrow(100 - 6, MENU_LINE1_YPOS, 13 /*font_h*/, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "[SCADA SERIAL PORT SETTINGS]");
  //1.Baud rate
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "1. Baud rate");
  xconvert(gStr1, "%06d", comport.baud.all);
  u8g2.drawStr(100, MENU_LINE1_YPOS, gStr1);
  //2.Data bits
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "2. Data bits");
  xconvert(gStr1, "%d", comport.databits);
  u8g2.drawStr(100, MENU_LINE2_YPOS, gStr1);
  //3.Stop bits
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS, "3. Stop bits");
  xconvert(gStr1, "%d", comport.stopbits);
  u8g2.drawStr(100, MENU_LINE3_YPOS, gStr1);
  //4.Parity
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS, "4. Parity");
  if (comport.parity == 0) xconvert(gStr1, "%s", "NONE");
  else if (comport.parity == 1) xconvert(gStr1, "%s", "ODD ");
  else if (comport.parity == 2) xconvert(gStr1, "%s", "EVEN");
  u8g2.drawStr(100, MENU_LINE4_YPOS, gStr1);
}

//----------------------------------------------------------------
// Reset Parameters [Triggered by Page 12]
//----------------------------------------------------------------
void resetParams()
{
  // reset runtime
  status.motor.runtime_sec = 0;
  status.motor.runtime_min = 0;
  status.motor.runtime_hour = 0;
  status.motor.runtime_day.all = 0;
  status.motor.runtime_total = 0;
  // reset cycletime
  status.motor.cycletime_sec = 0;
  status.motor.cycletime_min = 0;
  status.motor.cycletime_hour = 0;
  status.motor.cycletime_day.all = 0;
  status.motor.cycletime_total = 0;
  // reset duty which is update on LCD every 1 min
  status.motor.duty = 0;
}

//----------------------------------------------------------------
//Page 12 [HISTORY FAULT RECORD] p12
//----------------------------------------------------------------
void MenuBufferPage12() {
  menuPage = 12;
  menuTotalItem = 0;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  int num_records = 0;
  for (int i = 0; i < 5; i++) {
    if (eep.history[i].err.all != 0) num_records++;
  }
  numHistoryRecords = num_records;
  xconvert(gStr1, "[HISTORY FAULT RECORD] - %d RECORD(s)", numHistoryRecords);
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, gStr1);
  if (eep.history[0].err.all != 0) {
    if (eep.history_index < MAX_HISTORY_FAULT_RECORD) {
      //DATE & TIME
      xconvert(gStr1, "%d/%02d/%02d %02d:%02d:%02d", eep.history[toggleHistory].dnt.year + 2000, eep.history[toggleHistory].dnt.month, eep.history[toggleHistory].dnt.day, eep.history[toggleHistory].dnt.hour, eep.history[toggleHistory].dnt.minute, eep.history[toggleHistory].dnt.second);
      u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, gStr1);
      //ERROR CODE
      xconvert(gStr1, "ERROR COUNT:%02d", __builtin_popcount(eep.history[toggleHistory].err.all));
      u8g2.drawStr(120, MENU_LINE1_YPOS, gStr1);
      //STATUS CODE
      xconvert(gStr1, "STATUS COUNT:%02d", __builtin_popcount(eep.history[toggleHistory].status.all));
      u8g2.drawStr(120, MENU_LINE2_YPOS, gStr1);
      //Record number
      if (numHistoryRecords != 0)
        xconvert(gStr1, "RECORD NUM:%02d/%02d", toggleHistory + 1, numHistoryRecords);
      else
        xconvert(gStr1, "RECORD NUM:%02d/%02d", 0, 0);
      u8g2.drawStr(120, MENU_LINE4_YPOS, gStr1);
    } else {
      //data index error
    }
  } else {
    u8g2.drawStr(75, MENU_LINE2_YPOS, "No records found!");
  }
  // Enter Key hold for 1s to
  if (keyEnterhold) {
    if (syserr.all == 0) {
      // Fault history reset
      eep.history_index = 0;
      for (int i = 0; i < MAX_HISTORY_FAULT_RECORD; i++) {
        eep.history[i].dnt.year = 0;
        eep.history[i].dnt.month = 0;
        eep.history[i].dnt.day = 0;
        eep.history[i].dnt.hour = 0;
        eep.history[i].dnt.minute = 0;
        eep.history[i].dnt.second = 0;
        eep.history[i].err.all = 0;
        eep.history[i].status.all = 0;
      }
      faultRecord = false;
      toggleHistory = 0;
      updateEEPROM = true;  //save setting
      resetParams();
      strcpy(userMsg[1], "FAULT HISTORY RESET!");
    } else {
      strcpy(userMsg[0], "CANNOT RESET HISTORY:" );
      strcpy(userMsg[1], "PLEASE FIRST CLEAR ANY");
      strcpy(userMsg[2], "EXISTING FAULT(S)!");
    }
  }
}
//----------------------------------------------------------------
//Page 13 [SYSTEM INFO] p13
//----------------------------------------------------------------
void MenuBufferPage13() {
  menuPage = 13;
  menuTotalItem = 0;
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  //u8g2_draw_arrow(128 - 6, MENU_LINE3_YPOS, 13 /*font_h*/, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  if (showFactoryMenu == false) {
    u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "[SYSTEM INFO]");
    //1.Hardware Version
    u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "1. Power Ctl Version");
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE1_YPOS, POWERCTL_VERSION_STRING );
    //2.Software Version
    u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "2. System Ctl Version");
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE2_YPOS, SYSTEMCTL_VERSION_STRING );
    //reset
    u8g2.drawStr(32, MENU_LINE4_YPOS, "Hold ENTER to restart controller.");
    // Enter Key hold for 1s to software reset the ESP32
    if (keyEnterhold) {
      u8g2.clearBuffer();  // clear lcd ram buffer
      u8g2.drawStr(80, MENU_LINE2_YPOS, "RESTARTING...");
      LcdUpdate();
      delay(2000);
      ESP.restart();
    }
  }
}

//----------------------------------------------------------------
//Page 14 [FACTORY SETTING] p14
//----------------------------------------------------------------
void MenuBufferPage14() {
  menuPage = 14;
  if (status.motor.lock_factory_mode == false) {
    menuTotalItem = 4;
  } else {
    menuTotalItem = 1;
  }
  u8g2.clearBuffer();  // clear lcd ram buffer
  //scrollbar & title
  u8g2_draw_scrollbar();
  u8g2_draw_arrow(138 - 6, MENU_LINE1_YPOS, 13 /*font_h*/, menuTotalItem /*totalItem*/);
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(MENU_COL1_XPOS, MENU_TITLE_YPOS, "[FACTORY SETTING]");
  //1.Factory Mode
  u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE1_YPOS, "1. Lock Factory Mode");
  if (status.motor.lock_factory_mode == true) {
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE1_YPOS, "LOCKED");
  } else {
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE1_YPOS, "UNLOCKED");
  }
  //2.3.show menu only when unlocked
  if (status.motor.lock_factory_mode == false) {
    //2.Maximum PSI
    u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE2_YPOS, "2. Maximum PSI");
    xconvert(gStr1, "%05d psi", status.motor.max_psi_sensor.all);
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE2_YPOS, gStr1);
    //3.Zero PSI Volt
    u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE3_YPOS, "3. Zero PSI volt");
    xconvert(gStr1, "%05d mV", status.motor.zero_psi_sensor_volt.all);
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE3_YPOS, gStr1);
    //4.Max. PSI Volt
    u8g2.drawStr(MENU_COL1_XPOS, MENU_LINE4_YPOS, "3. Max. PSI volt");
    xconvert(gStr1, "%05d mV", status.motor.max_psi_sensor_volt.all);
    u8g2.drawStr(MENU_COL2_XPOS, MENU_LINE4_YPOS, gStr1);
  }
}

//=========================================================================================================================================================================
// LcdPrepareBuffer() LcdPrepareBuffer() LcdPrepareBuffer() LcdPrepareBuffer() LcdPrepareBuffer() LcdPrepareBuffer() LcdPrepareBuffer() LcdPrepareBuffer() LcdPrepareBuffer
//=========================================================================================================================================================================
void LcdPrepareBuffer() {
  //----------------------------------------------------------------
  //Page 1 [Main Display]
  //----------------------------------------------------------------
  if (nextItem == MENU_PAGE1) {
    MenuBufferPage1();
  }
  //----------------------------------------------------------------
  //Page 2 [Sub Display]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE2) {
    MenuBufferPage2();
  }
  //----------------------------------------------------------------
  //Page 3 [Week Record]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE3) {
    MenuBufferPage3();
  }
  //----------------------------------------------------------------
  //Page 4 Motor Turns
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE4) {
    MenuBufferPage4();
  }
  //----------------------------------------------------------------
  //Page 5 [COMPRESSOR SETTINGS 1]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE5) {
    MenuBufferPage5();
  }
  //----------------------------------------------------------------
  //Page 6 [COMPRESSOR SETTINGS 2]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE6) {
    MenuBufferPage6();
  }
  //----------------------------------------------------------------
  //Page 7 [COMPRESSOR SETTINGS 3]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE7) {
    MenuBufferPage7();
  }
  //----------------------------------------------------------------
  //Page 8 [COMPRESSOR SETTINGS 4]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE8) {
    MenuBufferPage8();
  }
  //----------------------------------------------------------------
  //Page 9 [SCADA MODBUS SETTINGS 1]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE9) {
    MenuBufferPage9();
  }
  //----------------------------------------------------------------
  //Page 10 [SCADA MODBUS SETTINGS 2]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE10) {
    MenuBufferPage10();
  }
  //----------------------------------------------------------------
  //Page 11 [SCADA SERIAL PORT SETTINGS]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE11) {
    MenuBufferPage11();
  }
  //----------------------------------------------------------------
  //Page 12 [HISTORY FAULT RECORD]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE12) {
    MenuBufferPage12();
  }
  //----------------------------------------------------------------
  //Page 13 [SYSTEM INFO]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE13) {
    MenuBufferPage13();
  }
  //----------------------------------------------------------------
  //Page 14 [FACTORY SETTING]
  //----------------------------------------------------------------
  else if (nextItem == MENU_PAGE14) {
    MenuBufferPage14();
  }
}

//--------------------------------------------------------------------------
// NO MORE
//--------------------------------------------------------------------------