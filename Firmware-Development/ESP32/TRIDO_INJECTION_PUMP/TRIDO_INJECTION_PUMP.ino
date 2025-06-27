/******************************************************************************************************
     Project: Injection Pump Application
     Software Version: see SOFTWARE_VERSION_STRING[] below
     Hardware Version: see HARDWARE_VERSION_STRING[] below
     Modified: 01/08/2023
 *******************************************************************************************************/
#include <EEPROM.h>
#include <FS.h>
#include <SD.h>
#include <esp_task_wdt.h>
#include <stdlib.h>

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
#include "src\menu.h"

//<!Modbus Include
#include "src\eModbus\src\ModbusClientRTU.h"
#include "src\eModbus\src\ModbusServerRTU.h"
#include "src\eModbus\src\CoilData.h"

const char HARDWARE_VERSION_STRING[] = "YH_PB01_V6";  // Rf = 0.005
//const char HARDWARE_VERSION_STRING[] = "YH_PB01_V5.001";
const char SOFTWARE_VERSION_STRING[] = "2025.06.27";

//<! Set to 0 for manufacturing
#define DEBUG_MODBUS_ERROR 0
#define DEBUG_PRINT_RESPONSE 0
#define DEBUG_FUNCTION 0
#define DEBUG_WRITEFLASH_FUNCTION 0
#define DEBUG_SD_WRITE_FUNCTION 0
#define DEBUG_PUMP_SETTING 0

//5 seconds Watchdog Timeout
#define WDT_TIMEOUT 5000

//<! Global Variables / Objects--------------------------------------------------------/
// General
char gStr1[42] = "";
u32 ms100Counter = 0;
bool clockTicking = false;

// Watchdog
int wdtResetCount = millis();

// Error
SYSTEM_ERR_T syserr;      // System Error bits
SYSTEM_STATUS_T sysstat;  // System Status/Warning bits
bool faultRecord = false;
u32 vfdErrCounter = 0;
u32 vfdFaultRecordCounter = 0;
u32 vfdHeartbeatCounter = 0;
u32 espResetCounter = 0;

//DIO
IO_T io;

// EEPROM
EEPROM_T eep;
u32 eepCounter = 0;
bool updateEEPROM = false;
unsigned int rbuf[EEPROM_SIZE];
unsigned int wbuf[EEPROM_SIZE];

//SD Datalogger
bool sdIsSaving = false;
bool newFileflag = false;
bool writeFileflag = false;
bool sdState = true;
bool sdEject = false;
char cardTypeStr[5] = { "    " };
char cardSizeStr[14] = { "             " };

//Motor
CONTROL_T ctrl;  // Control Parameters Send To VFD
bool startUpReset = true;
u16 MINIMUM_OCP_DCAMP = 10;   // 1.0A
u16 MAXIMUM_OCP_DCAMP = 100;  // 10.0A

// Pump Setting Error bits (set = error)
struct PUMP_SETTING_ERR_BIT_T {
  bool stroke_length_high_err : 1;  //0
  bool stroke_length_low_err : 1;   //1
  bool stroke_rate_high_err : 1;    //2
  bool stroke_rate_low_err : 1;     //3
  bool rsvd0 : 1;                   //4
  bool rsvd1 : 1;                   //5
  bool rsvd2 : 1;                   //6
  bool rsvd3 : 1;                   //7
};

union PUMP_SETTING_ERR_T {
  struct PUMP_SETTING_ERR_BIT_T bit;
  u8 all;
};

PUMP_SETTING_ERR_T pseterr[NO_OF_PUMPS];

typedef enum {
  CHANGE_STROKE_LENGTH = 0,
  //TEST_MAX_STROKE_LENGTH,
  CHANGE_STROKE_LENGTH_OF_OTHER_PUMPS,
  //TEST_MIN_STROKE_LENGTH
} NEXT_ACTION_e;

NEXT_ACTION_e nextTrial = CHANGE_STROKE_LENGTH;

// Status
STATUS_T status;  // Status Read from VFD

// Keypad
const char keys[KEY_ROWS][KEY_COLS] = {
  { '1', '2', '3' },  // ESC  | UP    | Local/Remote
  { '4', '5', '6' },  // LEFT | DOWN  | RIGHT
  { '7', '8', '9' }   // ON   | ENTER | OFF
};
byte pinRows[KEY_ROWS] = { KEYPAD_R1_PIN, KEYPAD_R2_PIN, KEYPAD_R3_PIN };
byte pinCols[KEY_COLS] = { KEYPAD_C1_PIN, KEYPAD_C2_PIN, KEYPAD_C3_PIN };
Keypad kp = Keypad(makeKeymap(keys), pinRows, pinCols, sizeof(pinRows), sizeof(pinCols));
byte kpState;
bool keyUphold = false;
bool keyOnhold = false;
bool keyDownhold = false;
bool keyLefthold = false;
bool keyRighthold = false;
bool keyEnterhold = false;
bool keyEschold = false;
bool keyLRhold = false;
bool keypadLocked = false;

//Modbus RTU client instance on Serial2
ModbusClientRTU mbClient(Serial2, MODBUS_CH1_EN_PIN, 10 /*queue_limit*/);
//Modbus RTU server instance listening on Serial1 with 2000ms timeout
ModbusServerRTU mbServer(Serial1, 2000, MODBUS_CH2_EN_PIN);
uint32_t mbTokenCounter = 0;

//SCADA
SCADA_SETTING_T scada_setting;
COM_T comport;
CoilData mycoils(scada_setting.max_coil, false);

char volUnitStr[10];   // hold the L or gal unit for volume
char tempUnitStr[11];  // hold the CELCIUS or FAHRENHEIT for temperature

const char Protocol_arr[3][16] = {
  "RTU  ",
  "ASCII",
  "TCP  "
};
const char Parity_arr[3][16] = {
  "NONE",
  "ODD ",
  "EVEN"
};
const u32 baudRate[12] = {
  1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 256000, 512000, 921600
};
const char runtimeReset_arr[2][16] = {
  "NO ACTION",
  "RESET NOW"
};
const char enabledDisabled_arr[2][16] = {
  "ENABLED ",
  "DISABLED"
};
const char comfailAction_arr[2][16] = {
  "NO ACTION ",
  "STOP MOTOR"
};
const char Stroke_arr[6][16] = {
  "1/3\"", "1/2\"", "3/4\"", "1\"  ", "1.5\"", "2.0\""
};
const char Piston_arr[5][16] = {
  "3/16\"", "1/4\" ", "3/8\" ", "1/2\" ", "3/4\" "
};
const char monthStr[12][4] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};
const char volUnit_arr[2][16] = {
  "LITRE ",
  "GALLON"
};
const char tempUnit_arr[2][16] = {
  "CELCIUS   ",
  "FAHRENHEIT"
};
const char brdSeries_arr[2][16] = {
  "A",
  "D"
};
const char mtrModel_arr[6][16] = {
  "TRIDO_25MM",
  "TRIDO_35MM",
  "TRIDO_50MM",
  "TRIDO_70MM",
  "TRIDO_GMI ",
  "TRIDO_70LS",
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
MENU_T *pActiveMenu;
bool copyValue = true;
bool funcUpdate = true;
bool protectScreen = false;
bool protectScreenPrev = false;
FUNCSTATE_E funcResponse = MENU_FUNC_IDLE;
char errorDisplayCount = 0;
char warnDisplayCount = 0;
u16 screenSaverCount = 0;
char userMsg[3][42] = {
  { '\0' }, { '\0' }, { '\0' }
};
char errorMsg[8] = { '\0' };
const u8 menuPosCarbon[10][2] = {
  { 0, MENU_LINE1_YPOS }, { 0, MENU_LINE2_YPOS }, { 0, MENU_LINE3_YPOS }, { 0, MENU_LINE4_YPOS }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 }
};

//-------------------------
// twos2Int
//-------------------------
int twos2Int(u8 msb, u8 lsb) {
  long int val = ((long int)msb << 8) | lsb;
  if ((val & 0x8000)) val -= 65536;
  return (int)val;
}

//-----------------------
// power() x^y
//-----------------------
size_t power(size_t x, size_t y) {
  size_t res = x;
  size_t j = y;

  if (y == 0) return 1;
  while (--j) res = res * 10;
  return res;
}

//------------------------------------------------------------------------
// SetParaFloat (floating point)
//------------------------------------------------------------------------
void SetParaFloat(u16 x, u16 y, u16 *val, u8 maxvlen, u8 precision, u16 minval, u16 maxval) {
  static u16 tmpval = 0;
  static u8 cusorPos = 0;
  u8 maxindex = maxvlen - 1;
  u16 mul;
  u16 div;

  if (copyValue) {
    Key = NO_KEY;
    cusorPos = maxindex;
    tmpval = *val;
    copyValue = false;
  }

#if DEBUG_FUNCTION
  if (cusorPos > maxindex)
    Serial.println("SetParaFloat: cusorPos > maxindex");
  Serial.print("key:");
  Serial.println(Key);
#endif

  mul = power(10, maxindex - cusorPos);

  if (Key == KEY_UP || keyUphold) {
    if (tmpval <= (maxval - mul)) tmpval += mul;
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (tmpval >= (minval + mul)) tmpval -= mul;
    funcUpdate = true;
  } else if (Key == KEY_LEFT) {
    if (cusorPos > 0) cusorPos--;
  } else if (Key == KEY_RIGHT) {
    if (cusorPos < maxindex) cusorPos++;
  } else if (Key == KEY_ENTER) {
    *val = tmpval;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    // clear the area
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawBox(x - 9, y - 9, 9, 11);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    // draw arrow
    u8g2.setFont(u8g2_font_8x13_t_symbols);
    u8g2.drawGlyph(x - 7, y + 1, 9658);
    u8g2.setFont(u8g2_font_t0_12_mf);  // set font
    char tmp[20];
    xconvert(tmp, "%%0%dd", maxvlen - precision);
    //Serial.println(tmp);
    div = power(10, precision);
    xconvert(gStr1, tmp, (int)((double)tmpval / (double)div));
    u8g2.drawStr(x, y, gStr1);
    xconvert(tmp, ".%%0%dd", precision);
    xconvert(gStr1, tmp, tmpval % div);
    u8g2.drawStr(x + (maxvlen - precision) * 6, y, gStr1);
    if (cusorPos >= (maxvlen - precision))
      u8g2.drawHLine(x + (cusorPos + 1) * 6, y + 1, 6);
    else
      u8g2.drawHLine(x + cusorPos * 6, y + 1, 6);
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }

    funcUpdate = false;
  }
}

//------------------------------------------------------------------------
// SetParaBoolstr (bool->gStr1)
//------------------------------------------------------------------------
void SetParaBoolstr(u16 x, u16 y, bool *val, const char p[2][16]) {
  static bool tmpval = false;

  if (copyValue) {
    Key = NO_KEY;
    tmpval = *val;
    copyValue = false;
  }

  if (Key == KEY_UP || Key == KEY_DOWN) {
    tmpval = !tmpval;
    funcUpdate = true;
  } else if (Key == KEY_ENTER) {
    *val = tmpval;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    // clear the area
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawBox(x - 9, y - 9, 9, 11);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    // draw arrow
    u8g2.setFont(u8g2_font_8x13_t_symbols);
    u8g2.drawGlyph(x - 7, y + 1, 9658);
    u8g2.setFont(u8g2_font_t0_12_mf);  // set font
    u8g2.drawStr(x, y, p[tmpval]);
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }
    funcUpdate = false;
  }
}

//--------------------------------------------------------------------------------
// SetParaU8str (u8->gStr1)
//--------------------------------------------------------------------------------
void SetParaU8str(u16 x, u16 y, u8 *val, u8 minval, u8 maxval, const char p[3][16]) {
  static u8 tmpval = 0;

  if (copyValue) {
    Key = NO_KEY;
    tmpval = *val;
    copyValue = false;
  }

  if (Key == KEY_UP || keyUphold) {
    if (tmpval < maxval) tmpval++;
    else tmpval = minval;  //roll over
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (tmpval > minval) tmpval--;
    else tmpval = maxval;  //roll over
    funcUpdate = true;
  } else if (Key == KEY_ENTER) {
    *val = tmpval;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    // clear the area
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawBox(x - 9, y - 9, 9, 11);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    // draw arrow
    u8g2.setFont(u8g2_font_8x13_t_symbols);
    u8g2.drawGlyph(x - 7, y + 1, 9658);
    u8g2.setFont(u8g2_font_t0_12_mf);  // set font
    u8g2.drawStr(x, y, p[tmpval]);
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }
    funcUpdate = false;
  }
}

//-----------------------------------------------------------------------------
// SelectParaFromList p (u32)
//-----------------------------------------------------------------------------
void SelectParaFromList(u16 x, u16 y, u32 *val, const u32 *p, u8 maxitem, u8 maxclen) {
  static u32 tmpval = 0;
  static u8 listIndex = 0;
  u8 k = maxitem;
  u8 j = maxitem - 1;

  if (copyValue) {
    Key = NO_KEY;
    tmpval = *val;
    while (k--) {
      if (listIndex > j) {
        listIndex = j;
        break;
      }
      if (tmpval != p[listIndex]) {
        listIndex++;
      } else break;
    }
    copyValue = false;
  }

  if (Key == KEY_UP || keyUphold) {
    if (listIndex < j) listIndex++;
    else listIndex = 0;
    tmpval = p[listIndex];
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (listIndex > 0) listIndex--;
    else listIndex = j;
    tmpval = p[listIndex];
    funcUpdate = true;
  } else if (Key == KEY_ENTER) {
    *val = tmpval;
    listIndex = 0;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    listIndex = 0;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    // clear the area
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawBox(x - 9, y - 9, 9, 11);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    // draw arrow
    u8g2.setFont(u8g2_font_8x13_t_symbols);
    u8g2.drawGlyph(x - 7, y + 1, 9658);
    u8g2.setFont(u8g2_font_t0_12_mf);  // set font

    char tmp[20];
    xconvert(tmp, "%%0%dd", maxclen);
    xconvert(gStr1, tmp, tmpval);
    u8g2.drawStr(x, y, gStr1);
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }
    funcUpdate = false;
  }
}

//------------------------------------------------------------------------
// SetParaU8 (u8)
//------------------------------------------------------------------------
void SetParaU8(u16 x, u16 y, u8 *val, u8 maxvlen, u8 minval, u8 maxval) {
  static u8 tmpval = 0;
  static u8 cusorPos = 0;
  u8 maxindex = maxvlen - 1;
  u8 mul;

  if (copyValue) {
    Key = NO_KEY;
    cusorPos = maxindex;
    tmpval = *val;
    copyValue = false;
  }

#if DEBUG_FUNCTION
  if (cusorPos > maxindex)
    Serial.println("SetParaU8: cusorPos > maxindex");
  Serial.print("key:");
  Serial.println(Key);
#endif

  mul = power(10, maxindex - cusorPos);

  if (Key == KEY_UP || keyUphold) {
    if (tmpval <= (maxval - mul)) tmpval += mul;
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (tmpval >= (minval + mul)) tmpval -= mul;
    funcUpdate = true;
  } else if (Key == KEY_LEFT) {
    if (cusorPos > 0) cusorPos--;
  } else if (Key == KEY_RIGHT) {
    if (cusorPos < maxindex) cusorPos++;
  } else if (Key == KEY_ENTER) {
    *val = tmpval;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    // clear the area
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawBox(x - 9, y - 9, 9, 11);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    // draw arrow
    u8g2.setFont(u8g2_font_8x13_t_symbols);
    u8g2.drawGlyph(x - 7, y + 1, 9658);
    u8g2.setFont(u8g2_font_t0_12_mf);  // set font

    char tmp[10];
    xconvert(tmp, "%%0%dd", maxvlen);
    xconvert(gStr1, tmp, tmpval);
    u8g2.drawStr(x, y, gStr1);
    u8g2.drawHLine(x + cusorPos * 6, y + 1, 6);
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }
    funcUpdate = false;
  }
}

//------------------------------------------------------------------------
// SetParaUint (u16)
//------------------------------------------------------------------------
void SetParaUint(u16 x, u16 y, u16 *val, u8 maxvlen, u16 minval, u16 maxval) {
  static u16 tmpval = 0;
  static u8 cusorPos = 0;
  u8 maxindex = maxvlen - 1;
  u16 mul;

  if (copyValue) {
    Key = NO_KEY;
    cusorPos = maxindex;
    tmpval = *val;
    copyValue = false;
  }

#if DEBUG_FUNCTION
  if (cusorPos > maxindex)
    Serial.println("SetParaUint: cusorPos > maxindex");
  Serial.printf("key:%u\n", Key);
#endif

  mul = power(10, maxindex - cusorPos);

  if (Key == KEY_UP || keyUphold) {
    if (tmpval <= (maxval - mul)) tmpval += mul;
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (tmpval >= (minval + mul)) tmpval -= mul;
    funcUpdate = true;
  } else if (Key == KEY_LEFT) {
    if (cusorPos > 0) cusorPos--;
  } else if (Key == KEY_RIGHT) {
    if (cusorPos < maxindex) cusorPos++;
  } else if (Key == KEY_ENTER) {
    *val = tmpval;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    // clear the area
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawBox(x - 9, y - 9, 9, 11);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    // draw arrow
    u8g2.setFont(u8g2_font_8x13_t_symbols);
    u8g2.drawGlyph(x - 7, y + 1, 9658);
    u8g2.setFont(u8g2_font_t0_12_mf);  // set font

    char tmp[10];
    xconvert(tmp, "%%0%dd", maxvlen);
    xconvert(gStr1, tmp, tmpval);
    u8g2.drawStr(x, y, gStr1);
    u8g2.drawHLine(x + cusorPos * 6, y + 1, 6);
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }
    funcUpdate = false;
  }
}

//------------------------------------------------------------------------
// SetParaInt (int16)
//------------------------------------------------------------------------
void SetParaInt(u16 x, u16 y, int *val, int maxvlen, int minval, int maxval) {
  static int tmpval = 0;
  static u8 cusorPos = 0;
  static int prevNum = 0;
  u8 maxindex = maxvlen - 1;
  int mul;
  INT16_T tmp;

  if (copyValue) {
    Key = NO_KEY;
    tmpval = *val;
    tmp.all = tmpval;
    tmpval = twos2Int(tmp.byte[1], tmp.byte[0]);
    if (tmpval >= 0) cusorPos = maxindex;
    else cusorPos = maxvlen;
    copyValue = false;
  } else {
    tmp.all = tmpval;
    tmpval = twos2Int(tmp.byte[1], tmp.byte[0]);
  }

#if DEBUG_FUNCTION
  if (cusorPos > maxindex)
    Serial.println("SetParaFloat: cusorPos > maxindex");
  Serial.printf("key:%u\n", Key);
#endif

  if (tmpval >= 0) mul = power(10, maxindex - cusorPos);
  else mul = power(10, maxvlen - cusorPos);

  if (Key == KEY_UP || keyUphold) {
    prevNum = tmpval;
    if (tmpval <= (maxval - mul)) tmpval += mul;
    if ((prevNum < 0) && (tmpval >= 0)) cusorPos--;
    funcUpdate = true;
  } else if (Key == KEY_DOWN || keyDownhold) {
    prevNum = tmpval;
    if (tmpval >= (minval + mul)) tmpval -= mul;
    if ((prevNum >= 0) && (tmpval < 0)) cusorPos++;
    funcUpdate = true;
  } else if (Key == KEY_LEFT) {
    if (tmpval >= 0) {
      if (cusorPos > 0) cusorPos--;
    } else {
      if (cusorPos > 1) cusorPos--;
    }
  } else if (Key == KEY_RIGHT) {
    if (tmpval >= 0) {
      if (cusorPos < maxindex) cusorPos++;
    } else {
      if (cusorPos < maxvlen) cusorPos++;
    }
  } else if (Key == KEY_ENTER) {
    *val = tmpval;
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_SAVE;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  if (funcUpdate) {
    // clear the area
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawBox(x - 9, y - 9, 9, 11);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    // draw arrow
    u8g2.setFont(u8g2_font_8x13_t_symbols);
    u8g2.drawGlyph(x - 7, y + 1, 9658);
    u8g2.setFont(u8g2_font_t0_12_mf);  // set font

    char tmp[10];
    if (tmpval >= 0)
      xconvert(tmp, "%%0%dd ", maxvlen);
    else
      xconvert(tmp, "%%0%dd", maxvlen);
    xconvert(gStr1, tmp, tmpval);
    u8g2.drawStr(x, y, gStr1);
    if (tmpval < 0) {
      if (cusorPos == 0) cusorPos = 1;
    }
    u8g2.drawHLine(x + cusorPos * 6, y + 1, 6);
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }
    funcUpdate = false;
  }
}

//----------------------
// getMaxday()
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
// getTime()
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
        Serial.println("getTime(): check date and time error!");
        *dt1 = { 2000, 1, 1, 0, 0, 0 };  //yyyymmdd,hhmmss,set to known value when error
      }
    }
  }

  return ret;
}

//------------------------------------------
// checkTime()
//------------------------------------------
void checkTime() {
  now = rtc.now();  //get time
  if (now.year() == 2000) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    now = rtc.now();  //read adjusted time
    Serial.println("RTC:adjusting date and time.");
    //check date and time value
    getTime(now, &dnt);
  }
}

//------------------------------------------
// COMport1Config() Update SCADA COM port Config
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

//------------------------------------------
// getVolUnit()
//------------------------------------------
void getVolUnit() {
  // L/gal Unit
  if (status.motor.volunit == 0) {
    strcpy(volUnitStr, "L/24hrs");  //Litre
  } else if (status.motor.volunit == 1) {
    strcpy(volUnitStr, "gal/24hrs");  //Gallon
  }
}

//------------------------------------------
// getTempUnit()
//------------------------------------------
void getTempUnit() {
  // deg C / F Unit
  if (status.motor.tempunit == 0) {
    strcpy(tempUnitStr, "deg C");  //deg C
  } else if (status.motor.tempunit == 1) {
    strcpy(tempUnitStr, "F");  //F
  }
}

//------------------------------------------
// updateStrokeRateFromRpm()
//------------------------------------------
void updateStrokeRateFromRpm(u16 rpm) {
  status.motor.cal_stroke_rate_f = (double)rpm / status.motor.gear_ratio_f;
}

//------------------------------------------
// getPistonRadius()
//------------------------------------------
double getPistonRadius(u8 ph) {
  double ret = 0.0;

  // piston radius
  if (status.motor.piston_diameter[ph] == 0) {
    ret = 0.0047625 / 2;  //3/16"
  } else if (status.motor.piston_diameter[ph] == 1) {
    ret = 0.00635 / 2;  //1/4"
  } else if (status.motor.piston_diameter[ph] == 2) {
    ret = 0.009525 / 2;  //3/8"
  } else if (status.motor.piston_diameter[ph] == 3) {
    ret = 0.0127 / 2;  //1/2"
  } else if (status.motor.piston_diameter[ph] == 4) {
    ret = 0.01905 / 2;  //3/4"
  }

  return ret;
}

//------------------------------------------
// updatePumpRatebyStrokeRate()
//------------------------------------------
double updatePumpRatebyStrokeRate(double stroke_rate, u8 ph) {
  double ret;

  double r = getPistonRadius(ph);                             // piston radius
  double w = stroke_rate;                                     // input stroke rate
  double c = status.motor.conversion_factor_f;                // L/gal unit conversion factor
  double l = status.motor.stroke_length_inch_f[ph] * 0.0254;  // stroke length
  double f = status.motor.pump[ph].correction_factor_f;       // correction factor
  double r2_m_f = r * r;

  ret = (PI_VALUE * r2_m_f * l * c * w * 1440.0) / f;  // return the new pump rate
  //Serial.printf("max prate:%f\n", ret);

  return ret;
}

//------------------------------------------
// updateStrokeRatebyGearRatio()
//------------------------------------------
double updateStrokeRatebyGearRatio(double gr) {
  double ret;

  double gear_ratio_f = gr;  // gear ratio

  ret = ctrl.rpm.all / gear_ratio_f;  // return the new stroke rate = current rpm setting / gear ratio

  return ret;
}

//------------------------------------------
// CalMaxPumpRatebyPistonLimit()
//------------------------------------------
double CalMaxPumpRatebyPistonLimit(u8 ph) {
  double ret;

  double r = getPistonRadius(ph);                                 // piston radius
  double wmax = (double)MAXIMUM_RPM / status.motor.gear_ratio_f;  // minimum stroke rate limited by motor speed
  double c = status.motor.conversion_factor_f;                    // L/gal unit conversion factor
  double lmax = 0.0254;                                           // minimum stroke length = 1.0"
  double f = status.motor.pump[ph].correction_factor_f;           // correction factor
  double r2_m_f = r * r;

  ret = (PI_VALUE * r2_m_f * lmax * c * wmax * 1440.0) / f;
  //Serial.printf("max prate:%f\n", ret);

  return ret;
}

//------------------------------------------
// CalMinPumpRatebyPistonLimit()
//------------------------------------------
double CalMinPumpRatebyPistonLimit(u8 ph) {
  double ret;

  double r = getPistonRadius(ph);                                 // piston radius
  double wmin = (double)MINIMUM_RPM / status.motor.gear_ratio_f;  // minimum stroke rate limited by motor speed
  double c = status.motor.conversion_factor_f;                    // L/gal unit conversion factor
  double lmin = 0.00635;                                          // minimum stroke length = 0.25"
  double f = status.motor.pump[ph].correction_factor_f;           // correction factor
  double r2_m_f = r * r;

  ret = (PI_VALUE * r2_m_f * lmin * c * wmin * 1440.0) / f;
  //Serial.printf("min prate:%f\n", ret);

  return ret;
}

double Lnew[NO_OF_PUMPS] = { 0.0254, 0.0254, 0.0254, 0.0254 };  // new stroke length
double Lnew_inch[NO_OF_PUMPS] = { 1.0, 1.0, 1.0, 1.0 };         // new stroke length in inch

//------------------------------------------
// ValidatePumpSetting()
//------------------------------------------
bool ValidatePumpSetting(double v, u8 ph) {
  //double maxprate = 0.0, maxprate_prev = 0.0;  // max. pump rate of all pumps
  //u8 pump_has_maxprate = 0;                    // pump has the max. pump rate
  //double minprate = 0.0, minprate_prev = 0.0;  // min. pump rate of all pumps
  //u8 pump_has_minprate = 0;                    // pump has the min. pump rate
  bool ret = false;

  double r = 0.009525 / 2;  // piston radius
  double l;                 // stroke length
  double f;                 // correction factor
  double w;                 // stroke rate
  double pump_rate = v;     // pump rate set
  double r2_m_f;            // r^2

  //const double lmin = 0.00635;                                // minimum stroke length = 0.25"
  const double lmax = 0.0254;                                   // maximum stroke length = 1.0"
  const double wmin = MINIMUM_RPM / status.motor.gear_ratio_f;  // minimum stroke rate limited by motor speed
  const double wmax = MAXIMUM_RPM / status.motor.gear_ratio_f;  // maximum stroke rate limited by motor speed
  //double wnew = 0.0;                                            // new stroke rate
  double c;  // L/gal unit conversion factor

  /*
  // return if absolute limit is reached
  if (v > CalMaxPumpRatebyPistonLimit(ph)) {
    return false;
  }
  if (v < CalMinPumpRatebyPistonLimit(ph)) {
    return false;
  }*/

  // set user adjust limit
  status.motor.max_pump_rate_f = 999.9;
  status.motor.max_pump_rate = 9999;
  status.motor.min_pump_rate_f = 0.1;
  status.motor.min_pump_rate = 1;

  // multi-head pump
  if (status.motor.total_pump_head > 1) {
    // test this pump
    r = getPistonRadius(ph);
    r2_m_f = r * r;                                 // r2
    f = status.motor.pump[ph].correction_factor_f;  // correction factor
    c = status.motor.conversion_factor_f;           // L/gal unit conversion factor
    updateStrokeRateFromRpm(ctrl.rpm.all);          // stroke per min
    w = status.motor.cal_stroke_rate_f;
    l = pump_rate * f / (PI_VALUE * r2_m_f * c * w * 1440.0);  // calculate stroke length
                                                               // 0.0001 corrects the double precision
                                                               //if (l >= (lmin - 0.0001)) {
    Lnew[ph] = l;                                              // calculate stroke length
    Lnew_inch[ph] = Lnew[ph] / 0.0254;                         // calculate stroke length in Inch
    // test the other 3 pumps
    for (u8 i = 0; i < status.motor.total_pump_head; i++) {
      if (i != ph) {
        pump_rate = status.motor.pump_rate_f[i];                         // pump rate
        w = status.motor.cal_stroke_rate_f;                              // stroke per min
        r = getPistonRadius(i);                                          // piston radius
        r2_m_f = r * r;                                                  // r2
        f = status.motor.pump[i].correction_factor_f;                    // correction factor
        Lnew[i] = pump_rate * f / (PI_VALUE * r2_m_f * c * w * 1440.0);  // calculate new stroke length
                                                                         //if ((Lnew[i] >= lmin)) {
        Lnew_inch[i] = Lnew[i] / 0.0254;                                 // calculate stroke length in inch
      }
    }
    return true;  // let operator check
    //
    // TEST_MAX_STROKE_LENGTH
    //
    /*case TEST_MAX_STROKE_LENGTH:
#if DEBUG_PUMP_SETTING
          Serial.println("TEST_MAX_STROKE_LENGTH");
#endif
          // find the ph having the max pump rate set
          for (u8 i = 0; i < status.motor.total_pump_head; i++) {
            maxprate_prev = maxprate;
            maxprate = findmax(maxprate, status.motor.pump_rate_f[i]);
            if (maxprate > maxprate_prev) {
              pump_has_maxprate = i;
            }
          }
#if DEBUG_PUMP_SETTING
          Serial.printf("pump_has_maxprate:%u\n", pump_has_maxprate);
#endif
          pump_rate = maxprate;
          // run the maximum stroke length test
          l = lmax;                                                         // stroke length
          Lnew[pump_has_maxprate] = l;                                      // store stroke length for next checking
          Lnew_inch[pump_has_maxprate] = Lnew[pump_has_maxprate] / 0.0254;  // store calculate stroke length in inch for next checking
          r = getPistonRadius(ph);                                          // piston radius
          r2_m_f = r * r;                                                   // r2
          f = status.motor.pump[ph].correction_factor_f;                    // correction factor
          wnew = pump_rate * f / (PI_VALUE * r2_m_f * c * l * 1440.0);      // calculate new stroke rate
#if DEBUG_PUMP_SETTING
          Serial.printf("w@lmax:%f\n", wnew);
#endif
          if ((wnew <= wmax) && (wnew >= wmin)) {
            nextTrial = CHANGE_STROKE_LENGTH_OF_OTHER_PUMPS;
#if DEBUG_PUMP_SETTING
            Serial.println("CONTINUE");
#endif
            break;
          } else if (wnew > wmax) {
            pseterr[ph].bit.stroke_rate_high_err = 1;  // record the error
          } else if (wnew < wmin) {
            pseterr[ph].bit.stroke_rate_low_err = 1;  // record the error
          }
          nextTrial = TEST_MIN_STROKE_LENGTH;
          break;
          //
          // TEST_MIN_STROKE_LENGTH
          //
        case TEST_MIN_STROKE_LENGTH:
#if DEBUG_PUMP_SETTING
          Serial.println("TEST_MIN_STROKE_LENGTH");
#endif
          // find the ph having the min pump rate set
          for (u8 i = 0; i < status.motor.total_pump_head; i++) {
            minprate_prev = minprate;
            minprate = findmin(minprate, status.motor.pump_rate_f[i]);
            if (minprate < minprate_prev) {
              pump_has_minprate = i;
            }
          }
#if DEBUG_PUMP_SETTING
          Serial.printf("pump_has_minprate:%u\n", pump_has_minprate);
#endif
          pump_rate = minprate;
          // run the minimum stroke length test
          l = lmin;                                                         // stroke length
          Lnew[pump_has_minprate] = l;                                      // store stroke length for next checking
          Lnew_inch[pump_has_minprate] = Lnew[pump_has_minprate] / 0.0254;  // store calculate stroke length in inch for next checking
          r = getPistonRadius(ph);                                          // piston radius
          r2_m_f = r * r;                                                   // r2
          f = status.motor.pump[ph].correction_factor_f;                    // correction factor
          wnew = pump_rate * f / (PI_VALUE * r2_m_f * c * l * 1440.0);      // calculate new stroke rate
#if DEBUG_PUMP_SETTING
          Serial.printf("w@lmin:%f\n", wnew);
#endif
          if ((wnew <= wmax) && (wnew >= wmin)) {
            nextTrial = CHANGE_STROKE_LENGTH_OF_OTHER_PUMPS;
#if DEBUG_PUMP_SETTING
            Serial.println("CONTINUE");
#endif
            break;
          } else if (wnew > wmax) {
            pseterr[ph].bit.stroke_rate_high_err = 1;  //record the error
          } else if (wnew < wmin) {
            pseterr[ph].bit.stroke_rate_low_err = 1;  //record the error
          }
          nextTrial = CHANGE_STROKE_LENGTH;
          return false;*/
    // Apply new setting
    /*
          ctrl.rpm.all = wnew * status.motor.gear_ratio_f;
          for (u8 i = 0; i < status.motor.total_pump_head; i++) {
            status.motor.stroke_length_inch_f[i] = Lnew_inch[i];  // calculate stroke length in inch
            status.motor.pump[i].stroke_length.all = (u16)(status.motor.stroke_length_inch_f[ph] * 100.0);
#if DEBUG_PUMP_SETTING
            Serial.printf("lnew_inch[%u]:%f\n", i, Lnew_inch[i]);
#endif
          }
          updateStrokeRateFromRpm(ctrl.rpm.all);  // update stroke per min
#if DEBUG_PUMP_SETTING
          Serial.println("SUCCESS & return");
#endif*/
  } else {  // 1 pump head only
    // piston radius
    if (status.motor.piston_diameter[ph] == 0) {
      r = 0.0047625 / 2;  //3/16"
    } else if (status.motor.piston_diameter[ph] == 1) {
      r = 0.00635 / 2;  //1/4"
    } else if (status.motor.piston_diameter[ph] == 2) {
      r = 0.009525 / 2;  //3/8"
    } else if (status.motor.piston_diameter[ph] == 3) {
      r = 0.0127 / 2;  //1/2"
    } else if (status.motor.piston_diameter[ph] == 4) {
      r = 0.01905 / 2;  //3/4"
    }

    r2_m_f = r * r;                                 // r2
    f = status.motor.pump[ph].correction_factor_f;  // correction factor
    c = status.motor.conversion_factor_f;           // L/gal unit conversion factor
    l = lmax;                                       // stroke length

    w = pump_rate * f / (PI_VALUE * r2_m_f * c * l * 1440.0);  // calculate stroke per min MAXIMUM_RPM / status.motor.gear_ratio_f;

    if ((w <= wmax) && (w >= wmin)) {
      // update motor speed
      status.motor.cal_stroke_rate_f = w;
      status.motor.stroke_rate.all = (u16)(status.motor.cal_stroke_rate_f * 10.0);
      ctrl.rpm.all = (u16)(w * status.motor.gear_ratio_f);
      return true;
    }
  }

  return ret;
}

//------------------------------------------------------------------
// updateStrokeCount()
//------------------------------------------------------------------
void updateStrokeCount(u8 pump_head) {
  //update day & accumulated stroke count
  if ((ctrl.onoff == ON) && (status.onoff == ON) && (status.rpm.all > 0)) {
    //maximum stroke per second ~= 1.0
    status.motor.pump_record.stroke_count_f += (status.motor.cal_stroke_rate_f / 60.0);
    for (double i = 1.0; i < 3.0; i = i + 1.0) {
      if (status.motor.pump_record.stroke_count_f > i) {
        status.motor.pump_record.hour_stroke_count.all += (u32)status.motor.pump_record.stroke_count_f;  //auto-clear every hour
        status.motor.pump_record.day_stroke_count.all += (u32)status.motor.pump_record.stroke_count_f;   //auto-clear every day

        if (status.motor.pump_record.record_index < 2) {
          status.motor.pump_record.month_stroke_count[status.motor.pump_record.record_index * 12 + dnt.month - 1].all += (u32)status.motor.pump_record.stroke_count_f;  //auto-clear every month
        } else {
          //some errors, we should not be here
          strcpy(userMsg[0], "Record Index");
          strcpy(userMsg[1], "Invalid!");
          status.motor.pump_record.record_index = 0;
        }
        status.motor.pump_record.accumulated_stroke_count.all += (u32)status.motor.pump_record.stroke_count_f;
        status.motor.pump_record.stroke_count_f -= i;
      }
    }
  }
}

//------------------------------------------------------------------
// drawMenu()
//------------------------------------------------------------------
void drawMenu(MENU_CTRL_T *pctrl) {
  char title[MENU_STRING_BUFSIZE] = { 0 };
  // temp pointer
  MENU_T *p;
  u8 ypos = pctrl->parent_text_ypos + pctrl->item_offset;

  u8g2.clearBuffer();                // clear lcd ram buffer
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.setFontMode(0);               // no transparency

  // update parent menu text & assign to display buffer
  u8g2.setDrawColor(TEXT_COLOR_INVERTED);

  // show reminder
  /*if (sysstat.bit.reboot_pending == false) {
    u8g2.drawStr(0, pctrl->parent_text_ypos, "REBOOT PENDING");
  }*/

  if (pActiveMenu == menu) {
    switch (pctrl->parent->layer) {
      case MENU_LAYER_0:
        xconvert(title, "%s", "PUMP RATES");
        break;
      case MENU_LAYER_1:
        switch (pctrl->parent->id) {
          case 11:
            if (status.motor.total_pump_head > 1) {
              xconvert(title, "%s", "PUMP #1");
            } else {
              xconvert(title, "%s", "PUMP");
            }
            status.motor.pump_number = 0;
            break;
          case 12:
            xconvert(title, "%s", "PUMP #2");
            status.motor.pump_number = 1;
            break;
          case 13:
            xconvert(title, "%s", "PUMP #3");
            status.motor.pump_number = 2;
            break;
          case 14:
            xconvert(title, "%s", "PUMP #4");
            status.motor.pump_number = 3;
            break;
        }
        break;
      case MENU_LAYER_2:
        switch (pctrl->parent->id) {
          case 113:
            if (status.motor.total_pump_head > 1) {
              xconvert(title, "%s", "PUMP #1 CALIBRATION");
            } else {
              xconvert(title, "%s", "PUMP CALIBRATION");
            }
            status.motor.pump_number = 0;
            break;
          case 123:
            xconvert(title, "%s", "PUMP #2 CALIBRATION");
            status.motor.pump_number = 1;
            break;
          case 133:
            xconvert(title, "%s", "PUMP #3 CALIBRATION");
            status.motor.pump_number = 2;
            break;
          case 143:
            xconvert(title, "%s", "PUMP #4 CALIBRATION");
            status.motor.pump_number = 3;
            break;
        }
        break;
      case MENU_LAYER_3:
        break;
      case MENU_LAYER_MAX:
        break;
    }
  } else if (pActiveMenu == menu_setting) {
    switch (pctrl->parent->layer) {
      case MENU_LAYER_0:
        xconvert(title, "%s", "SYSTEM & SETTINGS");
        break;
      case MENU_LAYER_1:
        switch (pctrl->parent->id) {
          case 11:
            xconvert(title, "%s", "CARBON CREDIT SUMMARY");
            break;
          case 12:
            xconvert(title, "%s", "SYSTEM SETTINGS");
            break;
          case 13:
            xconvert(title, "%s", "TEMPERATURE ON/OFF SETTINGS");
            break;
          case 14:
            xconvert(title, "%s", "SCADA SETTINGS");
            break;
          case 15:
            xconvert(title, "%s", "FAULT HISTORY");
            break;
          case 16:
            xconvert(title, "%s", "SYSTEM TIME SETTING");
            break;
          case 17:
            xconvert(title, "%s", "SYSTEM INFORMATION");
            break;
        }
        break;
      case MENU_LAYER_2:
        switch (pctrl->parent->id) {
          case 141:
            xconvert(title, "%s", "SCADA MODBUS SETTINGS");
            break;
          case 142:
            xconvert(title, "%s", "SCADA SERIAL PORT SETTINGS");
            break;
        }
        break;
      case MENU_LAYER_3:
        break;
      case MENU_LAYER_MAX:
        break;
    }
  }
  u8g2.drawStr(pctrl->text_xpos, pctrl->parent_text_ypos, title);
  u8g2.setDrawColor(TEXT_COLOR_NORMAL);

  // display child text & assign to display buffer
  if (pctrl->menu_offset >= pctrl->lcd_max_items) {
    p = pctrl->selected - pctrl->lcd_max_items + 1;
    // temporarily solve the problem of the item in the middle for counting the p position
    if ((pActiveMenu == menu) && ((pctrl->selected->id == 115) || (pctrl->selected->id == 125) || (pctrl->selected->id == 135) || (pctrl->selected->id == 145) || (pctrl->selected->id == 116) || (pctrl->selected->id == 126) || (pctrl->selected->id == 136) || (pctrl->selected->id == 146))) {
      p--;
    }
  } else {
    // point to parent
    p = pctrl->parent + 1;
  }
  while (1) {
    if ((p->layer == (pctrl->parent->layer + 1)) && (p->state == true)) {
      // update menu text
      // mtext
      if (pActiveMenu == menu) {
        switch (p->id) {
          case 11:
            xconvert(p->ptext, "PH #1                  %05.1f %s", status.motor.pump_rate_f[0], volUnitStr);
            break;
          case 111:
            xconvert(p->ptext, "Pump Rate              %05.1f %s", status.motor.pump_rate_f[0], volUnitStr);
            break;
          case 112:
            xconvert(p->ptext, "Stroke Rate            %.1f/min", status.motor.cal_stroke_rate_f);
            break;
          case 113:
            if (pctrl->selected->id == 113) {
              xconvert(p->ptext, "Calibrated             CALIBRATE NOW");
            } else if (status.motor.pump[0].calibration_dnt.year == 2000) {
              xconvert(p->ptext, "Calibrated             --/--/-- --:--:--");
            } else {
              xconvert(p->ptext, "Calibrated             %d/%02d/%02d %02d:%02d:%02d",
                       status.motor.pump[0].calibration_dnt.year, status.motor.pump[0].calibration_dnt.month, status.motor.pump[0].calibration_dnt.day,
                       status.motor.pump[0].calibration_dnt.hour, status.motor.pump[0].calibration_dnt.minute, status.motor.pump[0].calibration_dnt.second);
            }
            break;
          case 1131:
            xconvert(p->ptext, "Set pumped volume observed   %05.1f %s", status.motor.actual_rate_f[0], volUnitStr);
            break;
          case 114:
            xconvert(p->ptext, "Piston Size            %s", Piston_arr[status.motor.piston_diameter[0]]);
            break;
          case 115:
            //if (status.motor.total_pump_head > 1)
            xconvert(p->ptext, "Stroke Length          %.2f\"", status.motor.stroke_length_inch_f[0]);
            //else
            //xconvert(p->ptext, "Stroke Length          %s", Stroke_arr[status.motor.stroke_length[0]]);
            break;
          case 116:
            xconvert(p->ptext, "Gear Ratio             %.1f [Preset]", status.motor.gear_ratio_f);
            break;
          case 12:
            xconvert(p->ptext, "PH #2                  %05.1f %s", status.motor.pump_rate_f[1], volUnitStr);
            break;
          case 121:
            xconvert(p->ptext, "Pump Rate              %05.1f %s", status.motor.pump_rate_f[1], volUnitStr);
            break;
          case 122:
            xconvert(p->ptext, "Stroke Rate            %.1f/min", status.motor.cal_stroke_rate_f);
            break;
          case 123:
            if (pctrl->selected->id == 123) {
              xconvert(p->ptext, "Calibrated             CALIBRATE NOW");
            } else if (status.motor.pump[1].calibration_dnt.year == 2000) {
              xconvert(p->ptext, "Calibrated             --/--/-- --:--:--");
            } else {
              xconvert(p->ptext, "Calibrated             %d/%02d/%02d %02d:%02d:%02d",
                       status.motor.pump[1].calibration_dnt.year, status.motor.pump[1].calibration_dnt.month, status.motor.pump[1].calibration_dnt.day,
                       status.motor.pump[1].calibration_dnt.hour, status.motor.pump[1].calibration_dnt.minute, status.motor.pump[1].calibration_dnt.second);
            }
            break;
          case 1231:
            xconvert(p->ptext, "Set pumped volume observed   %05.1f %s", status.motor.actual_rate_f[1], volUnitStr);
            break;
          case 124:
            xconvert(p->ptext, "Piston Size            %s", Piston_arr[status.motor.piston_diameter[1]]);
            break;
          case 125:
            xconvert(p->ptext, "Stroke Length          %.2f\"", status.motor.stroke_length_inch_f[1]);
            break;
          case 126:
            xconvert(p->ptext, "Gear Ratio             %.1f [Preset]", status.motor.gear_ratio_f);
            break;
          case 13:
            xconvert(p->ptext, "PH #3                  %05.1f %s", status.motor.pump_rate_f[2], volUnitStr);
            break;
          case 131:
            xconvert(p->ptext, "Pump Rate              %05.1f %s", status.motor.pump_rate_f[2], volUnitStr);
            break;
          case 132:
            xconvert(p->ptext, "Stroke Rate            %.1f/min", status.motor.cal_stroke_rate_f);
            break;
          case 133:
            if (pctrl->selected->id == 133) {
              xconvert(p->ptext, "Calibrated             CALIBRATE NOW");
            } else if (status.motor.pump[2].calibration_dnt.year == 2000) {
              xconvert(p->ptext, "Calibrated             --/--/-- --:--:--");
            } else {
              xconvert(p->ptext, "Calibrated             %d/%02d/%02d %02d:%02d:%02d",
                       status.motor.pump[2].calibration_dnt.year, status.motor.pump[2].calibration_dnt.month, status.motor.pump[2].calibration_dnt.day,
                       status.motor.pump[2].calibration_dnt.hour, status.motor.pump[2].calibration_dnt.minute, status.motor.pump[2].calibration_dnt.second);
            }
            break;
          case 1331:
            xconvert(p->ptext, "Set pumped volume observed   %05.1f %s", status.motor.actual_rate_f[2], volUnitStr);
            break;
          case 134:
            xconvert(p->ptext, "Piston Size            %s", Piston_arr[status.motor.piston_diameter[2]]);
            break;
          case 135:
            xconvert(p->ptext, "Stroke Length          %.2f\"", status.motor.stroke_length_inch_f[2]);
            break;
          case 136:
            xconvert(p->ptext, "Gear Ratio             %.1f [Preset]", status.motor.gear_ratio_f);
            break;
          case 14:
            xconvert(p->ptext, "PH #4                  %05.1f %s", status.motor.pump_rate_f[3], volUnitStr);
            break;
          case 141:
            xconvert(p->ptext, "Pump Rate              %05.1f %s", status.motor.pump_rate_f[3], volUnitStr);
            break;
          case 142:
            xconvert(p->ptext, "Stroke Rate            %.1f/min", status.motor.cal_stroke_rate_f);
            break;
          case 143:
            if (pctrl->selected->id == 143) {
              xconvert(p->ptext, "Calibrated             CALIBRATE NOW");
            } else if (status.motor.pump[3].calibration_dnt.year == 2000) {
              xconvert(p->ptext, "Calibrated             --/--/-- --:--:--");
            } else {
              xconvert(p->ptext, "Calibrated             %d/%02d/%02d %02d:%02d:%02d",
                       status.motor.pump[3].calibration_dnt.year, status.motor.pump[3].calibration_dnt.month, status.motor.pump[3].calibration_dnt.day,
                       status.motor.pump[3].calibration_dnt.hour, status.motor.pump[3].calibration_dnt.minute, status.motor.pump[3].calibration_dnt.second);
            }
            break;
          case 1431:
            xconvert(p->ptext, "Set pumped volume observed   %05.1f %s", status.motor.actual_rate_f[3], volUnitStr);
            break;
          case 144:
            xconvert(p->ptext, "Piston Size            %s", Piston_arr[status.motor.piston_diameter[3]]);
            break;
          case 145:
            xconvert(p->ptext, "Stroke Length          %.2f\"", status.motor.stroke_length_inch_f[3]);
            break;
          case 146:
            xconvert(p->ptext, "Gear Ratio             %.1f [Preset]", status.motor.gear_ratio_f);
            break;
        }
      }
      // update menu_setting text
      else if (pActiveMenu == menu_setting) {
        switch (p->id) {
          case 11:
            xconvert(p->ptext, "%s", "1. Carbon Credit Summary");
            break;
          case 12:
            xconvert(p->ptext, "%s", "2. System Settings");
            break;
          case 121:
            xconvert(p->ptext, "1. # of Pump Heads     %d", status.motor.total_pump_head);
            break;
          case 122:
            xconvert(p->ptext, "2. Min. Input Voltage  %04.1fV", ctrl.min_dcv_input_f);
            break;
          case 123:
            xconvert(p->ptext, "3. Slow Down Voltage   %04.1fV", ctrl.slow_down_dcv_f);
            break;
          case 124:
            xconvert(p->ptext, "4. OCP Amps            %04.1fA", ctrl.ocp_dcamp_f);
            break;
          case 125:
            xconvert(p->ptext, "5. Max Input Voltage   %04.1fV [Preset]", ctrl.max_dcv_input_f);
            break;
          case 126:
            xconvert(p->ptext, "6. Run Time Reset      %s", runtimeReset_arr[status.motor.reset_runtime]);
            break;
          case 127:
            xconvert(p->ptext, "7. Digital IO Mode     %s", enabledDisabled_arr[status.motor.dio_mode]);
            break;
          case 128:
            xconvert(p->ptext, "8. Com. Fail Action    %s", comfailAction_arr[ctrl.communication_fail_action]);
            break;
          case 129:
            xconvert(p->ptext, "9. Volume Unit         %s", volUnit_arr[status.motor.volunit]);
            break;
          case 1210:
            xconvert(p->ptext, "10. Temperature Unit   %s", tempUnit_arr[status.motor.tempunit]);
            break;
          case 1211:
            xconvert(p->ptext, "11. Gear Ratio         %04.1f", status.motor.gear_ratio_f);
            break;
          case 1212:
            xconvert(p->ptext, "12. Board Series       %s", brdSeries_arr[ctrl.motor_model.byte[1]]);
            break;
          case 1213:
            xconvert(p->ptext, "13. Motor Model        %s", mtrModel_arr[ctrl.motor_model.byte[0]]);
            break;
          case 13:
            xconvert(p->ptext, "%s", "3. Temperature On/Off Settings");
            break;
          case 131:
            xconvert(p->ptext, "1. Temp. On/Off           %s", enabledDisabled_arr[status.motor.temp_ctrl]);
            break;
          case 132:
            status.motor.off_temp.all = twos2Int(status.motor.off_temp.byte[1], status.motor.off_temp.byte[0]);
            xconvert(p->ptext, "2. Off Temp.              %d %s", status.motor.off_temp.all, tempUnitStr);
            break;
          case 133:
            status.motor.on_temp.all = twos2Int(status.motor.on_temp.byte[1], status.motor.on_temp.byte[0]);
            xconvert(p->ptext, "3. On Temp.               %d %s", status.motor.on_temp.all, tempUnitStr);
            break;
          case 134:
            status.motor.temp_now.all = twos2Int(status.motor.temp_now.byte[1], status.motor.temp_now.byte[0]);
            xconvert(p->ptext, "    Current Temp Reading: %d %s", status.motor.temp_now.all, tempUnitStr);
            break;
          case 14:
            xconvert(p->ptext, "%s", "4. SCADA Settings");
            break;
          case 141:
            xconvert(p->ptext, "%s", "1. SCADA MODBUS Settings");
            break;
          case 1411:
            xconvert(p->ptext, "1. Protocol              %s [RTU/ASCII]", Protocol_arr[scada_setting.protocol]);
            break;
          case 1412:
            xconvert(p->ptext, "2. Server ID             %03d   [1-247]", scada_setting.server_id);
            break;
          case 1413:
            xconvert(p->ptext, "3. Timeout(ASCII)        %05d [100-65535]", scada_setting.timeout);
            break;
          case 1414:
            xconvert(p->ptext, "4. Output Coil Addr      %05d [0-%05d]", scada_setting.coil_addr, 65535 - scada_setting.max_coil);
            break;
          case 1415:
            xconvert(p->ptext, "5. Disc. Input Addr      %05d [0-%05d]", scada_setting.discrete_addr, 65535 - scada_setting.max_discrete);
            break;
          case 1416:
            xconvert(p->ptext, "6. Input Reg. Addr       %05d [0-%05d]", scada_setting.inputreg_addr, 65535 - scada_setting.max_inputreg);
            break;
          case 1417:
            xconvert(p->ptext, "7. Hold Reg. Addr        %05d [0-%05d]", scada_setting.holdreg_addr, 65535 - scada_setting.max_holdreg);
            break;
          case 142:
            xconvert(p->ptext, "%s", "2. SCADA Serial Port Settings");
            break;
          case 1421:
            xconvert(p->ptext, "1. Baud Rate             %06d", comport.baud.all);
            break;
          case 1422:
            xconvert(p->ptext, "2. Data Bits             %d", comport.databits);
            break;
          case 1423:
            xconvert(p->ptext, "3. Stop Bits             %d", comport.stopbits);
            break;
          case 1424:
            xconvert(p->ptext, "4. Parity                %s", Parity_arr[comport.parity]);
            break;
          case 15:
            xconvert(p->ptext, "%s", "5. Fault History");
            break;
          case 16:
            xconvert(p->ptext, "%s", "6. System Time Setting");
            break;
          case 17:
            xconvert(p->ptext, "%s", "7. System Information");
            break;
        }
      }
      // Draw to LCD buffer
      if (p >= pctrl->display) {
        if (p == pctrl->selected) {
          // draw text with inverted color
          u8g2.setDrawColor(TEXT_COLOR_INVERTED);
          u8g2.drawStr(pctrl->text_xpos, ypos, p->ptext);
          u8g2.setDrawColor(TEXT_COLOR_NORMAL);
        } else {
          // draw text with normal color
          u8g2.setDrawColor(TEXT_COLOR_NORMAL);
          u8g2.drawStr(pctrl->text_xpos, ypos, p->ptext);
        }
        if (ypos <= pctrl->lcd_height - 1 - pctrl->item_offset) {
          ypos += pctrl->item_offset;
        } else {
          break;
        }
      }
    } else if (p->layer == pctrl->parent->layer) {
      break;
    } else if (p->layer == MENU_LAYER_0) {
      break;
    }
    p++;
  }
  LcdUpdate();
}

//------------------------------------------------------------------
// findParentMenu()
//------------------------------------------------------------------
MENU_T *findParentMenu(MENU_T *pmenu) {
  MENU_T *p = pmenu;

  if (p->layer == MENU_LAYER_0)
    return NULL;

  while (1) {
    p--;
    if ((p->layer == ((pmenu->layer) - 1)) && (p->state == true)) {
      return p;
    }
  }
}

//------------------------------------------------------------------
// findNextMenu()
//------------------------------------------------------------------
MENU_T *findNextMenu(MENU_T *pmenu, MENU_CTRL_T *pctrl, u8 dir) {
  // temp pointer
  MENU_T *p = pmenu;

  while (1) {
    if (dir == 1)  // up
      p++;
    else  // down
      p--;

    if ((p->layer == ((pctrl->parent->layer) + 1)) && (p->state == true)) {
      return p;
    } else if (p->layer == (pctrl->parent->layer)) {
      return NULL;
    } else if (p->layer == MENU_LAYER_0) {
      return NULL;
    }
  }
}

//------------------------------------------------------------------
// MenuGetMaxValidItemsInLayer()
//------------------------------------------------------------------
void MenuGetMaxValidItemsInLayer(MENU_CTRL_T *pctrl) {
  MENU_T *p = pctrl->parent;

  while (1) {
    if (p->type == MENU_TYPE_NULL) {
      break;
    } else {
      if (p->state == true) {
        pctrl->max_items[p->layer]++;
      }
    }
    p++;
  }
}

//------------------------------------------------------------------
// MenuSetItemState()
//------------------------------------------------------------------
void MenuSetItemState(MENU_CTRL_T *pctrl, u32 id, bool state) {
  MENU_T *p = pctrl->parent;

  while (1) {
    if (p->type == MENU_TYPE_NULL) {
      break;
    } else {
      if (p->id == id) {
        p->state = state;
        break;
      }
    }
    p++;
  }
}

//------------------------------------------------------------------
// menuInit()
//------------------------------------------------------------------
void menuInit(MENU_T *pmenu, MENU_CTRL_T *pctrl) {
  if (pctrl->initialized == true) return;

  // Initialize menu control
  pctrl->parent = pmenu;
  pctrl->display = pmenu + 1;
  pctrl->selected = pctrl->display;
  pctrl->lcd_height = MAX_LCD_HEIGHT;
  pctrl->lcd_width = MAX_LCD_WIDTH;
  pctrl->font_height = 11;
  pctrl->font_width = 6;
  pctrl->menu_offset = 0;
  pctrl->parent_text_ypos = 10;
  pctrl->text_xpos = 0;
  pctrl->text_ypos = 0;
  pctrl->item_offset = 13;
  pctrl->initialized = true;
  pctrl->enable = true;
  pctrl->update = true;
  pctrl->runfunc = false;
  for (u8 i = 0; i < MENU_LAYER_MAX; i++) {
    pctrl->max_items[i] = 0;
  }
  pctrl->lcd_max_items = (pctrl->lcd_height - pctrl->parent_text_ypos) / pctrl->item_offset;

  // set unused pump head menu to inactive
  if (pActiveMenu != menu_setting) {
    switch (status.motor.total_pump_head) {
      case 1:
        // skip the first menu if only one pump head
        if (pActiveMenu == menu) {
          pctrl->parent = pActiveMenu + 1;
          pctrl->display = pActiveMenu + 2;
          pctrl->selected = pctrl->display;
        }
        MenuSetItemState(pctrl, 11, true);
        MenuSetItemState(pctrl, 12, false);
        MenuSetItemState(pctrl, 13, false);
        MenuSetItemState(pctrl, 14, false);
        break;
      case 2:
        MenuSetItemState(pctrl, 11, true);
        MenuSetItemState(pctrl, 12, true);
        MenuSetItemState(pctrl, 13, false);
        MenuSetItemState(pctrl, 14, false);
        break;
      case 3:
        MenuSetItemState(pctrl, 11, true);
        MenuSetItemState(pctrl, 12, true);
        MenuSetItemState(pctrl, 13, true);
        MenuSetItemState(pctrl, 14, false);
        break;
      case 4:
        MenuSetItemState(pctrl, 11, true);
        MenuSetItemState(pctrl, 12, true);
        MenuSetItemState(pctrl, 13, true);
        MenuSetItemState(pctrl, 14, true);
        break;
    }
  }

  // get maximum items in each layer
  MenuGetMaxValidItemsInLayer(pctrl);
}

//------------------------------------------------------------------
// menuScan()
//------------------------------------------------------------------
void menuScan(MENU_T *pmenu, MENU_CTRL_T *pctrl, char key) {
  static u32 menu_updatecnt = millis();

  if (pctrl->enable == false) return;

  if (pctrl->runfunc == true) {
    pctrl->selected->pfunc();
    return;
  }

  // do not update when screensaver is on
  if (protectScreen == true) {
    return;
  } else if (protectScreenPrev == true) {
    protectScreenPrev = false;
    return;
  }

  // update screen at 1s interval
  if (millis() - menu_updatecnt >= 1000) {
    menu_updatecnt = millis();
    pctrl->update = true;  // redraw menu
  }

  if (userMsg[1][0] != '\0') {
    // Update menu
    if (protectScreen == false) {
      // when function is not running, display user messages/errors
      if (menuctrl.runfunc == false) {
        u8g2.clearBuffer();                // clear lcd ram buffer
        u8g2.setFont(u8g2_font_t0_14_mf);  // set font
        u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[0]) * 7) / 2, 22, userMsg[0]);
        u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[1]) * 7) / 2, 38, userMsg[1]);
        u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[2]) * 7) / 2, 54, userMsg[2]);
        LcdUpdate();  // Send to display & use size 12 font
        vTaskDelay(1500 / portTICK_PERIOD_MS);
        pctrl->update = true;  // redraw menu
      }
    }
    userMsg[0][0] = '\0';
    userMsg[1][0] = '\0';
    userMsg[2][0] = '\0';
  }

  // redraw menu
  if (pctrl->update == true) {
    drawMenu(pctrl);
    pctrl->update = false;
  }

  // temp pointer
  MENU_T *p;

  if (key) {
    switch (key) {
      case KEY_ESC:
        // go to home page
        if (pActiveMenu == menu_setting) {
          menuctrl.initialized = false;
          pActiveMenu = menu;
          menuInit(pActiveMenu, &menuctrl);
          return;
        }
        // Do not allow ESC to previous layer when only one pump head
        if ((status.motor.total_pump_head == 1) && (pctrl->display->layer == MENU_LAYER_2) && (pActiveMenu != menu_setting)) {
          break;
        }
        p = findParentMenu(pctrl->parent);
        if (p != NULL) {
          pctrl->parent = p;
          pctrl->selected = menulast.history[pctrl->parent->layer * 2];
          pctrl->display = menulast.history[pctrl->parent->layer * 2 + 1];
          pctrl->menu_offset = menulast.menu_offset[pctrl->parent->layer];
        }
        break;
      case KEY_UP:
        if (pctrl->selected->layer == MENU_LAYER_1) {
          menuctrl.initialized = false;
          pActiveMenu = menu;
          menuInit(pActiveMenu, &menuctrl);
          return;
        }
        break;
      case KEY_DOWN:
        if ((pctrl->selected->layer == MENU_LAYER_1) || (status.motor.total_pump_head == 1)) {
          menuctrl.initialized = false;
          pActiveMenu = menu_setting;
          menuInit(pActiveMenu, &menuctrl);
          return;
        }
        break;
      case KEY_LEFT:
        p = findNextMenu(pctrl->selected, pctrl, 0);
        if (p != NULL) {
          pctrl->selected = p;
          if (pctrl->menu_offset == 0) {
            pctrl->display = pctrl->selected;
          } else {
            pctrl->menu_offset--;
          }
        } else {
          while (1) {
            p = findNextMenu(pctrl->selected, pctrl, 1);
            if (p != NULL) {
              pctrl->selected = p;
              if (pctrl->menu_offset > pctrl->max_items[pctrl->selected->layer] - 2) {
                p = findNextMenu(pctrl->display, pctrl, 1);
                if (p != NULL) {
                  pctrl->display = p;
                }
              } else {
                pctrl->menu_offset++;
              }
            } else {
              break;
            }
          }
        }
        break;
      case KEY_RIGHT:
        p = findNextMenu(pctrl->selected, pctrl, 1);
        if (p != NULL) {
          pctrl->selected = p;
          if (pctrl->menu_offset > pctrl->max_items[pctrl->selected->layer] - 2) {
            p = findNextMenu(pctrl->display, pctrl, 1);
            if (p != NULL) {
              pctrl->display = p;
            }
          } else {
            pctrl->menu_offset++;
          }
        } else {
          pctrl->selected = pctrl->parent + 1;
          pctrl->display = pctrl->selected;
          pctrl->menu_offset = 0;
        }
        break;
      case KEY_ENTER:

        if (status.motor.total_pump_head == 1) {
          // 112 we don't want to change stroke rate for 1 pump
          // 115 we don't want to change stroke legnth for 1 pump
          if ((pctrl->selected->id == 112) || (pctrl->selected->id == 115)) {
            // warn user
            strcpy(userMsg[1], "NOT AVAILABLE!");
            break;
          }
        }
        // if next menu has something, advance
        if (pctrl->selected->layer == (pctrl->parent->layer + 1)) {
          // it's a menu function
          if (pctrl->selected->type == MENU_TYPE_FUNC) {
            if (pctrl->selected->pfunc != NULL) {
              // update function running state
              pctrl->runfunc = true;
              // calculate the correct ypos to print
              if (pctrl->menu_offset < pctrl->lcd_max_items) {
                pctrl->text_ypos = pctrl->parent_text_ypos + ((pctrl->menu_offset % pctrl->lcd_max_items) + 1) * pctrl->item_offset;
              } else {
                pctrl->text_ypos = pctrl->parent_text_ypos + pctrl->lcd_max_items * pctrl->item_offset;
              }
              // execute function
              pctrl->selected->pfunc();
            }
          }
          // it's a menu list
          else if (pctrl->selected->type != MENU_TYPE_TEXT) {
            // record history so that we know where the previous menu is
            menulast.history[pctrl->parent->layer * 2] = pctrl->selected;
            menulast.history[pctrl->parent->layer * 2 + 1] = pctrl->display;
            menulast.menu_offset[pctrl->parent->layer] = pctrl->menu_offset;
            pctrl->parent = pctrl->selected;
            pctrl->selected = pctrl->parent + 1;
            pctrl->display = pctrl->selected;
            pctrl->menu_offset = 0;
          }
          // it's a text only
          else {
            // warn user
            strcpy(userMsg[1], "NOT AVAILABLE!");
          }
        }
        break;
    }
    // update display update flag
    pctrl->update = true;
    Key = NO_KEY;  // prevent false trigger of menu
  }

  if (kpState) {
    pctrl->update = true;
  }
}

//---------------------------------------------------------
// MenuDrawArrowAtArray
//---------------------------------------------------------
void MenuDrawArrowAtArray(const u8 array[10][2], u8 maxitem, u8 sel_item) {
  for (u8 i = 0; i < maxitem; i++) {
    if (i == sel_item) {
      // draw arrow
      u8g2.setFont(u8g2_font_8x13_t_symbols);
      u8g2.drawGlyph(array[i][0], array[i][1], 9658);
      u8g2.setFont(u8g2_font_t0_12_mf);  // set font
    } else u8g2.drawStr(array[i][0], array[i][1], " ");
  }
}

//------------------------------------------
// MenuMoveItemIndex
// function: set item no pointed by item arrow
//------------------------------------------
void MenuMoveItemIndex(u8 *p, u8 maxitem) {

  if (Key == KEY_LEFT) {
    if (*p > 0) *p -= 1;
  } else if (Key == KEY_RIGHT) {
    if (*p < maxitem - 1) *p += 1;
  } else if (Key == KEY_UP || Key == KEY_DOWN)
    *p = 0;
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

//------------------------------------------------------------------
// MenuFuncSetGearRatio()
//------------------------------------------------------------------
void MenuFuncSetGearRatio() {
  SetParaFloat(138, menuctrl.text_ypos, &status.motor.gear_ratio.all, 3, 1, 75, 150);  //gear ratio
  status.motor.gear_ratio_f = (double)status.motor.gear_ratio.all / 10.0;
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      // update stroke rate but keep the rpm constant
      status.motor.cal_stroke_rate_f = updateStrokeRatebyGearRatio(status.motor.gear_ratio_f);
      status.motor.stroke_rate.all = (u16)(status.motor.cal_stroke_rate_f * 10.0);
      //ctrl.rpm.all = (u16)(status.motor.cal_stroke_rate_f * status.motor.gear_ratio_f);
#if DEBUG_PUMP_SETTING
      Serial.println("--------------------------------------------");
      Serial.printf("MenuFuncSetGearRatio():gear_ratio.all = %d\n", status.motor.gear_ratio.all);
      Serial.printf("MenuFuncSetGearRatio():gear_ratio_f = %f\n", status.motor.gear_ratio_f);
      Serial.printf("MenuFuncSetGearRatio():cal_stroke_rate_f = %f\n", status.motor.cal_stroke_rate_f);
      Serial.printf("MenuFuncSetGearRatio():stroke_rate.all = %d\n", status.motor.stroke_rate.all);
      //Serial.printf("MenuFuncSetGearRatio():ctrl.rpm.all = %d\n", ctrl.rpm.all);
#endif
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncSetBoardSeries()
//------------------------------------------------------------------
void MenuFuncSetBoardSeries() {
  static u8 board_series;
  if ((funcResponse != MENU_FUNC_CONFIRM) && (funcResponse != MENU_FUNC_WAIT_CONTROLLER)) {
    board_series = ctrl.motor_model.byte[1];
    SetParaU8str(138, menuctrl.text_ypos, &board_series, 0, 1, brdSeries_arr);
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      u8g2.clearBuffer();                // clear lcd ram buffer
      u8g2.setFont(u8g2_font_t0_12_mf);  // set font
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(0, MENU_LINE1_YPOS - 6, "WARNING: CHANGE BOARD SERIES");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
      u8g2.drawStr(0, MENU_LINE2_YPOS - 6, "Warning: This may damage the controller");
      u8g2.drawStr(0, MENU_LINE3_YPOS - 6, "if the current board series is mismatched!");
      u8g2.drawStr(0, MENU_LINE4_YPOS - 6, "Contact Trido Support for alternatives.");
      LcdUpdate();                       // Send to display & use size 12 font
      funcResponse = MENU_FUNC_CONFIRM;  //wait for user confirm
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
      funcResponse = MENU_FUNC_IDLE;
    } else if (funcResponse == MENU_FUNC_CONFIRM) {
      if (Key == KEY_ENTER) {
        ctrl.motor_model.byte[1] = board_series;
        ctrl.reset = true;  // request a reset of controller
        updateEEPROM = true;
        lockKeysExceptEscOff();
        funcResponse = MENU_FUNC_WAIT_CONTROLLER;
      } else if (Key == KEY_ESC) {
        menuctrl.runfunc = false;
        funcResponse = MENU_FUNC_IDLE;
      }
    } else if (funcResponse == MENU_FUNC_WAIT_CONTROLLER) {
      //wait for controller reset
      if (ctrl.reset_ok == true) {
        updateOCLimitByBoardMotor();
        updateEEPROM = true;
        ctrl.reset_ok = false;
        ctrl.reset = false;
        menuctrl.runfunc = false;
        funcResponse = MENU_FUNC_IDLE;
        lockKeys(0);
      } else {
        u8g2.clearBuffer();                // clear lcd ram buffer
        u8g2.setFont(u8g2_font_t0_12_mf);  // set font
        u8g2.setDrawColor(TEXT_COLOR_INVERTED);
        u8g2.drawStr(0, MENU_LINE1_YPOS - 6, "SETTING: WAITING FOR CONTROLLER RESPONSE");
        u8g2.setDrawColor(TEXT_COLOR_NORMAL);
        u8g2.drawStr(0, MENU_LINE2_YPOS - 6, "Setting: Waiting for controller to apply");
        u8g2.drawStr(0, MENU_LINE3_YPOS - 6, "new setting, if there is no response for");
        u8g2.drawStr(0, MENU_LINE4_YPOS - 6, "a few seconds, try to power cycle.");
        LcdUpdate();  // Send to display & use size 12 font
      }
    }
  }
}

//------------------------------------------------------------------
// MenuFuncSetMotorModel()
//------------------------------------------------------------------
void MenuFuncSetMotorModel() {
  static u8 motor_model;
  if ((funcResponse != MENU_FUNC_CONFIRM) && (funcResponse != MENU_FUNC_WAIT_CONTROLLER)) {
    motor_model = ctrl.motor_model.byte[0];
    SetParaU8str(138, menuctrl.text_ypos, &motor_model, 0, 5, mtrModel_arr);
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      u8g2.clearBuffer();                // clear lcd ram buffer
      u8g2.setFont(u8g2_font_t0_12_mf);  // set font
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      u8g2.drawStr(0, MENU_LINE1_YPOS - 6, "WARNING: CHANGE MOTOR MODEL");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
      u8g2.drawStr(0, MENU_LINE2_YPOS - 6, "Warning: This may damage the controller");
      u8g2.drawStr(0, MENU_LINE3_YPOS - 6, "if the connected motor model is mismatched!");
      u8g2.drawStr(0, MENU_LINE4_YPOS - 6, "Contact Trido Support for alternatives.");
      LcdUpdate();                       // Send to display & use size 12 font
      funcResponse = MENU_FUNC_CONFIRM;  //wait for user confirm
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
      funcResponse = MENU_FUNC_IDLE;
    } else if (funcResponse == MENU_FUNC_CONFIRM) {
      if (Key == KEY_ENTER) {
        ctrl.motor_model.byte[0] = motor_model;
        ctrl.reset = true;  // request a reset of controller
        updateEEPROM = true;
        lockKeysExceptEscOff();
        funcResponse = MENU_FUNC_WAIT_CONTROLLER;
      } else if (Key == KEY_ESC) {
        menuctrl.runfunc = false;
        funcResponse = MENU_FUNC_IDLE;
      }
    } else if (funcResponse == MENU_FUNC_WAIT_CONTROLLER) {
      //wait for controller reset
      if (ctrl.reset_ok == true) {
        ctrl.reset_ok = false;
        ctrl.reset = false;
        menuctrl.runfunc = false;
        funcResponse = MENU_FUNC_IDLE;
        lockKeys(0);
      } else {
        u8g2.clearBuffer();                // clear lcd ram buffer
        u8g2.setFont(u8g2_font_t0_12_mf);  // set font
        u8g2.setDrawColor(TEXT_COLOR_INVERTED);
        u8g2.drawStr(0, MENU_LINE1_YPOS - 6, "SETTING: WAITING FOR CONTROLLER RESPONSE");
        u8g2.setDrawColor(TEXT_COLOR_NORMAL);
        u8g2.drawStr(0, MENU_LINE2_YPOS - 6, "Setting: Waiting for controller to apply");
        u8g2.drawStr(0, MENU_LINE3_YPOS - 6, "new setting, if there is no response for");
        u8g2.drawStr(0, MENU_LINE4_YPOS - 6, "a few seconds, try to power cycle.");
        LcdUpdate();  // Send to display & use size 12 font
      }
    }
  }
}

void MenuFuncDisplaySettingError() {
  if (status.motor.total_pump_head == 1) {
    u8g2.clearBuffer();                // clear lcd ram buffer
    u8g2.setFont(u8g2_font_t0_12_mf);  // set font
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawStr(0, MENU_LINE1_YPOS - 6, "ERROR: VOLUME RANGE EXCEEDED");
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    u8g2.drawStr(0, MENU_LINE2_YPOS - 6, "Error: Cannot pump the set volume. A");
    u8g2.drawStr(0, MENU_LINE3_YPOS - 6, "different piston may be required.");
    u8g2.drawStr(0, MENU_LINE4_YPOS - 6, "Contact Trido support for options.");
    LcdUpdate();  // Send to display & use size 12 font
  }
}

void MenuFuncDisplaySettingResult() {
  u8g2.clearBuffer();                // clear lcd ram buffer
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.setDrawColor(TEXT_COLOR_INVERTED);
  u8g2.drawStr(0, MENU_TITLE_YPOS, "CHANGE STROKE LENGTH");
  u8g2.setDrawColor(TEXT_COLOR_NORMAL);
  u8g2.drawStr(0, MENU_LINE1_YPOS, "To reach the required pumping volume,");
  u8g2.drawStr(0, MENU_LINE2_YPOS, "stroke lengths must be maually set to:");
  for (int i = 0; i < status.motor.total_pump_head; i++) {
    if ((Lnew_inch[i] >= (status.motor.stroke_length_inch_f[i] + 0.005)) || (Lnew_inch[i] <= (status.motor.stroke_length_inch_f[i] - 0.005))) {
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      xconvert(gStr1, "PH#%d %.2f\"->%.2f\"", i + 1, status.motor.stroke_length_inch_f[i], Lnew_inch[i]);
    } else {
      xconvert(gStr1, "PH#%d %.2f\"", i + 1, Lnew_inch[i]);
    }
    if ((i == 0) || (i == 1)) {
      u8g2.drawStr(10 + 120 * i, MENU_LINE3_YPOS, gStr1);
    } else if ((i == 2) || (i == 3)) {
      u8g2.drawStr(10 + 120 * (i - 2), MENU_LINE4_YPOS, gStr1);
    }
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
  }
  LcdUpdate();  // Send to display & use size 12 font
}

//------------------------------------------------------------------
// MenuFuncSetPistonDiameter()
//------------------------------------------------------------------
void MenuFuncSetPistonDiameter() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaU8str(138, menuctrl.text_ypos, &status.motor.piston_diameter[status.motor.pump_number], 0, 4, Piston_arr);  //piston diameter
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      if (ValidatePumpSetting(status.motor.pump_rate_f[status.motor.pump_number], status.motor.pump_number) == false) {
        MenuFuncDisplaySettingError();
        funcResponse = MENU_FUNC_CONFIRM;  //wait for user to confirm
      } else {
        // multi-pumphead
        if (status.motor.total_pump_head > 1) {
          MenuFuncDisplaySettingResult();
          funcResponse = MENU_FUNC_CONFIRM;  //wait for user confirm
        } else {
          // single pumphead
          menuctrl.runfunc = false;
          updateEEPROM = true;
          funcResponse = MENU_FUNC_IDLE;
        }
      }
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
      funcResponse = MENU_FUNC_IDLE;
    } else if (funcResponse == MENU_FUNC_CONFIRM) {
      if ((Key == KEY_ENTER) || (Key == KEY_ESC)) {
        menuctrl.runfunc = false;
        updateEEPROM = true;
        funcResponse = MENU_FUNC_IDLE;
      }
    }
  }
}

//------------------------------------------------------------------
// MenuFuncSetStrokeLength()
//------------------------------------------------------------------
void MenuFuncSetStrokeLength() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaFloat(138, menuctrl.text_ypos, &status.motor.pump[status.motor.pump_number].stroke_length.all, 3, 2, 10, 200);  //stroke length
  }
  status.motor.stroke_length_inch_f[status.motor.pump_number] = (double)status.motor.pump[status.motor.pump_number].stroke_length.all / 100.0;
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      if (ValidatePumpSetting(status.motor.pump_rate_f[status.motor.pump_number], status.motor.pump_number) == false) {
        MenuFuncDisplaySettingError();
        funcResponse = MENU_FUNC_CONFIRM;  //wait for user to confirm
      } else {
        // multi-pumphead
        if (status.motor.total_pump_head > 1) {
          MenuFuncDisplaySettingResult();
          funcResponse = MENU_FUNC_CONFIRM;  //wait for user confirm
        } else {
          // single pumphead
          menuctrl.runfunc = false;
          updateEEPROM = true;
          funcResponse = MENU_FUNC_IDLE;
        }
      }
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
      funcResponse = MENU_FUNC_IDLE;
    } else if (funcResponse == MENU_FUNC_CONFIRM) {
      if ((Key == KEY_ENTER) || (Key == KEY_ESC)) {
        menuctrl.runfunc = false;
        updateEEPROM = true;
        funcResponse = MENU_FUNC_IDLE;
      }
    }
  }
}

//------------------------------------------------------------------
// MenuFuncSetPumpRate()
//------------------------------------------------------------------
void MenuFuncSetPumpRate() {
  static u16 pump_rate;
  static double pump_rate_f;
  if (funcResponse != MENU_FUNC_CONFIRM) {
    pump_rate = status.motor.pump_rate[status.motor.pump_number].all;
    SetParaFloat(138, menuctrl.text_ypos, &pump_rate, 4, 1, status.motor.min_pump_rate, status.motor.max_pump_rate);  //desired rate
    pump_rate_f = (double)pump_rate / 10.0;
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      if (ValidatePumpSetting(pump_rate_f, status.motor.pump_number) == false) {
        MenuFuncDisplaySettingError();
        funcResponse = MENU_FUNC_CONFIRM;  //wait for user to confirm
      } else {
        status.motor.pump_rate[status.motor.pump_number].all = pump_rate;
        status.motor.pump_rate_f[status.motor.pump_number] = (double)status.motor.pump_rate[status.motor.pump_number].all / 10.0;
        // multi-pumphead
        if (status.motor.total_pump_head > 1) {
          MenuFuncDisplaySettingResult();
          funcResponse = MENU_FUNC_CONFIRM;  //wait for user confirm
        } else {
          // single pumphead
          menuctrl.runfunc = false;
          updateEEPROM = true;
          funcResponse = MENU_FUNC_IDLE;
        }
      }
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
      funcResponse = MENU_FUNC_IDLE;
    } else if (funcResponse == MENU_FUNC_CONFIRM) {
      if ((Key == KEY_ENTER) || (Key == KEY_ESC)) {
        menuctrl.runfunc = false;
        updateEEPROM = true;
        funcResponse = MENU_FUNC_IDLE;
      }
    }
  }
}

//------------------------------------------------------------------
// MenuFuncSetStrokeRate()
//------------------------------------------------------------------
void MenuFuncSetStrokeRate() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    u16 smin = (u16)((double)MINIMUM_RPM * 10.0 / status.motor.gear_ratio_f);
    u16 smax = (u16)((double)MAXIMUM_RPM * 10.0 / status.motor.gear_ratio_f);
    status.motor.stroke_rate.all = (u16)(status.motor.cal_stroke_rate_f * 10.0);
    SetParaFloat(138, menuctrl.text_ypos, &status.motor.stroke_rate.all, 3, 1, smin, smax);  //stroke rate
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      status.motor.cal_stroke_rate_f = (double)status.motor.stroke_rate.all / 10.0;
      ctrl.rpm.all = (u16)(status.motor.cal_stroke_rate_f * status.motor.gear_ratio_f);
      if (ValidatePumpSetting(status.motor.pump_rate_f[status.motor.pump_number], status.motor.pump_number) == false) {
        MenuFuncDisplaySettingError();
        funcResponse = MENU_FUNC_CONFIRM;  //wait for user to confirm
      } else {
        // multi-pumphead
        if (status.motor.total_pump_head > 1) {
          MenuFuncDisplaySettingResult();
          funcResponse = MENU_FUNC_CONFIRM;  //wait for user confirm
        } else {
          // single pumphead
          menuctrl.runfunc = false;
          updateEEPROM = true;
          funcResponse = MENU_FUNC_IDLE;
        }
      }
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
      funcResponse = MENU_FUNC_IDLE;
    } else if (funcResponse == MENU_FUNC_CONFIRM) {
      if ((Key == KEY_ENTER) || (Key == KEY_ESC)) {
        menuctrl.runfunc = false;
        updateEEPROM = true;
        funcResponse = MENU_FUNC_IDLE;
      }
    }
  }
}

// fpslen
void ValidateStrokeLength(double v, u8 ph) {
  double r = 0.009525 / 2;  // piston radius
  double l;                 // stroke length
  double f;                 // correction factor
  double w;                 // stroke rate
  double pump_rate = v;     // pump rate set
  double r2_m_f;            // r^2

  //const double lmin = 0.00635;                                  // minimum stroke length = 0.25"
  const double lmax = 0.0254;                                   // maximum stroke length = 1.0"
  const double wmin = MINIMUM_RPM / status.motor.gear_ratio_f;  // minimum stroke rate limited by motor speed
  const double wmax = MAXIMUM_RPM / status.motor.gear_ratio_f;  // maximum stroke rate limited by motor speed
  double c;                                                     // L/gal unit conversion factor

  // set user adjust limit
  status.motor.max_pump_rate_f = 999.9;
  status.motor.max_pump_rate = 9999;
  status.motor.min_pump_rate_f = 0.1;
  status.motor.min_pump_rate = 1;

  // multi-head pump
  if (status.motor.total_pump_head > 1) {
    // piston radius
    r = getPistonRadius(ph);
    r2_m_f = r * r;                                 // r2
    f = status.motor.pump[ph].correction_factor_f;  // correction factor
    c = status.motor.conversion_factor_f;           // L/gal unit conversion factor
#if DEBUG_PUMP_SETTING
    Serial.println("--------------------------------------------");
    Serial.println("CHANGE_STROKE_LENGTH");
#endif
    // run the current stroke length test
    updateStrokeRateFromRpm(ctrl.rpm.all);  // stroke per min
    w = status.motor.cal_stroke_rate_f;
    l = pump_rate * f / (PI_VALUE * r2_m_f * c * w * 1440.0);  // calculate stroke length
#if DEBUG_PUMP_SETTING
    Serial.printf("l:%f\"\n", l / 0.0254);
#endif
    status.motor.stroke_length_inch_f[ph] = l / 0.0254;  // calculate stroke length in inch
    status.motor.pump[ph].stroke_length.all = (u16)(status.motor.stroke_length_inch_f[ph] * 100.0);
    pseterr[ph].bit.stroke_length_high_err = 0;  // clear the error
    pseterr[ph].bit.stroke_length_low_err = 0;   // clear the error
#if DEBUG_PUMP_SETTING
    Serial.println("SUCCESS & return");
#endif
  } else  // 1 pump head only
  {
    // piston radius
    if (status.motor.piston_diameter[ph] == 0) {
      r = 0.0047625 / 2;  //3/16"
    } else if (status.motor.piston_diameter[ph] == 1) {
      r = 0.00635 / 2;  //1/4"
    } else if (status.motor.piston_diameter[ph] == 2) {
      r = 0.009525 / 2;  //3/8"
    } else if (status.motor.piston_diameter[ph] == 3) {
      r = 0.0127 / 2;  //1/2"
    } else if (status.motor.piston_diameter[ph] == 4) {
      r = 0.01905 / 2;  //3/4"
    }

    r2_m_f = r * r;                                 // r2
    f = status.motor.pump[ph].correction_factor_f;  // correction factor
    c = status.motor.conversion_factor_f;           // L/gal unit conversion factor
    l = lmax;                                       // stroke length

    w = pump_rate * f / (PI_VALUE * r2_m_f * c * l * 1440.0);  // calculate stroke per min MAXIMUM_RPM / status.motor.gear_ratio_f;

    if ((w <= wmax) && (w >= wmin)) {
      // update motor speed
      status.motor.cal_stroke_rate_f = w;
      status.motor.stroke_rate.all = (u16)(status.motor.cal_stroke_rate_f * 10.0);
      ctrl.rpm.all = (u16)(w * status.motor.gear_ratio_f);
    }
  }
}

//------------------------------------------------------------------
// MenuFuncCalibration()
//------------------------------------------------------------------
void MenuFuncCalibration() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaFloat(174, menuctrl.text_ypos, &status.motor.actual_rate[status.motor.pump_number].all, 4, 1, status.motor.min_pump_rate, status.motor.max_pump_rate);  //desired rate
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      status.motor.pump[status.motor.pump_number].calibration_dnt = dnt;
      status.motor.desired_rate_f[status.motor.pump_number] = status.motor.pump_rate_f[status.motor.pump_number];
      status.motor.desired_rate[status.motor.pump_number].all = (u16)(status.motor.desired_rate_f[status.motor.pump_number] * 10.0);
      status.motor.actual_rate_f[status.motor.pump_number] = (double)status.motor.actual_rate[status.motor.pump_number].all / 10.0;
      // update correction factor
      status.motor.pump[status.motor.pump_number].correction_factor_f = status.motor.desired_rate_f[status.motor.pump_number] / status.motor.actual_rate_f[status.motor.pump_number];
      status.motor.pump[status.motor.pump_number].correction_factor.all = (u16)(status.motor.pump[status.motor.pump_number].correction_factor_f * 10000);
      if (ValidatePumpSetting(status.motor.pump_rate_f[status.motor.pump_number], status.motor.pump_number) == false) {
        MenuFuncDisplaySettingError();
        funcResponse = MENU_FUNC_CONFIRM;  //wait for user to confirm
      } else {
        // multi-pumphead
        if (status.motor.total_pump_head > 1) {
          MenuFuncDisplaySettingResult();
          funcResponse = MENU_FUNC_CONFIRM;  //wait for user confirm
        } else {
          // single pumphead
          menuctrl.runfunc = false;
          updateEEPROM = true;
          funcResponse = MENU_FUNC_IDLE;
        }
      }
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
      funcResponse = MENU_FUNC_IDLE;
    } else if (funcResponse == MENU_FUNC_CONFIRM) {
      if ((Key == KEY_ENTER) || (Key == KEY_ESC)) {
        menuctrl.runfunc = false;
        updateEEPROM = true;
        funcResponse = MENU_FUNC_IDLE;
      }
    }
  }
}

//------------------------------------------------------------------
// MenuFuncSetPumpHead()
//------------------------------------------------------------------
void MenuFuncSetPumpHead() {
  SetParaU8(138, menuctrl.text_ypos, &status.motor.total_pump_head, 1, 1, 4);
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      // change stroke length to 1" if ph = 1
      if (status.motor.total_pump_head == 1) {         // 0 = 1/3", 1 = 1/2", 2 = 3/4", 3 = 1", 4 = 1.5", 5 = 2"
        status.motor.stroke_length_inch_f[0] = 1.0;    // 1.0"
        status.motor.pump[0].stroke_length.all = 100;  // 1.0"
        ValidatePumpSetting(status.motor.pump_rate_f[0], 0);
      }
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncSetMinDcv()
//------------------------------------------------------------------
void MenuFuncSetMinDcv() {
  SetParaFloat(138, menuctrl.text_ypos, &ctrl.min_dcv_input.all, 3, 1, 180, 319);
  ctrl.min_dcv_input_f = (double)ctrl.min_dcv_input.all / 10.0;

  if (funcResponse > MENU_FUNC_IDLE) {
    checkUserSettings();
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncSetSlowdownDcv()
//------------------------------------------------------------------
void MenuFuncSetSlowdownDcv() {
  SetParaFloat(138, menuctrl.text_ypos, &ctrl.slow_down_dcv.all, 3, 1, 180, 319);
  ctrl.slow_down_dcv_f = (double)ctrl.slow_down_dcv.all / 10.0;
  if (funcResponse > MENU_FUNC_IDLE) {
    checkUserSettings();
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncSetDcAmp()
//------------------------------------------------------------------
void MenuFuncSetDcAmp() {
  updateOCLimitByBoardMotor();
  SetParaFloat(138, menuctrl.text_ypos, &ctrl.ocp_dcamp.all, 3, 1, MINIMUM_OCP_DCAMP, MAXIMUM_OCP_DCAMP);
  ctrl.ocp_dcamp_f = (double)ctrl.ocp_dcamp.all / 10.0;
  if (funcResponse > MENU_FUNC_IDLE) {
    checkUserSettings();
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncResetRuntime()
//------------------------------------------------------------------
void MenuFuncResetRuntime() {
  SetParaBoolstr(138, menuctrl.text_ypos, &status.motor.reset_runtime, runtimeReset_arr);
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      if (status.motor.reset_runtime == 1) {
        //status.motor.runtime_total = 0;
        status.motor.runtime_day = 0;
        status.motor.runtime_hour = 0;
        status.motor.runtime_min = 0;
        status.motor.runtime_sec = 0;
        updateEEPROM = true;
        status.motor.reset_runtime = 0;
        strcpy(userMsg[1], "RUN TIME RESET!");
      }
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFunc()
//------------------------------------------------------------------
void MenuFuncSetDioMode() {
  SetParaBoolstr(138, menuctrl.text_ypos, &status.motor.dio_mode, enabledDisabled_arr);
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      //switch to local mode if DIO mode was used
      if (status.motor.dio_mode) {
        if (ctrl.mode == DIO_MODE) (ctrl.mode = LOCAL_MODE);
      }
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncComFailAction()
//------------------------------------------------------------------
void MenuFuncComFailAction() {
  SetParaBoolstr(138, menuctrl.text_ypos, &ctrl.communication_fail_action, comfailAction_arr);
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncSetVolumeUnit()
//------------------------------------------------------------------
void MenuFuncSetVolumeUnit() {
  SetParaU8str(138, menuctrl.text_ypos, &status.motor.volunit, 0, 1, volUnit_arr);  // L/gal Unit
  getVolUnit();
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

int getFfromDegC(int degc) {
  double tmp;
  int ret;

  tmp = degc * 1.8 + 32.0;
  ret = (int)tmp;

  return ret;
}

int getCfromF(int f) {
  double tmp;
  int ret;

  tmp = (f - 32.0) / 1.8;
  ret = (int)tmp;

  return ret;
}

//------------------------------------------------------------------
// MenuFuncSetTempUnit()
//------------------------------------------------------------------
void MenuFuncSetTempUnit() {
  SetParaU8str(144, menuctrl.text_ypos, &status.motor.tempunit, 0, 1, tempUnit_arr);  // deg C / F
  getTempUnit();
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      if (status.motor.tempunit == 0)  //deg C
      {
        status.motor.off_temp.all = getCfromF(status.motor.off_temp.all);
        status.motor.on_temp.all = getCfromF(status.motor.on_temp.all);
        status.motor.temp_now.all = getCfromF(status.motor.temp_now.all);
      } else  //F
      {
        status.motor.off_temp.all = getFfromDegC(status.motor.off_temp.all);
        status.motor.on_temp.all = getFfromDegC(status.motor.on_temp.all);
        status.motor.temp_now.all = getFfromDegC(status.motor.temp_now.all);
      }
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncSetTempCtrl()
//------------------------------------------------------------------
void MenuFuncSetTempCtrl() {
  SetParaU8str(156, menuctrl.text_ypos, &status.motor.temp_ctrl, 0, 1, enabledDisabled_arr);
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncSetOffTemp()
//------------------------------------------------------------------
void MenuFuncSetOffTemp() {
  if (status.motor.tempunit == 0) {
    SetParaInt(156, menuctrl.text_ypos, &status.motor.off_temp.all, 3, -127, 127);  // deg C
  } else {
    SetParaInt(156, menuctrl.text_ypos, &status.motor.off_temp.all, 3, -197, 260);  // F
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncSetOnTemp()
//------------------------------------------------------------------
void MenuFuncSetOnTemp() {
  if (status.motor.tempunit == 0) {
    SetParaInt(156, menuctrl.text_ypos, &status.motor.on_temp.all, 3, -127, 127);  // deg C
  } else {
    SetParaInt(156, menuctrl.text_ypos, &status.motor.on_temp.all, 3, -197, 260);  // F
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuWaitUserAction()
//------------------------------------------------------------------
void MenuWaitUserAction() {
  if (funcResponse == MENU_FUNC_EXIT_SAVE) {
    if (sysstat.bit.reboot_pending == false) {
      char usermsg[3][42];
      u8g2.clearBuffer();                // clear lcd ram buffer
      u8g2.setFont(u8g2_font_t0_14_mf);  // set font
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);
      strcpy(usermsg[0], "SYSTEM REBOOT REQUIRED");
      strcpy(usermsg[1], "Do you want to reboot now?");
      strcpy(usermsg[2], "YES(ENTER)/NO(ESC)");
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(usermsg[0]) * 7) / 2, 16, usermsg[0]);
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(usermsg[1]) * 7) / 2, 32, usermsg[1]);
      u8g2.drawStr((MAX_LCD_WIDTH - strlen(usermsg[2]) * 7) / 2, 48, usermsg[2]);
      LcdUpdate();                       // Send to display & use size 12 font
      funcResponse = MENU_FUNC_CONFIRM;  //wait for user YES/NO confirm
    } else {
      updateEEPROM = true;
      menuctrl.runfunc = false;
      funcResponse = MENU_FUNC_IDLE;
    }
  } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
    menuctrl.runfunc = false;
    funcResponse = MENU_FUNC_IDLE;
  } else if (funcResponse == MENU_FUNC_CONFIRM) {
    if (sysstat.bit.reboot_pending == false) {
      if (Key == KEY_ENTER) {
        menuctrl.runfunc = false;
        writeEEPROMdata();  //save setting
        ESP.restart();
      } else if (Key == KEY_ESC) {
        updateEEPROM = true;
        menuctrl.runfunc = false;
        sysstat.bit.reboot_pending = true;
        funcResponse = MENU_FUNC_IDLE;
      }
    }
    // we should never get here
    else {
      menuctrl.runfunc = false;
      funcResponse = MENU_FUNC_IDLE;
    }
  }
}

//------------------------------------------------------------------
// MenuFuncSetProtocol()
//------------------------------------------------------------------
void MenuFuncSetProtocol() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaU8str(150, menuctrl.text_ypos, &scada_setting.protocol, 0, 1, Protocol_arr);  //0=RTU, 1=ASCII, 2=TCP
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncSetServerID()
//------------------------------------------------------------------
void MenuFuncSetServerID() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaU8(150, menuctrl.text_ypos, &scada_setting.server_id, 3, 1, 247);  //server_id
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncSetTimeout()
//------------------------------------------------------------------
void MenuFuncSetTimeout() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaUint(150, menuctrl.text_ypos, &scada_setting.timeout.all, 5, 100, 65535);  //timeout
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncCoilAddr()
//------------------------------------------------------------------
void MenuFuncSetCoilAddr() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaUint(150, menuctrl.text_ypos, &scada_setting.coil_addr.all, 5, 0, 65535 - scada_setting.max_coil);  //coil_addr
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncSetDiscInputAddr()
//------------------------------------------------------------------
void MenuFuncSetDiscInputAddr() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaUint(150, menuctrl.text_ypos, &scada_setting.discrete_addr.all, 5, 0, 65535 - scada_setting.max_discrete);  //discrete_addr
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncSetInputRegAddr()
//------------------------------------------------------------------
void MenuFuncSetInputRegAddr() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaUint(150, menuctrl.text_ypos, &scada_setting.inputreg_addr.all, 5, 0, 65535 - scada_setting.max_inputreg);  //inputreg_addr
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncSetHoldRegAddr()
//------------------------------------------------------------------
void MenuFuncSetHoldRegAddr() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaUint(150, menuctrl.text_ypos, &scada_setting.holdreg_addr.all, 5, 0, 65535 - scada_setting.max_holdreg);  //holdreg_addr
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncSetBaudrate()
//------------------------------------------------------------------
void MenuFuncSetBaudrate() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SelectParaFromList(150, menuctrl.text_ypos, &comport.baud.all, baudRate, 12 /*maxitem*/, 6 /*maxclen*/);  //baud 300-921600
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncSetDataBit()
//------------------------------------------------------------------
void MenuFuncSetDataBit() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaU8(150, menuctrl.text_ypos, &comport.databits, 1, 7, 8);  //databits, 7/8
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncSetStopBit()
//------------------------------------------------------------------
void MenuFuncSetStopBit() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaU8(150, menuctrl.text_ypos, &comport.stopbits, 1, 1, 2);  //stopbits, 0/1
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncSetParity()
//------------------------------------------------------------------
void MenuFuncSetParity() {
  if (funcResponse != MENU_FUNC_CONFIRM) {
    SetParaU8str(150, menuctrl.text_ypos, &comport.parity, 0, 2, Parity_arr);  //parity [0=NONE, 1=ODD, 2=EVEN]
  }
  MenuWaitUserAction();  // required a reboot
}

//------------------------------------------------------------------
// MenuFuncCarbonCredit()
//------------------------------------------------------------------
void MenuFuncCarbonCredit() {
  static u8 sel_item = 0;
  static u16 current_page = 0;
  const u16 max_page = 6;
  bool print = false;
  const u8 totitem = 4;

  if (copyValue) {
    sel_item = 0;
    Key = NO_KEY;
    copyValue = false;
  }

  if (Key == KEY_UP || keyUphold) {
  } else if (Key == KEY_DOWN || keyDownhold) {
  } else if (Key == KEY_LEFT) {
    MenuMoveItemIndex(&sel_item, totitem);
    funcUpdate = true;
  } else if (Key == KEY_RIGHT) {
    MenuMoveItemIndex(&sel_item, totitem);
    funcUpdate = true;
  } else if (Key == KEY_ENTER) {
    if (sdState) {
      if (sel_item == 0) {
        if (current_page < max_page - 1) {
          current_page++;
        }
      } else if (sel_item == 1) {
        if (current_page > 0) {
          current_page--;
        }
      } else if (sel_item == 2) {
        print = true;
      } else if (sel_item == 3) {
        if (writeFileflag) {
          strcpy(userMsg[0], "SD Unmount:");
          strcpy(userMsg[1], "SD Card Is In Use!");
          strcpy(userMsg[2], "Please Try Again!");
        } else {
          sdEject = true;
          SD.end();
          sdState = false;
          sdEject = false;
        }
      }
    } else {
      // SD
      sdState = mountSD();
      if (!sdState) {
        strcpy(userMsg[0], "SD Mount:");
        strcpy(userMsg[1], "SD Card Not Detected!");
        strcpy(userMsg[2], "Please Try Again!");
        u8g2.clearBuffer();                // clear lcd ram buffer
        u8g2.setFont(u8g2_font_t0_14_mf);  // set font
        u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[0]) * 7) / 2, 16, userMsg[0]);
        u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[1]) * 7) / 2, 32, userMsg[1]);
        u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[2]) * 7) / 2, 48, userMsg[2]);
        userMsg[0][0] = '\0';
        userMsg[1][0] = '\0';
        userMsg[2][0] = '\0';
        LcdUpdate();  // Send to display & use size 12 font
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        copyValue = true;  // cause a re-print
      }
    }
    funcUpdate = true;
  } else if (Key == KEY_ESC) {
    copyValue = true;
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
  }

  u8g2.clearBuffer();                      // clear lcd ram buffer
  u8g2.setDrawColor(TEXT_COLOR_INVERTED);  //set color
  u8g2.drawStr(0, MENU_TITLE_YPOS, "CARBON CREDIT SUMMARY");
  u8g2.setDrawColor(TEXT_COLOR_NORMAL);  //set color
  u8g2.setFont(u8g2_font_t0_12_mf);      // set font
  u8g2.setFontMode(0);                   //set font mode, no transparency
  MenuDrawArrowAtArray(menuPosCarbon, totitem, sel_item);

  //if SD is not mounted
  if (!sdState) {
    u8g2.clearBuffer();                      // clear lcd ram buffer
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);  //set color
    u8g2.drawStr(0, MENU_TITLE_YPOS, "CARBON CREDIT SUMMARY");
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);  //set color
    u8g2.drawButtonUTF8(35, MENU_LINE2_YPOS, U8G2_BTN_INV | U8G2_BTN_BW2, 0, 2, 2, "Insert SD Card and Press ENTER!");
    u8g2.setFontMode(0);
  } else {
    //SD Card Type Icon
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);  //set color
    u8g2.drawStr(140, MENU_TITLE_YPOS, cardSizeStr);
    u8g2.drawStr(222, MENU_TITLE_YPOS, cardTypeStr);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);  //set color
    //Record Type Select
    u8g2.drawStr(6, MENU_LINE1_YPOS, "NEXT");
    u8g2.drawStr(6, MENU_LINE2_YPOS, "PREV");
    //Print to USB
    u8g2.drawStr(6, MENU_LINE3_YPOS, "PRINT");
    //Unmount SD Card
    u8g2.drawStr(6, MENU_LINE4_YPOS, "EJECT");
    //Page No
    xconvert(gStr1, "%d/%d", current_page + 1, max_page);
    u8g2.drawStr(225, MENU_LINE4_YPOS, gStr1);
    //Month
    u8 index = 0;
    u8 ypos = 0;
    // Print to COM Port is slow may exit the RTOS task, use a separated for() loop
    if (print) {
      Serial.println("CARBON CREDIT SUMMARY");
      for (u16 j = 0; j < 24; j++) {
        if (status.motor.pump_record.record_date[j].byte[0] != 0) {
          xconvert(gStr1, "%d/%02d %09d stroke",
                   status.motor.pump_record.record_date[j].byte[0] + 2000,
                   status.motor.pump_record.record_date[j].byte[1],
                   status.motor.pump_record.month_stroke_count[j].all);
          Serial.println(gStr1);
        } else {
          //no record
          Serial.println("------- --------- ------");
        }
      }
      print = false;
    }
    // Display Record
    for (u16 p = 0; p < max_page; p++) {
      for (u16 j = 0; j < 4; j++) {
        index = current_page * 4 + j;
        ypos = MENU_LINE1_YPOS + j * 13;
        u8g2.drawStr(51, ypos, monthStr[index % 12]);
        if (status.motor.pump_record.record_date[index].byte[0] != 0) {
          xconvert(gStr1, "%d/%02d %09d stroke",
                   status.motor.pump_record.record_date[index].byte[0] + 2000,
                   status.motor.pump_record.record_date[index].byte[1],
                   status.motor.pump_record.month_stroke_count[index].all);
          u8g2.drawStr(75, ypos, gStr1);
        } else {
          //no record
          u8g2.drawStr(75, ypos, "------- --------- ------");
        }
      }
    }
  }

  if (funcUpdate) {
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }
    funcUpdate = false;
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      //updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncHistoryFault()
//------------------------------------------------------------------
void MenuFuncHistoryFault() {
  static u8 current_page = 0;
  static u8 max_page = 0;

  if (copyValue) {
    Key = NO_KEY;
    copyValue = false;
    funcUpdate = true;
  }

  if (Key == KEY_UP || keyUphold) {
  } else if (Key == KEY_DOWN || keyDownhold) {
  } else if (Key == KEY_LEFT) {
    if (current_page > 0)
      current_page--;
    funcUpdate = true;
  } else if (Key == KEY_RIGHT) {
    if (max_page > 0) {
      if (current_page < max_page - 1)
        current_page++;
    }
    funcUpdate = true;
  } else if (Key == KEY_ENTER) {
    funcUpdate = true;
  } else if (Key == KEY_ESC) {
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
    copyValue = true;
  }

  u8g2.clearBuffer();                // clear lcd ram buffer
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8 num_records = 0;
  for (u8 i = 0; i < MAX_HISTORY_FAULT_RECORD; i++) {
    if (eep.history[i].err.all != 0) {
      num_records++;
    }
  }
  max_page = num_records;
  xconvert(gStr1, "%s", "FAULT HISTORY");
  u8g2.setDrawColor(TEXT_COLOR_INVERTED);  //set color
  u8g2.drawStr(0, MENU_TITLE_YPOS, gStr1);
  u8g2.setDrawColor(TEXT_COLOR_NORMAL);  //set color
  if (num_records > 1) {
    xconvert(gStr1, " - %d RECORDS", num_records);
  } else {
    xconvert(gStr1, " - %d RECORD", num_records);
  }
  u8g2.drawStr(78, MENU_TITLE_YPOS, gStr1);

  if (eep.history[0].err.all != 0) {
    if (eep.history_index < MAX_HISTORY_FAULT_RECORD) {
      //DATE & TIME
      xconvert(gStr1, "Recorded On            %d/%02d/%02d %02d:%02d:%02d", eep.history[current_page].dnt.year + 2000, eep.history[current_page].dnt.month, eep.history[current_page].dnt.day, eep.history[current_page].dnt.hour, eep.history[current_page].dnt.minute, eep.history[current_page].dnt.second);
      u8g2.drawStr(0, MENU_LINE1_YPOS, gStr1);
      //ERROR CODE
      xconvert(gStr1, "Error Code             %03d", eep.history[current_page].err.all);
      u8g2.drawStr(0, MENU_LINE2_YPOS, gStr1);
      //STATUS CODE
      xconvert(gStr1, "Status Code            %03d", eep.history[current_page].status.all);
      u8g2.drawStr(0, MENU_LINE3_YPOS, gStr1);
      //Record number
      if (max_page != 0)
        xconvert(gStr1, "Record:       %02d/%02d", current_page + 1, max_page);
      else
        xconvert(gStr1, "Record:       %02d/%02d", 0, 0);
      u8g2.drawStr(138, MENU_LINE4_YPOS, gStr1);
    } else {
      //data index error
    }
  } else {
    u8g2.drawStr(75, MENU_LINE2_YPOS, "NO RECORD FOUND!");
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
      current_page = 0;
      updateEEPROM = true;  //save setting
      strcpy(userMsg[1], "FAULT HISTORY RESET!");
    } else {
      strcpy(userMsg[0], "CANNOT PERFORM");
      strcpy(userMsg[1], "FAULT HISTORY RESET");
      strcpy(userMsg[2], "REMOVE FAULT FIRST!");
    }
    u8g2.clearBuffer();                // clear lcd ram buffer
    u8g2.setFont(u8g2_font_t0_14_mf);  // set font
    u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[0]) * 7) / 2, 16, userMsg[0]);
    u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[1]) * 7) / 2, 32, userMsg[1]);
    u8g2.drawStr((MAX_LCD_WIDTH - strlen(userMsg[2]) * 7) / 2, 48, userMsg[2]);
    userMsg[0][0] = '\0';
    userMsg[1][0] = '\0';
    userMsg[2][0] = '\0';
    LcdUpdate();  // Send to display & use size 12 font
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    copyValue = true;  // cause a re-print
  }

  if (funcUpdate) {
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }
    funcUpdate = false;
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncSetDatetime()
//------------------------------------------------------------------
void MenuFuncSetDatetime() {
  u8g2.clearBuffer();                // clear lcd ram buffer
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  u8g2.drawStr(5, MENU_LINE4_YPOS, "[UP/DN/L/R=change ENTER=save ESC=discard]");

  setDatetimeFunc(75, MENU_LINE2_YPOS);

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      //updateEEPROM = true;
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// MenuFuncSystemInfo()
//------------------------------------------------------------------
void MenuFuncSystemInfo() {
  if (copyValue) {
    Key = NO_KEY;
    copyValue = false;
    funcUpdate = true;
  }

  if (Key == KEY_UP || keyUphold) {
  } else if (Key == KEY_DOWN || keyDownhold) {
  } else if (Key == KEY_LEFT) {
  } else if (Key == KEY_RIGHT) {
  } else if (Key == KEY_ENTER) {
  } else if (Key == KEY_ESC) {
    funcResponse = MENU_FUNC_EXIT_NO_SAVE;
    copyValue = true;
  }

  u8g2.clearBuffer();                      // clear lcd ram buffer
  u8g2.setFont(u8g2_font_t0_12_mf);        // set font
  u8g2.setDrawColor(TEXT_COLOR_INVERTED);  //set color
  u8g2.drawStr(0, MENU_TITLE_YPOS, "SYSTEM INFO");
  u8g2.setDrawColor(TEXT_COLOR_NORMAL);  //set color
  //1.Hardware Version
  u8g2.drawStr(0, MENU_LINE1_YPOS, "1.Hardware Version");
  u8g2.drawStr(128, MENU_LINE1_YPOS, HARDWARE_VERSION_STRING);
  //2.Software Version
  u8g2.drawStr(0, MENU_LINE2_YPOS, "2.Software Version");
  u8g2.drawStr(128, MENU_LINE2_YPOS, SOFTWARE_VERSION_STRING);
  // Enter Key hold for 1s to software reset the ESP32
  /*if (keyEnterhold) {
    u8g2.clearBuffer();  // clear lcd ram buffer
    u8g2.drawStr(80, MENU_LINE2_YPOS, "RESTARTING...");
    LcdUpdate();
    delay(1000);
    ESP.restart();
  }*/

  if (funcUpdate) {
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }
    funcUpdate = false;
  }
  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      menuctrl.runfunc = false;
    } else if (funcResponse == MENU_FUNC_EXIT_NO_SAVE) {
      menuctrl.runfunc = false;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//------------------------------------------------------------------
// Date and Time Setting
//------------------------------------------------------------------
void setDatetimeFunc(u16 x, u16 y) {
  static u16 *p = NULL;
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
    Key = NO_KEY;
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
      p = &dnt1.year;
      mul = power(10, maxindex - cusorPos);
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
      p = &dnt1.month;
      mul = power(10, maxindex - (cusorPos - 5));
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
      p = &dnt1.day;
      mul = power(10, maxindex - (cusorPos - 8));
      break;
    case 11:
    case 12:
      maxvlen = 2;
      maxindex = maxvlen - 1;
      p = &dnt1.hour;
      minval = 0;
      maxval = 23;
      mul = power(10, maxindex - (cusorPos - 11));
      break;
    case 14:
    case 15:
      maxvlen = 2;
      maxindex = maxvlen - 1;
      p = &dnt1.minute;
      minval = 0;
      maxval = 59;
      mul = power(10, maxindex - (cusorPos - 14));
      break;
    case 17:
    case 18:
      maxvlen = 2;
      maxindex = maxvlen - 1;
      p = &dnt1.second;
      minval = 0;
      maxval = 59;
      mul = power(10, maxindex - (cusorPos - 17));
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
      if (*p <= (maxval - mul)) *p += mul;
      funcUpdate = true;
    }
  } else if (Key == KEY_DOWN || keyDownhold) {
    if (validpos) {
      if (*p >= (minval + mul)) *p -= mul;
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
    if (sdIsSaving == false) {
      u8g2.sendBuffer();  // send to LCD
    }
    funcUpdate = false;
  }
}

void checkUserSettings() {
  //warn the user for once when user hits ENTER with wrong setting values
  if ((ctrl.slow_down_dcv.all <= ctrl.min_dcv_input.all)
      || (ctrl.slow_down_dcv.all >= ctrl.max_dcv_input.all)
      || (ctrl.max_dcv_input.all <= ctrl.min_dcv_input.all)) {
    syserr.bit.setting_value_err = true;
    strcpy(userMsg[0], "WRONG SETTINGS!");
    strcpy(userMsg[1], "THE PUMP WILL BE");
    strcpy(userMsg[2], "STOPPED ON ERROR!");
  } else {
    syserr.bit.setting_value_err = false;
  }
}

//-------------------------------------------------------------------
// startupFunc1()
//-------------------------------------------------------------------
void startupFunc1() {
  static u8 item = 0;
  u8g2.clearBuffer();                // clear lcd ram buffer
  u8g2.setFont(u8g2_font_t0_12_mf);  // set font
  funcUpdate = true;
  switch (item) {
    case 0:
      u8g2.drawStr(5, MENU_LINE4_YPOS, "[UP/DN/L/R=change ENTER=save ESC=discard]");
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);  //set color
      u8g2.drawStr(81, MENU_LINE1_YPOS, "SET SYSTEM TIME");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);  //set color
      u8g2.drawStr(69, MENU_LINE2_YPOS, ">");
      setDatetimeFunc(75, MENU_LINE2_YPOS);
      break;
    case 1:
      u8g2.drawStr(17, MENU_LINE4_YPOS, "[UP/DN=change ENTER=save ESC=discard]");
      u8g2.setDrawColor(TEXT_COLOR_INVERTED);  //set color
      u8g2.drawStr(69, MENU_LINE1_YPOS, "SELECT UNIT FOR VOLUME");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);  //set color
      u8g2.drawStr(98, MENU_LINE2_YPOS, ">");
      SetParaU8str(104, MENU_LINE2_YPOS, &status.motor.volunit, 0, 1, volUnit_arr);
      // L/gal Unit
      getVolUnit();
      break;
    case 2:
      ctrl.startup_setting_completed = true;
      updateEEPROM = true;
      break;
  }

  if (funcResponse > MENU_FUNC_IDLE) {
    if (funcResponse == MENU_FUNC_EXIT_SAVE) {
      item++;
    }
    funcResponse = MENU_FUNC_IDLE;
  }
}

//-----------------------------
// getChecksum()
//-----------------------------
u16 getChecksum(unsigned int *buf, u16 bytecount) {
  UINT16_T crcval;

  crcval.all = calCrc(buf, bytecount);

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.printf("getChecksum crc:%u %u\n", crcval.byte[0], crcval.byte[1]);
#endif

  return crcval.all;
}

//-----------------------------
// prepareWrbuf()
//-----------------------------
void prepareWrbuf(unsigned int *buf, u16 *bytecount) {
  u16 index = 0;
  UINT16_T crcval;

  // VFD = 7 bytes
  buf[index++] = eep.ctrl.rpm.byte[0];
  buf[index++] = eep.ctrl.rpm.byte[1];
  buf[index++] = eep.ctrl.onoff;
  buf[index++] = eep.ctrl.mode;
  buf[index++] = eep.ctrl.startup_setting_completed;
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

  // Injection Pump = 251 bytes
  buf[index++] = eep.ctrl.min_dcv_input.byte[0];
  buf[index++] = eep.ctrl.min_dcv_input.byte[1];
  buf[index++] = eep.ctrl.max_dcv_input.byte[0];
  buf[index++] = eep.ctrl.max_dcv_input.byte[1];
  buf[index++] = eep.ctrl.slow_down_dcv.byte[0];
  buf[index++] = eep.ctrl.slow_down_dcv.byte[1];
  buf[index++] = eep.ctrl.ocp_dcamp.byte[0];
  buf[index++] = eep.ctrl.ocp_dcamp.byte[1];
  buf[index++] = eep.ctrl.communication_fail_action;
  buf[index++] = eep.motor.volunit;
  buf[index++] = eep.motor.dio_mode;
  buf[index++] = eep.motor.total_pump_head;
  buf[index++] = eep.motor.tempunit;
  buf[index++] = eep.motor.temp_ctrl;
  buf[index++] = eep.motor.off_temp.byte[0];
  buf[index++] = eep.motor.off_temp.byte[1];
  buf[index++] = eep.motor.on_temp.byte[0];
  buf[index++] = eep.motor.on_temp.byte[1];  // 18 bytes

  for (u8 i = 0; i < NO_OF_PUMPS; i++) {
    buf[index++] = (u8)(eep.motor.pump[i].calibration_dnt.year - 2000);
    buf[index++] = (u8)eep.motor.pump[i].calibration_dnt.month;
    buf[index++] = (u8)eep.motor.pump[i].calibration_dnt.day;
    buf[index++] = (u8)eep.motor.pump[i].calibration_dnt.hour;
    buf[index++] = (u8)eep.motor.pump[i].calibration_dnt.minute;
    buf[index++] = (u8)eep.motor.pump[i].calibration_dnt.second;  // 24 bytes

    buf[index++] = eep.motor.stroke_length[i].all;
    buf[index++] = eep.motor.piston_diameter[i];
    buf[index++] = eep.motor.desired_rate[i].byte[0];
    buf[index++] = eep.motor.desired_rate[i].byte[1];
    buf[index++] = eep.motor.actual_rate[i].byte[0];
    buf[index++] = eep.motor.actual_rate[i].byte[1];
    buf[index++] = eep.motor.pump_rate[i].byte[0];
    buf[index++] = eep.motor.pump_rate[i].byte[1];
    buf[index++] = eep.motor.pump[i].correction_factor.byte[0];
    buf[index++] = eep.motor.pump[i].correction_factor.byte[1];
    buf[index++] = eep.motor.pump[i].stroke_length.byte[0];
    buf[index++] = eep.motor.pump[i].stroke_length.byte[1];  // 48 bytes
  }

  buf[index++] = eep.motor.stroke_rate.byte[0];
  buf[index++] = eep.motor.stroke_rate.byte[1];  // 2 bytes

  buf[index++] = eep.motor.gear_ratio.byte[0];
  buf[index++] = eep.motor.gear_ratio.byte[1];  // 2 bytes

  for (u8 i = 0; i < 24; i++) {
    buf[index++] = eep.motor.pump_record.record_date[i].byte[0];
    buf[index++] = eep.motor.pump_record.record_date[i].byte[1];

    buf[index++] = eep.motor.pump_record.month_stroke_count[i].byte[0];
    buf[index++] = eep.motor.pump_record.month_stroke_count[i].byte[1];
    buf[index++] = eep.motor.pump_record.month_stroke_count[i].byte[2];
    buf[index++] = eep.motor.pump_record.month_stroke_count[i].byte[3];  //144 bytes
  }
  buf[index++] = eep.motor.pump_record.record_index;  // 1 bytes
  buf[index++] = eep.motor.pump_record.hour_stroke_count.byte[0];
  buf[index++] = eep.motor.pump_record.hour_stroke_count.byte[1];
  buf[index++] = eep.motor.pump_record.hour_stroke_count.byte[2];
  buf[index++] = eep.motor.pump_record.hour_stroke_count.byte[3];  //4 bytes
  buf[index++] = eep.motor.pump_record.day_stroke_count.byte[0];
  buf[index++] = eep.motor.pump_record.day_stroke_count.byte[1];
  buf[index++] = eep.motor.pump_record.day_stroke_count.byte[2];
  buf[index++] = eep.motor.pump_record.day_stroke_count.byte[3];  // 4 bytes
  buf[index++] = eep.motor.pump_record.accumulated_stroke_count.byte[0];
  buf[index++] = eep.motor.pump_record.accumulated_stroke_count.byte[1];
  buf[index++] = eep.motor.pump_record.accumulated_stroke_count.byte[2];
  buf[index++] = eep.motor.pump_record.accumulated_stroke_count.byte[3];  // 4 bytes

  //Fault history = 1 + 50 bytes
  buf[index++] = eep.history_index;
  for (int i = 0; i < MAX_HISTORY_FAULT_RECORD; i++) {
    buf[index++] = eep.history[i].dnt.year;
    buf[index++] = eep.history[i].dnt.month;
    buf[index++] = eep.history[i].dnt.day;
    buf[index++] = eep.history[i].dnt.hour;
    buf[index++] = eep.history[i].dnt.minute;
    buf[index++] = eep.history[i].dnt.second;
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
  Serial.print("data:");
  for (int i = 0; i < index; i++) {
    Serial.printf("%u ", buf[i]);
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

  // VFD = 7 bytes
  for (int i = 0; i < 7; i++) {
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

  // Injection Pump = 251 bytes
  for (int i = 0; i < 251; i++) {
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
  Serial.print("data:");
  for (int i = 0; i < index; i++) {
    Serial.printf("%u ", buf[i]);
  }
  Serial.print("\n");
#endif
}

//-----------------------------
// writeEEPROMdata()
//-----------------------------
void writeEEPROMdata() {
  bool update_needed = false;
  u16 br;
  u16 bw;

  // if battery voltage is too low probably the pump is powering off, stop recording to avoid data loss, at less than 18V the DC/DC supplying the circuit will not function
  if (syserr.bit.under_voltage_err) {
    //Serial.printf("DC Volt Low:%f V", status.dcv_f);
    //some errors, we should not be here
    strcpy(userMsg[0], "Error: memory update failed.");
    strcpy(userMsg[1], "Reason: power failure!");
    strcpy(userMsg[2], "Input is less than 20V.");
    return;
  }

  // Prevent too frequent flash write operation
  if (millis() - eepCounter < 100) {
    strcpy(userMsg[1], "SAVE CONFIG TOO FAST!");
    Serial.println("writeEEPROM:too frequent, last action is not saved!");  //Monitor via serial port
    eepCounter = millis();
    return;
  }

#if DEBUG_WRITEFLASH_FUNCTION
  Serial.print("writeEEPROM:start\n");  //Monitor via serial port
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
    Serial.println("writeEEPROM: write buf index not equal to read buf index!");  //Monitor via serial port
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
// restoreEEPROMdata()
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

  // VFD = 7 bytes
  eep.ctrl.rpm.byte[0] = rbuf[index++];
  eep.ctrl.rpm.byte[1] = rbuf[index++];
  eep.ctrl.onoff = rbuf[index++];
  eep.ctrl.mode = rbuf[index++];
  eep.ctrl.startup_setting_completed = rbuf[index++];
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

  // Injection Pump = 251 bytes
  eep.ctrl.min_dcv_input.byte[0] = rbuf[index++];
  eep.ctrl.min_dcv_input.byte[1] = rbuf[index++];
  eep.ctrl.max_dcv_input.byte[0] = rbuf[index++];
  eep.ctrl.max_dcv_input.byte[1] = rbuf[index++];
  eep.ctrl.slow_down_dcv.byte[0] = rbuf[index++];
  eep.ctrl.slow_down_dcv.byte[1] = rbuf[index++];
  eep.ctrl.ocp_dcamp.byte[0] = rbuf[index++];
  eep.ctrl.ocp_dcamp.byte[1] = rbuf[index++];
  eep.ctrl.communication_fail_action = rbuf[index++];
  eep.motor.volunit = rbuf[index++];
  eep.motor.dio_mode = rbuf[index++];
  eep.motor.total_pump_head = rbuf[index++];
  eep.motor.tempunit = rbuf[index++];
  eep.motor.temp_ctrl = rbuf[index++];
  eep.motor.off_temp.byte[0] = rbuf[index++];
  eep.motor.off_temp.byte[1] = rbuf[index++];
  eep.motor.on_temp.byte[0] = rbuf[index++];
  eep.motor.on_temp.byte[1] = rbuf[index++];  // 18

  for (u8 i = 0; i < 4; i++) {
    eep.motor.pump[i].calibration_dnt.year = rbuf[index++] + 2000;
    eep.motor.pump[i].calibration_dnt.month = rbuf[index++];
    eep.motor.pump[i].calibration_dnt.day = rbuf[index++];
    eep.motor.pump[i].calibration_dnt.hour = rbuf[index++];
    eep.motor.pump[i].calibration_dnt.minute = rbuf[index++];
    eep.motor.pump[i].calibration_dnt.second = rbuf[index++];  // 6 x 4

    eep.motor.stroke_length[i].all = rbuf[index++];
    eep.motor.piston_diameter[i] = rbuf[index++];
    eep.motor.desired_rate[i].byte[0] = rbuf[index++];
    eep.motor.desired_rate[i].byte[1] = rbuf[index++];
    eep.motor.actual_rate[i].byte[0] = rbuf[index++];
    eep.motor.actual_rate[i].byte[1] = rbuf[index++];
    eep.motor.pump_rate[i].byte[0] = rbuf[index++];
    eep.motor.pump_rate[i].byte[1] = rbuf[index++];
    eep.motor.pump[i].correction_factor.byte[0] = rbuf[index++];
    eep.motor.pump[i].correction_factor.byte[1] = rbuf[index++];
    eep.motor.pump[i].stroke_length.byte[0] = rbuf[index++];
    eep.motor.pump[i].stroke_length.byte[1] = rbuf[index++];  //12 x 4
  }

  eep.motor.stroke_rate.byte[0] = rbuf[index++];
  eep.motor.stroke_rate.byte[1] = rbuf[index++];  //2

  eep.motor.gear_ratio.byte[0] = rbuf[index++];
  eep.motor.gear_ratio.byte[1] = rbuf[index++];  //2

  for (u8 i = 0; i < 24; i++) {
    eep.motor.pump_record.record_date[i].byte[0] = rbuf[index++];
    eep.motor.pump_record.record_date[i].byte[1] = rbuf[index++];
    eep.motor.pump_record.month_stroke_count[i].byte[0] = rbuf[index++];
    eep.motor.pump_record.month_stroke_count[i].byte[1] = rbuf[index++];
    eep.motor.pump_record.month_stroke_count[i].byte[2] = rbuf[index++];
    eep.motor.pump_record.month_stroke_count[i].byte[3] = rbuf[index++];  //144
  }
  eep.motor.pump_record.record_index = rbuf[index++];  //4
  eep.motor.pump_record.hour_stroke_count.byte[0] = rbuf[index++];
  eep.motor.pump_record.hour_stroke_count.byte[1] = rbuf[index++];
  eep.motor.pump_record.hour_stroke_count.byte[2] = rbuf[index++];
  eep.motor.pump_record.hour_stroke_count.byte[3] = rbuf[index++];  //4
  eep.motor.pump_record.day_stroke_count.byte[0] = rbuf[index++];
  eep.motor.pump_record.day_stroke_count.byte[1] = rbuf[index++];
  eep.motor.pump_record.day_stroke_count.byte[2] = rbuf[index++];
  eep.motor.pump_record.day_stroke_count.byte[3] = rbuf[index++];  //4
  eep.motor.pump_record.accumulated_stroke_count.byte[0] = rbuf[index++];
  eep.motor.pump_record.accumulated_stroke_count.byte[1] = rbuf[index++];
  eep.motor.pump_record.accumulated_stroke_count.byte[2] = rbuf[index++];
  eep.motor.pump_record.accumulated_stroke_count.byte[3] = rbuf[index++];  //4

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

  //verify CRC bytes
  if ((rbuf[br - 2] != crcval.byte[0])
      || (rbuf[br - 1] != crcval.byte[1])) {
#if DEBUG_WRITEFLASH_FUNCTION
    Serial.println("restoreEEPROM: failed");                            //Monitor via serial port
    Serial.printf("crcval read: %u %u\n", rbuf[br - 2], rbuf[br - 1]);  //Monitor via serial port
#endif
    // Verification Error, set to default
    eepCounter = millis() + 500;
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
    status.dcv_f = 36.0;  // to skip voltage checking this time for writeEEPROMdata()
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
  // SCADA settings
  scada_setting = eep.scada_setting;
  // SCADA COM port settings
  comport = eep.comport;
  // Injection Pump
  status.motor = eep.motor;

  for (u8 i = 0; i < NO_OF_PUMPS; i++) {
    status.motor.desired_rate_f[i] = (double)status.motor.desired_rate[i].all / 10.0;
    status.motor.actual_rate_f[i] = (double)status.motor.actual_rate[i].all / 10.0;
    status.motor.pump_rate_f[i] = (double)status.motor.pump_rate[i].all / 10.0;
    status.motor.pump[i].correction_factor_f = (double)status.motor.pump[i].correction_factor.all / 10000.0;
    status.motor.stroke_length_inch_f[i] = (double)status.motor.pump[i].stroke_length.all / 100.0;
  }
  status.motor.cal_stroke_rate_f = (double)status.motor.stroke_rate.all / 10.0;
  status.motor.gear_ratio_f = (double)status.motor.gear_ratio.all / 10.0;

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
    if (ctrl.startup_setting_completed == true) {
      ms100Counter++;
      vfdErrCounter++;
      vfdFaultRecordCounter++;
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
        menuctrl.runfunc = false;  // exit any setting function
        // Motor ON/OFF
        if (ctrl.onoff == OFF) {
          if ((ctrl.mode == LOCAL_MODE) || (ctrl.mode == REMOTE_MODE)) {
            if (!syserr.bit.under_voltage_err) {
              ctrl.onoff = ON;
              strcpy(userMsg[1], "SYSTEM ON!");
              updateEEPROM = true;  //save the change
            } else {
              //some errors
              //Serial.printf("DC Volt Low:%f V", status.dcv_f);
              strcpy(userMsg[0], "Error: memory update failed.");
              strcpy(userMsg[1], "Reason: power failure!");
              strcpy(userMsg[2], "Input is less than 20V.");
            }
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
        menuctrl.runfunc = false;  // exit any setting function
        if (ctrl.mode < (2 - status.motor.dio_mode))
          ctrl.mode++;
        else
          ctrl.mode = 0;
        switch (ctrl.mode) {
          case LOCAL_MODE:
            strcpy(userMsg[1], "CTRL MODE:");
            strcpy(userMsg[2], "LOCAL MODE");
            mbServer.start();  // Start ModbusRTU background task
            break;
          case REMOTE_MODE:
            strcpy(userMsg[1], "CTRL MODE:");
            strcpy(userMsg[2], "REMOTE MODE");
            mbServer.start();  // Start ModbusRTU background task
            break;
          case DIO_MODE:
            strcpy(userMsg[1], "CTRL MODE:");
            strcpy(userMsg[2], "DIGITAL IO MODE");
            mbServer.stop();  // Stop ModbusRTU background task
            break;
        }
        updateEEPROM = true;
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
        keyLRhold = false;
      } else if (Key == KEY_ON) {
        keyOnhold = false;
      }
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
  //kp.lockKey(2, 2, 1);     // lock OFF
}

//-------------------------------
// taskScankey
//-------------------------------
void taskScankey(void *parameter) {
  for (;;) {
    Key = kp.getKey();  //scan a key
    if (Key && (ctrl.startup_setting_completed == true)) {
      funcUpdate = true;
      switch (Key) {
        case KEY_ESC:
          break;
        case KEY_MODE:
          break;
        case KEY_UP:
          break;
        case KEY_DOWN:
          break;
        case KEY_LEFT:
          break;
        case KEY_RIGHT:
          break;
        case KEY_ON:
          break;
        case KEY_ENTER:
          break;
        case KEY_OFF:
          menuctrl.runfunc = false;  // exit any setting function
          if (ctrl.onoff == ON) {
            //if DIO mode is on, switch back to local mode
            if (ctrl.mode == DIO_MODE) {
              ctrl.mode = LOCAL_MODE;
              // Restart ModbusRTU background task
              mbServer.start();
            }
            ctrl.onoff = OFF;
            updateEEPROM = true;  //save the change
            strcpy(userMsg[1], "SYSTEM OFF!");
          } else
            strcpy(userMsg[1], "SYSTEM ALREADY OFF!");
          break;
      }

      if ((Key == KEY_ESC) || (Key == KEY_OFF)) {
        //release all keys if one of the unlocked key(s) is pressed
        if (keypadLocked == true) {
          lockKeys(0);
          keypadLocked = false;
        }
      }

      //reset screensaver state and counter
      screenSaverCount = 0;
      protectScreen = false;
    }  //if Key

    // Show first time run setting / menu
    if (ctrl.startup_setting_completed == false) {
      // Display first time run setting
      startupFunc1();
    } else {
      // Display menu
      menuScan(pActiveMenu, &menuctrl, Key);
    }

    vTaskDelay(20 / portTICK_PERIOD_MS);  //20ms
  }
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

//-----------------------------
// taskScreenSaver
//-----------------------------
void taskScreenSaver(void *parameter) {
  for (;;) {
    if (protectScreen) {
      int x;
      int y;
      u8g2.clearBuffer();                    // clear lcd ram buffer
      u8g2.setFont(u8g2_font_t0_12_mf);      // set font
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);  // set color
      x = random(0, 82);
      y = random(12, 52);
      u8g2.drawStr(x, y, "SCREENSAVER (ESC key to exit)");
      LcdUpdate();
    }
    if (ctrl.startup_setting_completed == true) {
      if (screenSaverCount >= SCREEN_SAVER_TIMEOUT) {
        // keypad lock
        if (keypadLocked == false) {
          lockKeysExceptEscOff();
          keypadLocked = true;
        }
        protectScreen = true;
        protectScreenPrev = true;
        screenSaverCount = 0;
      }
      if (menuctrl.runfunc == false) {
        if (!protectScreen) {
          screenSaverCount++;
        }
      } else {
        screenSaverCount = 0;
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  //1000ms
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
  const u8 WDAT_SIZE = 14;
  u16 wdat[WDAT_SIZE];
  u16 index;

  for (;;) {
    index = 0;
    // Polling address 1, server ID 1, 1 word
    Error err = SUCCESS;
    switch (mbTokenCounter) {
      case FC03_SOMI:
        //reg 14 - 23
        err = mbClient.addRequest(FC03_SOMI /*token*/, 1 /*server_id*/, READ_HOLD_REGISTER /*FC*/, 14 /*reg*/, 10 /*count*/);
        debugMbRequest(err);
        break;
      case FC16_MOSI:
        //reg 0 - 13
        //reg 0 heartbeat out ->
        ctrl.count = ms100Counter;
        wdat[index++] = ctrl.count;
        // off during controller reset request
        if (ctrl.reset_ok == true) {
          //reg 1 ON/OFF  out  ->
          wdat[index++] = 0;
        } else {
          //reg 1 ON/OFF  out  ->
          wdat[index++] = ctrl.onoff;
        }
        //reg 2 rpm set ->
        wdat[index++] = ctrl.rpm.all;
        //reg 3 min dcv input set out ->
        wdat[index++] = ctrl.min_dcv_input.all;
        //reg 4 max dcv input set out ->
        wdat[index++] = ctrl.max_dcv_input.all;
        //reg 5 slow down dcv set out ->
        wdat[index++] = ctrl.slow_down_dcv.all;
        //reg 6 OCP DC amp set out ->
        wdat[index++] = ctrl.ocp_dcamp.all;
        //reg 7 Control mode ->
        wdat[index++] = ctrl.mode;
        //reg 8 Communication fail action ->
        wdat[index++] = ctrl.communication_fail_action;
        //reg 9 Off temperature ->
        wdat[index++] = status.motor.off_temp.all;
        //reg 10 On temperature ->
        wdat[index++] = status.motor.on_temp.all;
        //reg 11 Temperature control On/Off + Temperature unit ->
        wdat[index++] = (u16)(((u16)status.motor.temp_ctrl << 1) | (status.motor.tempunit & 0x01));
        //reg 12 Motor Model Number
        wdat[index++] = ctrl.motor_model.all;
        //reg 13 Controller reset
        wdat[index++] = ctrl.reset;
        err = mbClient.addRequest(FC16_MOSI /*token*/, 1 /*server_id*/, WRITE_MULT_REGISTERS /*FC*/, 0 /*reg*/, index /*words*/, index * 2 /*bytes*/, wdat);
        //####################################
        //-REMEMBER to change the wdat[size]!-
        //####################################
        break;
    }
    if (index > WDAT_SIZE) Serial.println("taskMbClient():index > WDAT_SIZE");
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
  Serial.print("\n");
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
      //reg 14 rpm in        <-
      status.rpm.all = rdata[index++];

      //reg 15 DC bus voltage in       <-
      status.dcv.all = rdata[index++];
      status.dcv.all = twos2Int(status.dcv.byte[1], status.dcv.byte[0]);
      status.dcv_f = (double)status.dcv.all / 10.0f;

      //reg 16 DC bus current in      <-
      status.dca.all = rdata[index++];
      status.dca.all = twos2Int(status.dca.byte[1], status.dca.byte[0]);
      status.dca_f = (double)status.dca.all / 10.0f;

      //reg 17 on/off state in      <-
      status.onoff = rdata[index++];

      //reg 18 error
      sysstat.bit.low_voltage_warning = bitRead(rdata[index], 0);
      sysstat.bit.over_current_warning = bitRead(rdata[index], 1);
      syserr.bit.over_current_err = bitRead(rdata[index], 2);
      syserr.bit.over_voltage_err = bitRead(rdata[index], 3);
      syserr.bit.under_voltage_err = bitRead(rdata[index], 4);
      syserr.bit.rpm_set_low_err = bitRead(rdata[index], 5);  //rpm set low err
      syserr.bit.setting_value_err = bitRead(rdata[index], 6);
      syserr.bit.vfd_est_err = bitRead(rdata[index], 7);  //FOC estimator error
      index++;

      //reg 19 Motor coil current in      <-
      status.motora.all = rdata[index++];
      status.motora.all = twos2Int(status.motora.byte[1], status.motora.byte[0]);
      status.motora_f = (double)status.motora.all / 10.0f;

      //reg 20 heartbeat
      status.count = rdata[index++];

      //reg 21 DIO state
      io.all = rdata[index++];
      //DIO control on/off
      if (io.bit.DI1) {
        if ((ctrl.onoff == OFF) && ((ctrl.mode == DIO_MODE))) {
          ctrl.onoff = ON;
        }
      } else {
        if ((ctrl.onoff == ON) && ((ctrl.mode == DIO_MODE))) {
          ctrl.onoff = OFF;
        }
      }

      //reg 22
      status.motor.temp_now.all = rdata[index++];

      //reg 23
      ctrl.reset_ok = rdata[index++];

      //####################################
      //-REMEMBER to change the rdat[size]!-
      //####################################

      if (index > RDAT_SIZE) Serial.println("handleData():index > RDAT_SIZE");
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
//Modbus Server(slave) - FC03
//---------------------------------------------------------------------
//FC03: worker do serve Modbus function code 0x03 request (READ_HOLD_REGISTER)
ModbusMessage FC03(ModbusMessage request) {
  ModbusMessage response;  // response message to be sent back
  u16 scada_max_holdregs_addr;
  u16 address;  // requested register address
  u16 words;    // requested number of registers
  u16 reg_address = 0;
  u16 reg_count = 0;
  u8 cnt = 0;
  u8 index = 0;

  request.get(2, address);  //get register address from request message
  request.get(4, words);    //get total words from request message

  //scada_setting.max_holdreg = 87; //see type_user.h
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
            response.add((u16)dnt.year);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 7) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.month);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 8) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.day);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 9) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.hour);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 10) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.minute);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 11) {
          if (reg_count <= words - 1) {
            response.add((u16)dnt.second);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 12) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.pump_number);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 13) {
          if (reg_count <= words - 1) {
            response.add((u16)status.motor.pump_rate[status.motor.pump_number].all);  //R/W
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        if (reg_address == scada_setting.holdreg_addr.all + 14) {
          if (reg_count <= words - 1) {
            response.add((u16)ctrl.onoff);
            reg_address++;
            reg_count++;
          } else
            return response;
        }
        // word 1: year offset from 2000 << 8 | month
        // word 2: Strokes << 16
        // word 3: Strokes & 0xFFFF
        for (int idx = 0; idx < 72; idx++) {
          //15-86
          if ((reg_address == scada_setting.holdreg_addr.all + 15 + idx)) {
            index = idx / 3;
            cnt = idx % 3;
            //Serial.printf("year:%u month:%u", status.motor.record_date[index].year, status.motor.record_date[index].month);
            if (reg_count <= words - 1) {
              if (cnt == 0) {
                response.add((u16)(status.motor.pump_record.record_date[index].all));  //date
                cnt++;
              } else if (cnt == 1) {
                response.add((u16)(status.motor.pump_record.month_stroke_count[index].all << 16));  //high byte
                cnt++;
              } else if (cnt == 2) {
                response.add((u16)(status.motor.pump_record.month_stroke_count[index].all & 0xFFFF));  //low byte
              }
              reg_address++;
              reg_count++;
            } else
              return response;
          }
        }
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

//---------------------------------------------------------------------
//Modbus Server(slave) - FC06
//---------------------------------------------------------------------
//FC06: worker do serve Modbus function code 0x06 request (WRITE SINGLE REGISTER)
ModbusMessage FC06(ModbusMessage request) {
  ModbusMessage response;  // response message to be sent back
  u16 scada_max_holdregs_addr;
  u16 address;  // requested register address
  u16 words;    // requested number of registers

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

  request.get(2, address);  //get register address from request message
  request.get(4, words);    //get total words from request message

  //scada_setting.max_holdreg = 6; //see type_user.h
  scada_max_holdregs_addr = scada_setting.holdreg_addr.all + scada_setting.max_holdreg - 1;

  // Data address valid?
  if ((address >= scada_setting.holdreg_addr.all) && (address <= scada_max_holdregs_addr)) {
    // Write requested data to requested register
    if (request.getFunctionCode() == WRITE_HOLD_REGISTER) {
      //<!Change Date & Time: year
      if (address == scada_setting.holdreg_addr.all + 6) {
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
      else if (address == scada_setting.holdreg_addr.all + 7) {
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
      else if (address == scada_setting.holdreg_addr.all + 8) {
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
      else if (address == scada_setting.holdreg_addr.all + 9) {
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
      else if (address == scada_setting.holdreg_addr.all + 10) {
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
      else if (address == scada_setting.holdreg_addr.all + 11) {
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
      //<!Change Pump Number
      else if (address == scada_setting.holdreg_addr.all + 12) {
        // Data value valid?
        if ((words >= 1) && (words <= 4)) {
          status.motor.pump_number = words;
          updateEEPROM = true;  //Save setting
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "CHANGE PUMP NUMBER!");
          strcpy(userMsg[2], "RESULT: APPLIED!");
          response = ECHO_RESPONSE;  //Echo response
        } else                       //if the requested data is not in range
        {
          strcpy(userMsg[0], "REMOTE CONTROL:");
          strcpy(userMsg[1], "CHANGE PUMP NUMBER!");
          strcpy(userMsg[2], "RESULT: WRONG VALUE!");
          // Set up error response.
          response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_VALUE);
        }
      }
      //<!Change Pump Rate
      else if (address == scada_setting.holdreg_addr.all + 13) {
        // Data value valid?
        if ((words >= status.motor.min_pump_rate) && (words <= status.motor.max_pump_rate)) {
          status.motor.pump_rate[status.motor.pump_number].all = words;
          status.motor.pump_rate_f[status.motor.pump_number] = (double)status.motor.pump_rate[status.motor.pump_number].all / 10.0;
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

      //<!Change system on/off state
      else if (address == scada_setting.holdreg_addr.all + 14) {
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

  // Are the parameters valid?
  if (coil_addr < mycoils.coils()) {
    // Looks like it. Get the requested coils from our storage
    if (coil_addr == 0) {
      mycoils.set(0, ctrl.onoff);
    } else if (coil_addr == 1) {
      //no variable assigned, the value is itself = mycoils[1], read should be always 0 when FC05 is not active
    } else if (coil_addr == 2) {
      //no variable assigned, the value is itself = mycoils[2], read should be always 0 when FC05 is not active
    }
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
// FC05: worker do serve Modbus function code 0x05 (WRITE SINGLE COIL)
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
  if (coil_addr < mycoils.coils()) {
    if (output_value == 0x0000 || output_value == 0xFF00) {
      if (output_value == 0x0000) {
        value = false;
      } else if (output_value == 0xFF00) {
        value = true;
      }
      //Check if any change is needed
      if (mycoils[coil_addr] == value)  // Is the ON/OFF value correct?
      {
        strcpy(userMsg[0], "REMOTE CONTROL:");
        strcpy(userMsg[1], "SET COILS!");
        strcpy(userMsg[2], "RESULT: NO ACTION!");
        response = ECHO_RESPONSE;
      } else {
        // set the coil value
        if (mycoils.set(coil_addr, value)) {
          // Data handling
          if (coil_addr == 0) {
            if (mycoils[0] == true)
              ctrl.onoff = ON;
            else if (mycoils[0] == false)
              ctrl.onoff = OFF;
            updateEEPROM = true;
            strcpy(userMsg[0], "REMOTE CONTROL:");
            strcpy(userMsg[1], "SYSTEM ON/OFF!");
            strcpy(userMsg[2], "RESULT: APPLIED!");
          } else if (coil_addr == 1) {
            if (mycoils[1] == true) {
              status.motor.pump_record.hour_stroke_count.all = 0;
              status.motor.pump_record.day_stroke_count.all = 0;
              for (u8 i = 0; i < 24; i++) {
                status.motor.pump_record.month_stroke_count[i].all = 0;
              }
              status.motor.pump_record.accumulated_stroke_count.all = 0;
              updateEEPROM = true;
              strcpy(userMsg[0], "REMOTE CONTROL:");
              strcpy(userMsg[1], "STROKE COUNT RESET!");
              strcpy(userMsg[2], "RESULT: APPLIED!");
              mycoils.set(1, false);
            }
          } else if (coil_addr == 2) {
            if (mycoils[2] == true) {
              status.motor.runtime_day = 0;
              status.motor.runtime_hour = 0;
              status.motor.runtime_min = 0;
              status.motor.runtime_sec = 0;
              updateEEPROM = true;
              strcpy(userMsg[0], "REMOTE CONTROL:");
              strcpy(userMsg[1], "RUN TIME RESET!");
              strcpy(userMsg[2], "RESULT: APPLIED!");
              mycoils.set(2, false);
            }
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

//---------------------------------------------------------------------
// checkFile
//---------------------------------------------------------------------
bool checkFile(fs::FS &fs, const char *path) {
  bool ret = false;

  //try to open the file
  File file = fs.open(path);

  if (file) {
    ret = true;  //true if file exists
  }

  return ret;
}

//---------------------------------------------------------------------
// checkFolder
//---------------------------------------------------------------------
bool checkFolder(fs::FS &fs, const char *path) {
  bool ret = false;

  //try to open the folder
  File root = fs.open(path);

  if (!root) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Failed to open directory");
#endif
    return ret;
  }

  if (!root.isDirectory()) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Not a directory");
#endif
    return ret;
  }

  ret = true;  //true if folder exists

  return ret;
}

bool readLine(fs::FS &fs, const char *path, u64 line, char *str, u64 maxbytes, u64 *byteread) {
  bool ret = true;
#if DEBUG_SD_WRITE_FUNCTION
  u32 time1 = 0;
  u32 time2 = 0;
#endif
  u64 line_read = 0;
  u64 cnt = 0;
  *byteread = 0;

#if DEBUG_SD_WRITE_FUNCTION
  Serial.printf("Reading file: %s\n", path);
#endif

  File file = fs.open(path);
  if (!file) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Failed to open file for reading");
#endif
    ret = false;
    return ret;
  }

#if DEBUG_SD_WRITE_FUNCTION
  Serial.print("Read from file:\r\n");
  time1 = millis();
#endif

  while (file.available()) {
    char c = file.read();
    str[cnt++] = c;
    if (c == '\n') {
      line_read++;
      if (line_read == line) {
        str[cnt] = '\0';
        *byteread = cnt;
#if DEBUG_SD_WRITE_FUNCTION
        Serial.printf("Bytes read: %llu\n", *byteread);
        Serial.print(str);
#endif
        break;
      } else {
        cnt = 0;
      }
    }
    if (cnt >= (maxbytes - 1)) {
      *byteread = cnt;
#if DEBUG_SD_WRITE_FUNCTION
      Serial.printf("Bytes read: %llu\n", *byteread);
      Serial.print(str);
#endif
      break;  //length is too long for buffer
    }
  }

#if DEBUG_SD_WRITE_FUNCTION
  time2 = millis();
  Serial.print("Read file in ");
  Serial.print(time2 - time1);
  Serial.print("ms.\n");
#endif

  file.close();

  return ret;
}

bool listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  bool ret = true;

#if DEBUG_SD_WRITE_FUNCTION
  Serial.printf("Listing directory: %s\n", dirname);
#endif

  File root = fs.open(dirname);
  if (!root) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Failed to open directory");
#endif
    ret = false;
    return ret;
  }
  if (!root.isDirectory()) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Not a directory");
#endif
    ret = false;
    return ret;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
#if DEBUG_SD_WRITE_FUNCTION
      Serial.print("  DIR : ");
      Serial.println(file.name());
#endif
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    }
#if DEBUG_SD_WRITE_FUNCTION
    else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
#endif
    file = root.openNextFile();
  }

  return ret;
}

bool createDir(fs::FS &fs, const char *path) {
  bool ret = true;

#if DEBUG_SD_WRITE_FUNCTION
  Serial.printf("Creating Dir: %s\n", path);
#endif
  if (fs.mkdir(path)) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Dir created");
#endif
  } else {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("mkdir failed");
#endif
    ret = false;
  }

  return ret;
}

bool removeDir(fs::FS &fs, const char *path) {
  bool ret = true;

#if DEBUG_SD_WRITE_FUNCTION
  Serial.printf("Removing Dir: %s\n", path);
#endif
  if (fs.rmdir(path)) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Dir removed");
#endif
  } else {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("rmdir failed");
#endif
    ret = false;
  }

  return ret;
}

bool readFile(fs::FS &fs, const char *path) {
  bool ret = true;

#if DEBUG_SD_WRITE_FUNCTION
  Serial.printf("Reading file: %s\n", path);
#endif
  File file = fs.open(path);
  if (!file) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Failed to open file for reading");
#endif
    ret = false;
    return ret;
  }

#if DEBUG_SD_WRITE_FUNCTION
  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
#endif
  file.close();

  return ret;
}

bool writeFile(fs::FS &fs, const char *path, const char *message) {
  bool ret = true;

#if DEBUG_SD_WRITE_FUNCTION
  Serial.printf("Writing file: %s\n", path);
#endif
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Failed to open file for writing");
#endif
    ret = false;
    return ret;
  }
  if (file.print(message)) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("File written");
#endif
  } else {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Write failed");
#endif
    ret = false;
  }
  file.close();

  return ret;
}

bool appendFile(fs::FS &fs, const char *path, const char *message) {
  bool ret = true;

#if DEBUG_SD_WRITE_FUNCTION
  Serial.printf("Appending to file: %s\n", path);
#endif
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Failed to open file for appending");
#endif
    ret = false;
    return ret;
  }
  if (file.print(message)) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Message appended");
#endif
  } else {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Append failed");
#endif
    ret = false;
  }
  file.close();

  return ret;
}

bool renameFile(fs::FS &fs, const char *path1, const char *path2) {
  bool ret = true;

#if DEBUG_SD_WRITE_FUNCTION
  Serial.printf("Renaming file %s to %s\n", path1, path2);
#endif
  if (fs.rename(path1, path2)) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("File renamed");
#endif
  } else {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Rename failed");
#endif
    ret = false;
  }

  return ret;
}

bool deleteFile(fs::FS &fs, const char *path) {
  bool ret = true;

#if DEBUG_SD_WRITE_FUNCTION
  Serial.printf("Deleting file: %s\n", path);
#endif
  if (fs.remove(path)) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("File deleted");
#endif
  } else {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Delete failed");
#endif
    ret = false;
  }

  return ret;
}

//---------------------------------------------------------------------
// runSdDatalogger fsd
//---------------------------------------------------------------------
void runSdDatalogger(fs::FS &fs) {
  // if battery voltage is too low probably the pump is powering off,
  // stop recording to avoid data loss, at less than 18V the DC/DC supplying the circuit will not function
  if (status.dcv_f < 20.0) return;

  //check SD state
  if (!sdState) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("SD not mounted!");
#endif
    return;
  }

  //safely remove SD card
  if (sdEject) {
    return;
  }

  //
  // 1 hour interval recorded to CSV file
  //
  //create new file for each month
  if (newFileflag) {
    sdIsSaving = true;
    char path[13];
    char csv_header[256];
    if (status.motor.volunit == 0) {
      strcpy(csv_header, "Date,Time,Stroke Rate(stroke/min),Day Stroke Count(stroke),Accumulated Stroke Count(stroke)\r\n");
    } else if (status.motor.volunit == 1) {
      strcpy(csv_header, "Date,Time,Stroke Rate(stroke/min),Day Stroke Count(stroke),Accumulated Stroke Count(stroke)\r\n");
    }

    //file path
    xconvert(path, "/%d_%02d.CSV", dnt.year, dnt.month);
    //write CSV header
    if (writeFile(SD, path, csv_header)) {
#if DEBUG_SD_WRITE_FUNCTION
      Serial.println("SD:create new file.");
#endif
    } else {
      if (writeFileflag) writeFileflag = false;  //cancel operation
      sdState = false;                           //SD seems to be not ready
    }
    newFileflag = false;
  }

  //write file every hour
  if (writeFileflag) {
    sdIsSaving = true;
    char path[13];
    char content[16];
    //file path
    xconvert(path, "/%d_%02d.CSV", dnt.year, dnt.month);
#if DEBUG_SD_WRITE_FUNCTION
    Serial.printf("File path:%s\n", path);
#endif
    //check if file exists
    if (checkFile(SD, path)) {
      //write data
      //date yyyy/mm/dd
      xconvert(content, "%04d/%02d/%02d,", dnt.year, dnt.month, dnt.day);
      appendFile(SD, path, content);
      //time hh:mm:ss
      xconvert(content, "%02d:%02d:%02d,", dnt.hour, dnt.minute, dnt.second);
      appendFile(SD, path, content);
      //Stroke Rate(stroke/min)
      xconvert(content, "%.2f,", status.motor.cal_stroke_rate_f);
      appendFile(SD, path, content);
      //Day Stroke Count(stroke)
      xconvert(content, "%d,", status.motor.pump_record.day_stroke_count.all);
      appendFile(SD, path, content);
      //Accumulated Stroke Count(stroke)
      xconvert(content, "%d,", status.motor.pump_record.accumulated_stroke_count.all);
      appendFile(SD, path, content);
      //end of row
      appendFile(SD, path, "\r\n");
      writeFileflag = false;  //wait for next cycle
    } else {
      //create new file
      newFileflag = true;
    }
  }
  sdIsSaving = false;
}

bool mountSD() {
  bool ret = true;

  if (!SD.begin(SD_CS_PIN)) {
#if DEBUG_SD_WRITE_FUNCTION
    Serial.println("Card Mount Failed");
#endif
    strcpy(cardTypeStr, "    ");
    strcpy(cardSizeStr, "             ");
    ret = false;
    return ret;
  } else {
    u8 cardType = SD.cardType();
    if (cardType == CARD_NONE) {
#if DEBUG_SD_WRITE_FUNCTION
      Serial.println("No SD card attached");
#endif
      strcpy(cardTypeStr, "    ");
      strcpy(cardSizeStr, "             ");
      ret = false;
      return ret;
    } else {
#if DEBUG_SD_WRITE_FUNCTION
      Serial.print("SD Card Type: ");
#endif
      if (cardType == CARD_MMC) {
#if DEBUG_SD_WRITE_FUNCTION
        Serial.println("MMC");
#endif
        strcpy(cardTypeStr, "MMC");
      } else if (cardType == CARD_SD) {
#if DEBUG_SD_WRITE_FUNCTION
        Serial.println("SDSC");
#endif
        strcpy(cardTypeStr, "SDSC");
      } else if (cardType == CARD_SDHC) {
#if DEBUG_SD_WRITE_FUNCTION
        Serial.println("SDHC");
#endif
        strcpy(cardTypeStr, "SDHC");
      } else {
        Serial.println("UNKNOWN");
        strcpy(cardTypeStr, "    ");
        strcpy(cardSizeStr, "             ");
        ret = false;
        return ret;
      }
      if (ret) {
        u32 cardSize = (u32)(SD.cardSize() / (1024 * 1024));      //MB
        u32 totalBytes = (u32)(SD.totalBytes() / (1024 * 1024));  //MB
        u32 usedBytes = (u32)(SD.usedBytes() / (1024 * 1024));    //MB
        if (cardSize > 10000) {
          xconvert(cardSizeStr, "%uGB/%uGB", usedBytes / 1024, totalBytes / 1024);
        } else {
          xconvert(cardSizeStr, "%uMB/%uMB", usedBytes, totalBytes);
        }
#if DEBUG_SD_WRITE_FUNCTION
        Serial.printf("SD Card Size: %lluMB\n", cardSize);
        Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
        Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
#endif
      }
    }
  }

  return ret;
}

//=========================================================================================================================================================================
// LcdUpdate
//=========================================================================================================================================================================
void LcdUpdate() {
  if (sdIsSaving == false) {
    u8g2.sendBuffer();  // send to LCD
    u8g2.setFont(u8g2_font_t0_12_mf);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
  }
}

// ferror
void dispError() {
  //error message
  if (syserr.all != 0x00000000) {
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    if (bitRead(syserr.all, errorDisplayCount)) {
      switch (errorDisplayCount) {
        case 0:
          if (!syserr.bit.vfd_heartbeat_err)
            strcpy(errorMsg, "OVPERR");  //DC input over-voltage error
          break;
        case 1:
          if (!syserr.bit.vfd_heartbeat_err)
            strcpy(errorMsg, "UVPERR");  //DC input under-voltage error
          break;
        case 2:
          if (!syserr.bit.vfd_heartbeat_err)
            strcpy(errorMsg, "OCPERR");  //Motor over current error
          break;
        case 3:
          strcpy(errorMsg, "ERR001");  //wrong circulation pump protection settings set by user
          break;
        case 4:
          strcpy(errorMsg, "ERR002");  //invalid data range read from motor controller
          break;
        case 5:
          strcpy(errorMsg, "ERR003");  //HMI lost communication with motor controller
          break;
        case 6:
          if (!syserr.bit.vfd_heartbeat_err)
            strcpy(errorMsg, "ESTERR");  //FOC estimator control error, reason maybe running current over hardware limit, locked rotor etc.
          break;
        case 7:
          if (!syserr.bit.vfd_heartbeat_err)
            strcpy(errorMsg, "RPMERR");  //wrong RPM setting set by user
          errorDisplayCount = 0;
          break;
      }
    }
    u8g2.drawStr(211, MENU_LINE4_YPOS, errorMsg);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    errorDisplayCount++;
    warnDisplayCount = 0;
  } else if ((sysstat.all > 3) && (sysstat.all < 32)) {
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
          strcpy(errorMsg, "REBOOT");  //reboot is pending
          warnDisplayCount = 2;
          break;
      }
    }
    u8g2.drawStr(211, MENU_LINE4_YPOS, errorMsg);
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
    warnDisplayCount++;
    errorDisplayCount = 0;
  } else  //normal running state
  {
    u8g2.setDrawColor(TEXT_COLOR_INVERTED);
    u8g2.drawStr(211, MENU_LINE4_YPOS, "SYSRUN");
    u8g2.setDrawColor(TEXT_COLOR_NORMAL);
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
  // Serial2.begin(SERIAL2_BAUD);  //Serial 2, IO17=TX2, IO16=RX2
  Serial2.begin(SERIAL2_BAUD, SERIAL_8N1, MODBUS_CH1_TX_PIN, MODBUS_CH1_RX_PIN); //Serial 2, IO17=TX2, IO16=RX2
  while (!Serial2) {};          //Wait for Serial 2
  // LCD
  u8g2.begin();
  // External RTC
  if (rtc.begin()) {
    //RTC error
    Serial.println("rtc.begin(): Error!");
    dnt = { 2000, 1, 1, 0, 0, 0 };  //yyyymmdd,hhmmss
  } else {
    //RTC ok
    rtc.useBattery();
    checkTime();
  }
  // SD
  sdState = mountSD();
  /*
    if (rtc.lostPower()) {
    // When time needs to be set on gStr1 new device, or after power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }*/
  // Keypad
  kp.begin(makeKeymap(keys));
  kp.addEventListener(kpEvent);  // Add an event listener.
  kp.setHoldTime(1000);          // Set minimum hold time in msec, i.e. the minimum depress period to be treated as KEY HOLD
  kp.setScanTime(0);             // Set scan rate in msec, set to 0 to be controlled by RTOS task
  kp.setDebounceTime(80);        // Set debounce time for noise immunity, i.e. the minimum depress period to be treated as PRESSED
  // EEPROM
  EEPROM.begin(EEPROM_SIZE);
  delay(10);
  //<! Initialize unknown variables
  ctrl.rpm.all = MAXIMUM_RPM;
  ctrl.min_dcv_input.all = MINIMUM_DCV_INPUT;
  ctrl.max_dcv_input.all = MAXIMUM_DCV_INPUT;
  ctrl.slow_down_dcv.all = SLOW_DOWN_DCV;
  ctrl.ocp_dcamp.all = 100;      // 10.0A
  ctrl.motor_model.byte[0] = 2;  // Motor Model = 50MM
  ctrl.motor_model.byte[1] = 1;  // Board Series = D
  status.rpm.all = 0;
  status.dcv.all = 0;
  status.dca.all = 0;
  status.motora.all = 0;
  for (u8 i = 0; i < NO_OF_PUMPS; i++) {
    status.motor.desired_rate[i].all = 1042;             //104.2
    status.motor.pump_rate[i].all = 1042;                //104.2
    status.motor.actual_rate[i].all = 1042;              //104.2
    status.motor.pump[i].correction_factor.all = 10000;  //1.0
    status.motor.pump[i].calibration_dnt = { 2000, 0, 0, 0, 0, 0 };
    status.motor.pump[i].stroke_length.all = 100;                                                   //1.00"
    status.motor.stroke_length_inch_f[i] = (double)status.motor.pump[i].stroke_length.all / 100.0;  //1.00"
  }
  status.motor.stroke_rate.all = 400;
  status.motor.gear_ratio.all = 100;
  //record table
  status.motor.pump_record.accumulated_stroke_count.all = 0;
  status.motor.pump_record.hour_stroke_count.all = 0;
  status.motor.pump_record.day_stroke_count.all = 0;
  for (u8 i = 0; i < 24; i++) {
    status.motor.pump_record.record_date[i].byte[0] = 0;
    status.motor.pump_record.record_date[i].byte[1] = 0;
    status.motor.pump_record.month_stroke_count[i].all = 0;
  }
  // X100 stored into memory
  status.motor.off_temp.all = -20;
  status.motor.on_temp.all = -10;
  status.motor.temp_now.all = 0;
  io.all = 0;
  sysstat.all = 0;
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
    eep.history[i].err.all = 0;
    eep.history[i].status.all = 0;
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
      u8g2.drawStr(0, MENU_LINE1_YPOS - 6, "ERROR: MEMORY FAIL");
      u8g2.setDrawColor(TEXT_COLOR_NORMAL);
      u8g2.drawStr(0, MENU_LINE2_YPOS - 6, "Error: Memory failed with retry.");
      u8g2.drawStr(0, MENU_LINE3_YPOS - 6, "The user setting data read back");
      u8g2.drawStr(0, MENU_LINE4_YPOS - 6, "were not valid.");
      LcdUpdate();  // Send to display & use size 12 font
      while (1) {}
    }
  } else {
    strcpy(userMsg[1], "READ CONFIG SUCCESS.");
  }

  updateOCLimitByBoardMotor();

  // L/gal Unit
  getVolUnit();
  getTempUnit();
  for (u16 k = 0; k < NO_OF_PUMPS; k++) {
    // init pump setting error
    pseterr[k].all = 0;
  }
  // Calculate stroke rate from known rpm
  updateStrokeRateFromRpm(ctrl.rpm.all);
  COMport1Config();
  // MENU
  pActiveMenu = menu;
  menuInit(pActiveMenu, &menuctrl);
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
  wdtResetCount = millis();
  esp_task_wdt_config_t esp_task_wdt = {
    .timeout_ms = WDT_TIMEOUT,
    .idle_core_mask = 0x3,
    .trigger_panic = true
  };
  esp_task_wdt_init(&esp_task_wdt);       //Enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                //Add current thread to WDT watch
}

//=========================================================================================================================================================================
// loop()
//=========================================================================================================================================================================
void loop() {
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
    //Make sure the below loop dnt.second != lastSec still runs when real time clock failed
    if (rtcFail) {
      dnt.second = 0;
      lastSec++;
    }
    wdtResetCount = millis();
  }
  if (ctrl.startup_setting_completed == true) {
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
  }

  //
  // Secondly Task
  //
  if (dnt.second != lastSec) {
    //
    // Minutely Task
    //
    if (dnt.second == 0) {
      //
      // Hourly Task
      //
      if (dnt.minute == 0) {
        if (sdState) {
          writeFileflag = true;
        }
        //update stroke count every hour
        if (status.motor.pump_record.record_index < 2) {
          //record time
          status.motor.pump_record.record_date[status.motor.pump_record.record_index * 12 + dnt.month - 1].byte[0] = dnt.year - 2000;
          status.motor.pump_record.record_date[status.motor.pump_record.record_index * 12 + dnt.month - 1].byte[1] = dnt.month;
        }
        // save data
        writeEEPROMdata();
        // reset hour stroke
        status.motor.pump_record.hour_stroke_count.all = 0;

        // notify user
        strcpy(userMsg[0], "AUTOSAVE:");
        strcpy(userMsg[1], "STROKE COUNT SAVED!");

        //
        // Daily Task
        //
        if (dnt.hour == 0) {
          // notice user
          strcpy(userMsg[0], "AUTORESET:");
          strcpy(userMsg[1], "DAY STROKE COUNT RESET!");
          //reset every year 20xx/01/01-00:00:00
          if ((dnt.month == 1) && (dnt.day == 1) && (dnt.hour == 0) && (dnt.minute == 0)) {
            //reset year counter at the beginning of year
            //clear hour record 20xx/xx/xx-00:00:00
            status.motor.pump_record.hour_stroke_count.all = 0;
            status.motor.pump_record.day_stroke_count.all = 0;
            if (status.motor.pump_record.record_index > 1) {
              status.motor.pump_record.record_index = 0;
            }
            strcpy(userMsg[0], "AUTORESET:");
            strcpy(userMsg[1], "MONTH STROKE COUNT RESET!");
          }
          //
          //  Monthly Task
          //
          //create new file each month
          u16 lastday = getMaxday(dnt.year, dnt.month);
          if (dnt.day == lastday) {
            //flag to create a new csv file
            newFileflag = true;
            //
            // Yearly Task
            //
            if (dnt.month == 0) {
              //increment year counter
              if (status.motor.pump_record.record_index < 1) {
                status.motor.pump_record.record_index++;
              } else {
                status.motor.pump_record.record_index = 0;
              }
            }
          }
        }
      }
    }

    // if communication is normal
    if (!syserr.bit.vfd_heartbeat_err) {
      if (status.onoff == ON)  //we keep track on the motor on/off state read from controller every second
      {
        status.motor.runtime_sec++;  //motor on sec++
      }
      // if we use 100*seconds/totalsecs to calculate %, it would overflow in a few days
      // u32 = 2,147,483,647 / 100 = 21,474,836.47 s
      // 21,474,836.47 / 3600 = 5,965 h / 24 = 248 days
      //min
      if (status.motor.runtime_sec >= 60) {
        status.motor.runtime_min++;
        //status.motor.runtime_total++;  //++ on 1min base
        status.motor.runtime_sec = 0;
      }
      //hour
      if (status.motor.runtime_min >= 60) {
        status.motor.runtime_hour++;
        status.motor.runtime_min = 0;
      }
      //day
      if (status.motor.runtime_hour >= 24) {
        status.motor.runtime_day++;
        status.motor.runtime_hour = 0;
      }
    }

    // Run Pump Calculations
    if (status.motor.total_pump_head <= NO_OF_PUMPS) {
      for (u8 i = 0; i < status.motor.total_pump_head; i++) {
        updateStrokeCount(i);
      }
    }

    lastSec = dnt.second;
  }

  sysstat.bit.system_onoff_status = ctrl.onoff;
  sysstat.bit.motor_onoff_status = status.onoff;

  if (updateEEPROM == true) {
    updateEEPROM = false;
    writeEEPROMdata();
  }

  //log data on SD card
  runSdDatalogger(SD);
}  //loop()

//--------------------------------------------------------------------------
// NO MORE
//--------------------------------------------------------------------------
