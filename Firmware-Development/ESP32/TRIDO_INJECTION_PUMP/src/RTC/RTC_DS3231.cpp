#include "RTClib.h"
#include <Arduino.h>
#include <Wire.h>

#define DS3231_TIME 0x00      ///< Time register
#define DS3231_ALARM1 0x07    ///< Alarm 1 register
#define DS3231_ALARM2 0x0B    ///< Alarm 2 register
#define DS3231_CONTROL 0x0E   ///< Control register
#define DS3231_STATUSREG 0x0F ///< Status register
#define DS3231_TEMPERATUREREG                                                  \
  0x11 ///< Temperature register (high byte - low byte is at 0x12), 10-bit
       ///< temperature value
	   

/**************************************************************************/
/*!
    @brief  Start I2C for the DS3231 and test succesful connection
    @param  wireInstance pointer to the I2C bus
    @return True if Wire can find DS3231 or false otherwise.
*/
/**************************************************************************/
bool RTC_DS3231::testDev() {
  Wire.beginTransmission(DS3231_ADDRESS);
  return Wire.endTransmission();
}

bool RTC_DS3231::begin() {
  Wire.begin();
  Wire.beginTransmission(DS3231_ADDRESS);
  return Wire.endTransmission();
}

void RTC_DS3231::useBattery() {
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(byte(DS3231_CONTROL));
  Wire.write(0x24);
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Check the status register Oscillator Stop Flag to see if the DS3231
   stopped due to power loss
    @return True if the bit is set (oscillator stopped) or false if it is
   running
*/
/**************************************************************************/
bool RTC_DS3231::lostPower(void) {
  uint8_t r = 0;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(byte(DS3231_STATUSREG));
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDRESS, 1);
  while(Wire.available())
  {
    r = Wire.read();
  }
  return r >> 7;
}

/**************************************************************************/
/*!
    @brief  Set the date and flip the Oscillator Stop Flag
    @param dt DateTime object containing the date/time to set
*/
/**************************************************************************/
void RTC_DS3231::adjust(const DateTime &dt) {
  uint8_t buffer[8] = {DS3231_TIME,
                       bin2bcd(dt.second()),
                       bin2bcd(dt.minute()),
                       bin2bcd(dt.hour()),
                       bin2bcd(dowToDS3231(dt.dayOfTheWeek())),
                       bin2bcd(dt.day()),
                       bin2bcd(dt.month()),
                       bin2bcd(dt.year() - 2000U)};

  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(byte(DS3231_STATUSREG));
  Wire.endTransmission();
  Wire.beginTransmission(DS3231_ADDRESS);
  for(int i=0; i<8;i++)
  {
    Wire.write(byte(buffer[i]));
  }
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Get the current date/time
    @return DateTime object with the current date/time
*/
/**************************************************************************/
DateTime RTC_DS3231::now() {
  uint8_t buffer[7];
  int i = 0;
  Wire.beginTransmission(DS3231_ADDRESS);
  Wire.write(byte(0x00));
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDRESS, 7);
  while(Wire.available())
  {
    buffer[i++]= Wire.read();;
  }
  
  return DateTime(bcd2bin(buffer[6]) + 2000U, bcd2bin(buffer[5] & 0x7F),
                  bcd2bin(buffer[4]), bcd2bin(buffer[2]), bcd2bin(buffer[1]),
                  bcd2bin(buffer[0] & 0x7F));
}
