#include "main.h"
#include "math.h"
#include "RTD.h"

const float fG =20.60784314L; // gain of opamp
const float fRTOP = 6800.0L; // the top resistor value in ohm, the bottom is RTD sensor
const float fADC_MAX = 4095.0L; // ADC maximum reading
const float fR0 = 100.0L; // resistance at 0¢XC
const float fR100 = 138.51L; // resistance at 100¢XC
const float fRbiasTop = 10000.0L; // the top resistor for V- bias is 10.0k
const float fRbiasBot = 56.0L; // the bottom resistor for V- bias is 56 ohm

//PT100 RTD sensor
//0.003851ohm /¢XC
//£\ = (R100¢XC - R0¢XC) / (R0¢XC * 100.0¢XC)
//R0¢XC = 100.0ohm
//R100¢XC = 138.51ohm
//R=R0¢XC(1+£\T)

float getTemperatureFloat(uint16_t adcval)
{
    float fALPHA = (fR100 - fR0) / (fR0 * 100.0L); // find alpha constant, 0.003851ohm /¢XC in this case
    float vdiff = (float) adcval / fG; // find the differential voltage
    float vminus = fADC_MAX * fRbiasBot /(fRbiasBot + fRbiasTop); // find the V-
    float vplus = vminus + vdiff; // find the V+
    float rt = vplus * fRTOP / (fADC_MAX - vplus); // find RT
    float t = (rt / fR0 - 1) / fALPHA; // calculate temperature

    return t;
}

int16_t getTemperature16(uint16_t adcval, float sf)
{
    float t = getTemperatureFloat(adcval);
    int16_t ret = (int16_t) (t * sf);

    return ret;
}

int16_t getFfromDegC(int16_t degc) {
  float tmp;
  int16_t ret;

  tmp = degc * 1.8 + 32.0;
  ret = (int16_t)tmp;

  return ret;
}

int16_t getCfromF(int16_t f) {
  float tmp;
  int16_t ret;

  tmp = (f - 32.0) / 1.8;
  ret = (int16_t)tmp;

  return ret;
}

//===========================================================================
// No more.
//===========================================================================
