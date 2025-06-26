#ifndef RTD_H_
#define RTD_H_

extern float getTemperatureFloat(uint16_t adcval);
extern int16_t getTemperature16(uint16_t adcval, float sf);
extern int16_t getFfromDegC(int degc);
extern int16_t getCfromF(int f);

#endif /* RTD_H_ */

//===========================================================================
// No more.
//===========================================================================
