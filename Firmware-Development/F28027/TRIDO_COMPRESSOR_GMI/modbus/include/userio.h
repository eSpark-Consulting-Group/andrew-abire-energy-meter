#ifndef USERIO_H_
#define USERIO_H_

#include "gpio.h"

//Digital Input
//DI1
#define DI1_PIN  GPIO_Number_16
#define DI1_MODE GPIO_16_Mode_GeneralPurpose
//DI2
#define DI2_PIN  GPIO_Number_17
#define DI2_MODE GPIO_17_Mode_GeneralPurpose
//DI3
#define DI3_PIN  GPIO_Number_19
#define DI3_MODE GPIO_19_Mode_GeneralPurpose
//DI4
#define DI4_PIN  GPIO_Number_18
#define DI4_MODE GPIO_18_Mode_GeneralPurpose

//Digital Output
//DO1
#define DO1_PIN     GPIO_Number_33
#define DO1_MODE    GPIO_33_Mode_GeneralPurpose
#define DO1_CLOSE() GPIO_setLow(myGpio, DO1_PIN)
#define DO1_OPEN()  GPIO_setHigh(myGpio, DO1_PIN)
//DO2
#define DO2_PIN     GPIO_Number_32
#define DO2_MODE    GPIO_32_Mode_GeneralPurpose
#define DO2_CLOSE() GPIO_setLow(myGpio, DO2_PIN)
#define DO2_OPEN()  GPIO_setHigh(myGpio, DO2_PIN)
//DO3
#define DO3_PIN     GPIO_Number_0
#define DO3_MODE    GPIO_0_Mode_GeneralPurpose
#define DO3_CLOSE() GPIO_setLow(myGpio, DO3_PIN)
#define DO3_OPEN()  GPIO_setHigh(myGpio, DO3_PIN)
//DO4
#define DO4_PIN     GPIO_Number_1
#define DO4_MODE    GPIO_1_Mode_GeneralPurpose
#define DO4_CLOSE() GPIO_setLow(myGpio, DO4_PIN)
#define DO4_OPEN()  GPIO_setHigh(myGpio, DO4_PIN)

#define DIR_CW      1
#define DIR_CCW     0

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

void InitGpios(void);
void readIOstates(void);
void setDigitalOutput(void);

extern GPIO_Handle myGpio;

#endif /* USERIO_H_ */
