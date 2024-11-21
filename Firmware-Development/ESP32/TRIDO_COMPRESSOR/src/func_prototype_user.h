#ifndef FUNC_PROTOTYPE_USER_H
#define FUNC_PROTOTYPE_USER_H

#include <Arduino.h>
char page1Func1(char input);
char page2Func1(char input);
char page3Func1(char input);
char page4Func1(char input);
char page5Func1(char input);
char page6Func1(char input);
char page7Func1(char input);
char page8Func1(char input);
char page9Func1(char input);
char page10Func1(char input);
char page11Func1(char input);
char page12Func1(char input);
char page13Func1(char input);
char page14Func1(char input);
void mbServerRegisterWorkers(void);
void LcdUpdate(void);
void LcdPrepareBuffer(void);
void UpdateSCADAConfig(void);
void UpdateCOMport1Config(void);
void calTimespan(DATETIME_T start,
                 DATETIME_T stop,
                 DATETIME_T *diff);
				 
#endif
