//20210822
#ifndef _SERIAL_H_
#define _SERIAL_H_
#include "stdio.h"//snprintf()

#include "../include/CircularQueue.h"
#include "../include/hal_obj.h"
#include "../include/main.h"
#include "../include/math.h"
#include "../include/sci.h"
//buffer size for formatted string
#define FORMAT_BUFFER_SIZE 20

extern HAL_Handle halHandle;
typedef struct serial_s{
    struct circularQueue txcq;
    char cStringBuffer[FORMAT_BUFFER_SIZE];//buffer for formatted string
    char receivedByte;
}serial_s;


interrupt void SCIRXINTAISR(void);
interrupt void SCITXINTAISR(void);
void serial_init(HAL_Handle handle);
void serial_loop(void);
void serial_xmit(char);
void serial_msg(char*);
int serial_trafficMax();
void serial_print(const char *_format, ...);//format detail: http://www.cplusplus.com/reference/cstdio/printf/
extern void serial_receiveHandle(char);//receive Handle

#endif
