#ifndef _DIGITALINPUT_H_
#define _DIGITALINPUT_H_
#include "types.h"//bool

typedef enum {RisingEdge=0, FallingEdge, RisingAndFallingEdge} DIGITALINPUT_POLARITY;//RisingEdge is active high, FallingEdge is active low

typedef struct _DIGITALINPUT_Obj_{
    unsigned int periodTop;
    unsigned int periodCount;//max 65535
    union{
        struct{
            unsigned char output:1;//1-bit
            unsigned char outputCount:8;//count how many press, 8-bit
            unsigned char qualificationCount:4;//4-bit
            DIGITALINPUT_POLARITY polarity:3;//3-bit
        };
        unsigned char bitfields;
    };
}DIGITALINPUT_Obj;

// period_ms above 0.5ms for 2000Hz
void DIGITALINPUT_init(DIGITALINPUT_Obj* const obj, unsigned int taskFreq_Hz, float qualTime_ms, DIGITALINPUT_POLARITY polarity);
void DIGITALINPUT_run(DIGITALINPUT_Obj* const obj, bool input);

inline void DIGITALINPUT_set_output(DIGITALINPUT_Obj* const obj, bool input){//initialize output for RisingAndFallingEdge
    obj->output = input;
}

inline bool DIGITALINPUT_get_output(DIGITALINPUT_Obj* const obj){//output represent active or not
    return obj->output;
}

inline unsigned char DIGITALINPUT_get_outputCount(DIGITALINPUT_Obj* const obj){
    return obj->outputCount;
}

#endif
