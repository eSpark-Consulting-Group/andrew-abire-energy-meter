//20210822
#ifndef _LINEARINPUT_H_
#define _LINEARINPUT_H_
#include "../include/filter_fo.h"//input filter
#include "../include/math.h"//_iq

typedef enum{MODE1=0, MODE2, MODE3} LINEARINPUT_MODE;

typedef struct _LINEARINPUT_Obj_{
    _iq output;//should only edit once for each cycle
    _iq min_pu;
    _iq deadBandHi_pu;
    _iq deadBandLow_pu;
    float fullscale;//it define fullscale of the iq form for input, min, mid, deadBandHi, deadBandLow
    FILTER_FO_Obj filter;//Input Filter
    union{
        struct{
            unsigned char direction:1;//1 = forward, 0 = reverse. act on final iq value, need to reset output when direction change
            unsigned char filterEnable:1;
            unsigned char outputBool:1;//Boolean output, threshold is deadBandHi and deadBandLow
            LINEARINPUT_MODE mode:3;//determine how to interpret min mid max
        };
        unsigned char bitfields;
    };
}LINEARINPUT_Obj;

void LINEARINPUT_init(LINEARINPUT_Obj* const obj, bool direction, LINEARINPUT_MODE mode, float min, float mid, float max, float deadBand);

void LINEARINPUT_run(LINEARINPUT_Obj* const obj, const float input);

inline _iq LINEARINPUT_getOutput(LINEARINPUT_Obj* const obj){
    return obj->output;
}

inline bool LINEARINPUT_get_outputBoolean(LINEARINPUT_Obj* const obj){
    return obj->outputBool;
}

void LINEARINPUT_filter_init(LINEARINPUT_Obj* obj, unsigned int taskFreq_Hz, float filterPole_rps);//max filterPole_rps is taskFreq_Hz

inline void LINEARINPUT_setFilterEnable(LINEARINPUT_Obj* obj, bool enable){
    obj->filterEnable = enable;
}




#endif
