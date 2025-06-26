//20210822
#ifndef _LIMITER_H_
#define _LIMITER_H_
#include "../include/filter_fo.h"//input filter
#include "../include/math.h"//_iq

typedef struct _LIMITER_Obj_{
    _iq ratio;
    _iq bottom;
    _iq top;
    FILTER_FO_Handle filter;//filter controlValue
    union{
        struct{
            unsigned char enable:1;
            unsigned char direction:1;//0 = normal, 1 = inverted
            unsigned char limiting:1;
            unsigned char filterEnable:1;
            unsigned char filterPresent:1;
        };
        unsigned char bitfields;
    };
}LIMITER_Obj;


void LIMITER_init(LIMITER_Obj* const obj, _iq bottom, _iq top, bool direction, bool enable);//direction: 0 = normal, 1 = inverted
void LIMITER_run(LIMITER_Obj* const obj, _iq controlValue, _iq* target);
void LIMITER_set_enable(LIMITER_Obj* const obj, bool enable);//config limiter_Obj.enable
bool LIMITER_get_limiting(LIMITER_Obj* const obj);

//input filter
void LIMITER_filter_init(LIMITER_Obj* obj, FILTER_FO_Obj* filterHandle, unsigned int taskFreq_Hz, float filterPole_rps, bool filterEnable);//max filterPole_rps is taskFreq_Hz
//inline void LIMITER_setFilterEnable(LIMITER_Obj* obj, bool enable){
//    obj->filterEnable = enable && obj->filterPresent;
//}


#endif
