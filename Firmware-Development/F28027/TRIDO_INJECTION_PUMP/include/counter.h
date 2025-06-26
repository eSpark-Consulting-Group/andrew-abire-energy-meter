//20220307
#ifndef _COUNTER_H_
#define _COUNTER_H_
#include "../include/types.h"//bool

typedef struct _COUNTER_Obj_{
    unsigned int count;
    unsigned int top;
}COUNTER_Obj;

inline void COUNTER_init(COUNTER_Obj* obj, unsigned int taskFreq_Hz, float period_ms){
    obj->top = period_ms / 1E3 * taskFreq_Hz + 0.5;
    obj->count = 0;
}


inline void COUNTER_init_direct(COUNTER_Obj* obj, unsigned int _top){
    obj->top = _top;
    obj->count = 0;
}

inline bool COUNTER_count_SAT(COUNTER_Obj* obj){
    //stop at top
    if(obj->count >= obj->top)
    {
        return true;
    }else{
        ++obj->count;
        return false;
    }
}

inline bool COUNTER_count_CTC(COUNTER_Obj* obj){
    //Clear on Compare Match (CTC) Mode
    if(++obj->count >= obj->top){
        obj->count = 0;
        return true;
    }
    return false;
}


inline void COUNTER_reset(COUNTER_Obj* obj){
    obj->count = 0;
}

#endif /* _COUNTER_H_ */
