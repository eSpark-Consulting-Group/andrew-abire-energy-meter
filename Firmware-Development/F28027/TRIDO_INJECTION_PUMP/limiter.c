//20210822
#include "limiter.h"

_iq proportionalRampEven(_iq input, _iq min, _iq max){//max must larger than min
    if(input >= max || input <= -max){
        return _IQ(1);
    }else if(input>min || input<-min){
        return _IQdiv(_IQabs(input) - min, max - min);
    }else{
        return _IQ(0);
    }
}



void LIMITER_init(LIMITER_Obj* const obj, _iq bottom, _iq top, bool direction, bool enable){
    obj->ratio = _IQ(1);
    obj->bottom = bottom;
    obj->top = top;
    obj->direction = direction;//0 = normal, 1 = invert
    obj->limiting = false;
    LIMITER_set_enable(obj, enable);

    //Input Filter
    obj->filter = NULL;
    obj->filterEnable = false;
    obj->filterPresent = false;
//    //default filter parameters are instance step response
//    FILTER_FO_set_a1(obj->filter, 0);
//    FILTER_FO_set_b0(obj->filter, _IQ(1));
//    FILTER_FO_set_b1(obj->filter, 0);
//    FILTER_FO_set_y1(obj->filter, 0);
//    FILTER_FO_set_x1(obj->filter, 0);
}//LIMITER_init()



void LIMITER_run(LIMITER_Obj* const obj, _iq controlValue, _iq* target){
    if(!obj->enable){
        return;
    }else{
        //Input Filter, filter controlValue
        if(obj->filterEnable && obj->filterPresent){
            controlValue = FILTER_FO_run(obj->filter, controlValue);
        }

        if(obj->direction){//0 = normal, 1 = inverted
            obj->ratio = _IQ(1) - proportionalRampEven(controlValue, obj->bottom, obj->top);//inverted limiter, output approach 0 when input approach maximum
        }else{
            obj->ratio = proportionalRampEven(controlValue, obj->bottom, obj->top);//positive limiter, output approach 1 when input approach maximum
        }

        *target = _IQmpy(*target, obj->ratio);

        if(obj->ratio != _IQ(1)){
            obj->limiting = true;
        }else{
            obj->limiting = false;
        }
    }
}



void LIMITER_set_enable(LIMITER_Obj* const obj, bool enable){
    if(!obj->enable && enable){
        obj->enable = enable;
    }else if(obj->enable && !enable){
        obj->enable = enable;
        obj->limiting = false;
        obj->ratio = _IQ(1);
    }
}



bool LIMITER_get_limiting(LIMITER_Obj* const obj){
    return obj->limiting;
}



void LIMITER_filter_init(LIMITER_Obj* obj, FILTER_FO_Obj* filterHandle, unsigned int taskFreq_Hz, float filterPole_rps, bool filterEnable){//max filterPole_rps is taskFreq_Hz
    _iq b0, a1;
    obj->filterEnable = filterEnable;
    obj->filterPresent = true;
    obj->filter = (FILTER_FO_Handle)filterHandle;
    b0 = _IQ((float)filterPole_rps/taskFreq_Hz);//20/1000=0.02
    //clip
    if(b0 > _IQ(1)){
        b0 = _IQ(1);
    }
    a1 = b0 - _IQ(1.0);
    b0 = _IQdiv(b0,_IQ(2));
    FILTER_FO_set_a1(obj->filter, a1);
    FILTER_FO_set_b0(obj->filter, b0);
    FILTER_FO_set_b1(obj->filter, b0);
    FILTER_FO_set_y1(obj->filter, 0);
    FILTER_FO_set_x1(obj->filter, 0);
}


//obsolete code
//output approach 1 when input approach maximum
//_iq ratio(_iq input, _iq min, _iq max){
//  if(input > max){
//    return _IQ(1);
//  }else if (input > min){
//    return _IQdiv((input - min),(max - min));
//  }else{
//    return 0;
//  }
//}

//a wrapper function
//_iq limiter(_iq input, _iq bottom, _iq top, bool direction){
//    if(direction){
//        return ratio(-input, -top, -bottom);//inverted limiter, output approach 0 when input approach maximum
//    }else{
//        return ratio(input, bottom, top);//positive limiter, output approach 1 when input approach maximum
//    }
//}
