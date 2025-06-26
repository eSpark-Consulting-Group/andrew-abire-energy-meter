//20210822
#include "linearInput.h"

//output approach 1 when input approach maximum
_iq ratio(_iq input, _iq min, _iq max){
  if(input > max){
    return _IQ(1);
  }else if (input > min){
    return _IQdiv((input - min),(max - min));
  }else{
    return 0;
  }
}

void LINEARINPUT_init(LINEARINPUT_Obj* const obj, bool direction, LINEARINPUT_MODE mode, float min, float mid, float max, float deadBand){
    _iq deadBandNum = _IQ(deadBand * (max - min) / max);//deadBand is the portion of whole range (max - min), 0 to 1
    _iq mid_pu = _IQ(mid/max);
    obj->min_pu = _IQ(min/max);
    obj->fullscale = max;
    obj->output = 0;
    obj->bitfields = 0;
    obj->mode = mode;
    obj->direction = direction;
    obj->outputBool = false;

    switch(obj->mode){
    case MODE1:
        obj->deadBandHi_pu = mid_pu + deadBandNum;
        obj->deadBandLow_pu = mid_pu - deadBandNum;
        break;
    case MODE2:
        obj->deadBandHi_pu = _IQ(1) - deadBandNum;//max - deadBandNum
        obj->deadBandLow_pu = obj->min_pu + deadBandNum;
        break;
    }

    //Input Filter
    obj->filterEnable = false;
    //default filter parameters are instance step response
    FILTER_FO_set_a1((FILTER_FO_Handle)&obj->filter, 0);
    FILTER_FO_set_b0((FILTER_FO_Handle)&obj->filter, _IQ(1));
    FILTER_FO_set_b1((FILTER_FO_Handle)&obj->filter, 0);
    FILTER_FO_set_y1((FILTER_FO_Handle)&obj->filter, 0);//last output
    FILTER_FO_set_x1((FILTER_FO_Handle)&obj->filter, 0);//last input
}



void LINEARINPUT_run(LINEARINPUT_Obj* const obj, const float input){
    _iq tempOutput;
    _iq tempInput = _IQ(input / obj->fullscale);

    //Input Filter
    if(obj->filterEnable){
        tempInput = FILTER_FO_run((FILTER_FO_Handle)&obj->filter, tempInput);
    }else{
        FILTER_FO_setInitialConditions((FILTER_FO_Handle)&obj->filter,tempInput,tempInput);
    }

    //map linear input to per unit scale. the operation is float since input is larger than 127
    switch(obj->mode)
    {
    case MODE1://with middle point. output -1 to 1
        if(tempInput > obj->deadBandHi_pu){
            tempOutput = ratio(tempInput, obj->deadBandHi_pu, _IQ(1));
        }else if(tempInput < obj->deadBandLow_pu){
            tempOutput = -ratio(-tempInput, -obj->deadBandLow_pu, -obj->min_pu);
        }else{
            tempOutput = 0;
        }
        break;
    case MODE2://end to end. output 0 to 1
        if(tempInput > obj->deadBandHi_pu){
            tempOutput = _IQ(1);
        }else if(tempInput > obj->deadBandLow_pu){
            tempOutput = ratio(tempInput, obj->deadBandLow_pu, obj->deadBandHi_pu);
        }else{
            tempOutput = 0;
        }
        break;
    }

    //boolean output
    if(!obj->outputBool && (tempInput > obj->deadBandHi_pu)){//rising edge
        obj->outputBool = true;
    }else if(obj->outputBool && (tempInput < obj->deadBandLow_pu)){//falling edge
        obj->outputBool = false;
    }

    //direction
    if(!obj->direction){//reverse output
        tempOutput = -tempOutput;
    }

    //final output
    obj->output = tempOutput;
}



void LINEARINPUT_filter_init(LINEARINPUT_Obj* obj, unsigned int taskFreq_Hz, float filterPole_rps){//max filterPole_rps is taskFreq_Hz
    _iq b0, a1;
    b0 = _IQ((float)filterPole_rps/taskFreq_Hz);//20/1000=0.02
    //clip
    if(b0 > _IQ(1)){
        b0 = _IQ(1);
    }
    a1 = b0 - _IQ(1);
    b0 = _IQdiv(b0,_IQ(2));
    FILTER_FO_set_a1((FILTER_FO_Handle)&obj->filter, a1);
    FILTER_FO_set_b0((FILTER_FO_Handle)&obj->filter, b0);
    FILTER_FO_set_b1((FILTER_FO_Handle)&obj->filter, b0);
    FILTER_FO_set_y1((FILTER_FO_Handle)&obj->filter, 0);
    FILTER_FO_set_x1((FILTER_FO_Handle)&obj->filter, 0);
}



