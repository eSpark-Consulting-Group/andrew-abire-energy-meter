#include "digitalInput.h"

void DIGITALINPUT_init(DIGITALINPUT_Obj* const obj, unsigned int taskFreq_Hz, float qualTime_ms, DIGITALINPUT_POLARITY polarity){
    obj->polarity = polarity;
    if(polarity == FallingEdge){
        obj->output = true;
    }else{
        obj->output = false;
    }
    obj->outputCount = 0;
    obj->periodTop = qualTime_ms / 1E4 * taskFreq_Hz + 0.5;//divide by 10 that is why 1E3 to 1E4
    obj->periodCount = 0;
    obj->qualificationCount = 0;//top is 10, 4-bit
}//DIGITALINPUT_init()



void DIGITALINPUT_run(DIGITALINPUT_Obj* const obj, bool input){
    if(++obj->periodCount >= obj->periodTop){
        obj->periodCount = 0;
        if(obj->polarity == FallingEdge){//RisingEdge is active high, FallingEdge is active low
            input = !input;
        }//from now on input represent active
        if(input != obj->output){//detect transition
            if(++obj->qualificationCount >= 10){//qualification
                obj->qualificationCount = 0;
                obj->output = input;//qualified transition
                if(input || (obj->polarity == RisingAndFallingEdge)){
                    if(obj->outputCount < 255){
                        obj->outputCount++;
                    }
                }
            }
        }else{//glitch
            obj->qualificationCount = 0;
        }
    }
}//DIGITALINPUT_run()


