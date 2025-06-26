//20210822
#ifndef _TRAJ_HETEROGENEOUS_H_
#define _TRAJ_HETEROGENEOUS_H_
//the delta for increase and decrease is difference to the original trajectory (TRAJ) object
#include "../include/IQmathLib.h"
#include "../include/types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _TRAJ_HETEROGENEOUS_Obj_
{
  _iq   targetValue;   //!< the target value for the trajectory
  _iq   intValue;      //!< the intermediate value along the trajectory
  _iq   minValue;      //!< the minimum value for the trajectory generator
  _iq   maxValue;      //!< the maximum value for the trajectory generator
  _iq   incDelta;      //!< the delta value for increasing magnitude
  _iq   decDelta;      //!< the delta value for decreasing magnitude
} TRAJ_HETEROGENEOUS_Obj;

typedef struct _TRAJ_HETEROGENEOUS_Obj_ *TRAJ_HETEROGENEOUS_Handle;


extern TRAJ_HETEROGENEOUS_Handle TRAJ_HETEROGENEOUS_constructor(void *pMemory,const size_t numBytes);

void TRAJ_HETEROGENEOUS_init(TRAJ_HETEROGENEOUS_Handle handle, const _iq incDelta, const _iq decDelta, const _iq minValue, const _iq maxValue);

static inline void TRAJ_HETEROGENEOUS_setIntValue(TRAJ_HETEROGENEOUS_Handle handle,const _iq intValue)
{
  TRAJ_HETEROGENEOUS_Obj *obj = (TRAJ_HETEROGENEOUS_Obj *)handle;
  obj->intValue = _IQsat(intValue,obj->maxValue,obj->minValue);//input over range check
  return;
} // end of TRAJ_HETEROGENEOUS_setIntValue() function

static inline void TRAJ_HETEROGENEOUS_setTargetValue(TRAJ_HETEROGENEOUS_Handle handle,const _iq targetValue)
{
  TRAJ_HETEROGENEOUS_Obj *obj = (TRAJ_HETEROGENEOUS_Obj *)handle;
  obj->targetValue = _IQsat(targetValue,obj->maxValue,obj->minValue);//input over range check
  return;
} // end of TRAJ_HETEROGENEOUS_setTargetValue() function

inline int sgn(_iq val) {//the sign function must follow https://en.wikipedia.org/wiki/Sign_function
    return (_IQ(0) < val) - (val < _IQ(0));
}

static inline _iq TRAJ_HETEROGENEOUS_getIntValue(TRAJ_HETEROGENEOUS_Handle handle)
{
  TRAJ_HETEROGENEOUS_Obj *obj = (TRAJ_HETEROGENEOUS_Obj *)handle;

  return(obj->intValue);
} // end of TRAJ_HETEROGENEOUS_getIntValue() function



static inline void TRAJ_HETEROGENEOUS_run(TRAJ_HETEROGENEOUS_Handle handle)
{
  TRAJ_HETEROGENEOUS_Obj *obj = (TRAJ_HETEROGENEOUS_Obj *)handle;
  _iq intValue = obj->intValue;
  int sign_intValue = sgn(intValue);
  int sign_targetValue = sgn(obj->targetValue);

  if(sign_intValue != sign_targetValue && sign_intValue != 0){
    //targetValue is different sign or zero, decrease magnitude toward zero
    intValue += _IQsat(-intValue,obj->decDelta,-obj->decDelta);
  }else{
    //same sign or both zero
    if(_IQabs(obj->targetValue) >= _IQabs(intValue)){
      //magnitude of targetValue >= intValue, increase magnitude
      intValue += _IQsat((obj->targetValue - intValue),obj->incDelta,-obj->incDelta);
    }else{
      //magnitude of targetValue < intValue, decrease magnitude
      intValue += _IQsat((obj->targetValue - intValue),obj->decDelta,-obj->decDelta);
    }
  }
  //bound and store the value
  obj->intValue = _IQsat(intValue,obj->maxValue,obj->minValue);
  return;
} // end of TRAJ_HETEROGENEOUS_run() function

#ifdef __cplusplus
}
#endif // extern "C"

#endif
