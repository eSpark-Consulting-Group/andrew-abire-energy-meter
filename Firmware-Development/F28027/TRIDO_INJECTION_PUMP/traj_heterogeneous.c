//20210822
#include "traj_heterogeneous.h"


TRAJ_HETEROGENEOUS_Handle TRAJ_HETEROGENEOUS_constructor(void *pMemory,const size_t numBytes)
{
  TRAJ_HETEROGENEOUS_Handle handle;


  if(numBytes < sizeof(TRAJ_HETEROGENEOUS_Obj))
    return((TRAJ_HETEROGENEOUS_Handle)NULL);

  // assign the handle
  handle = (TRAJ_HETEROGENEOUS_Handle)pMemory;

  return(handle);
} // end of TRAJ_HETEROGENEOUS_init() function

void TRAJ_HETEROGENEOUS_init(TRAJ_HETEROGENEOUS_Handle handle, const _iq incDelta, const _iq decDelta, const _iq minValue, const _iq maxValue)
{
  TRAJ_HETEROGENEOUS_Obj *obj = (TRAJ_HETEROGENEOUS_Obj *)handle;
  obj->incDelta = incDelta;
  obj->decDelta = decDelta;
  obj->minValue = minValue;
  obj->maxValue = maxValue;
  obj->targetValue = 0;
  obj->intValue = 0;
  return;
} // end of TRAJ_HETEROGENEOUS_setIntValue() function
