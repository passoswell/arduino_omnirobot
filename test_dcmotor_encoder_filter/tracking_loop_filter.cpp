#include "tracking_loop_filter.h"



void TrackingLoopFilter_Init(TrackingLoopFilter_t *Parameters, float Kp, float Ki, float Dt)
{
  Parameters->Kp = Kp;
  Parameters->Ki = Ki;
  Parameters->Dt = Dt;
  Parameters->AccError = 0.0;
  Parameters->Output = 0.0;
}



float TrackingLoopFilter_Compute(TrackingLoopFilter_t *Parameters, float Input)
{
  /* Difference between read and filtered accumulated encoder count */
  Parameters->Error = Input - Parameters->Output;
  /* Accumulating the difference */
  if((Input - Parameters->Input) == 0.0 && Parameters->Error == 0.0){
    Parameters->AccError = 0.0;
  }else{
    Parameters->AccError += Parameters->Error;
  }
  /* Computing pulses per second using the accumulator (integral)*/
  Parameters->DiffOutAcc = Parameters->AccError;
  Parameters->DiffOutAcc *= Parameters->Ki * Parameters->Dt;
  /* Computing pulses per second (+ proportional) */
  Parameters->DiffOutput = Parameters->Error;
  Parameters->DiffOutput = (Parameters->Kp * Parameters->DiffOutput) + Parameters->DiffOutAcc;
  /* Computing filtered accumulated encoder count */
  Parameters->Output += Parameters->DiffOutput * Parameters->Dt;
  Parameters->Input = Input;
  
  return Parameters->Output;
}




void TrackingLoopFilterInt_Init(TrackingLoopFilterInt_t *Parameters, float Kp, float Ki, float Dt)
{
  Parameters->Kp = Kp;
  Parameters->Ki = Ki;
  Parameters->Dt = Dt;
  Parameters->AccError = 0;
  Parameters->Output = 0;
}



float TrackingLoopFilterInt_Compute(TrackingLoopFilterInt_t *Parameters, int64_t Input)
{
  /* Difference between read and filtered accumulated encoder count */
  Parameters->Error = Input - Parameters->Output;
  /* Accumulating the difference */
  if((Input - Parameters->Input) == 0 && Parameters->Error == 0){
    Parameters->AccError = 0;
  }else{
    Parameters->AccError += Parameters->Error;
  }
  /* Computing pulses per second using the accumulator (integral)*/
  Parameters->DiffOutAcc = Parameters->AccError;
  Parameters->DiffOutAcc *= Parameters->Ki * Parameters->Dt;
  /* Computing pulses per second (+ proportional) */
  Parameters->DiffOutput = Parameters->Error;
  Parameters->DiffOutput = (Parameters->Kp * Parameters->DiffOutput) + Parameters->DiffOutAcc;
  /* Computing filtered accumulated encoder count */
  Parameters->Output += Parameters->DiffOutput * Parameters->Dt;
  Parameters->Input = Input;
  
  return Parameters->Output;
}
