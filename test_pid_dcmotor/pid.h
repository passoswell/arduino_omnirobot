#ifndef PID_H
#define PID_H

#include "Arduino.h"

#define LATCHSTATE 3

class PID
{
public:

  // ----- Constructor -----
  PID(float Kp, float Ki, float Kd, float SamplingTime);
  PID(void){};

  void Begin(float Kp, float Ki, float Kd, float SamplingTime);
  
  // Computs PID control signal
  float  Compute(float input, float setpoint);
  
  //  Set PID control signal limits
  void SetLimits(float PIDmax, float PIDmin); 
  
private:
  float _Kp, _Ki, _Kd;     // Internal PID gains
  float _error[3]; 		   // Present and past errors
  float _uk[2];			   // Present and past control signals
  float _coef[3];		   // PID coeficients
  float _PIDmax, _PIDmin;  // Max PWM value
  float _SamplingTime;	   // Sampling time in microseconds
  
};

#endif /* PID_H */

// End
