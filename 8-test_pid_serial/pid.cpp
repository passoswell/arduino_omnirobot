#include "Arduino.h"
#include "pid.h"

// ----- Initialization and Default Values -----


PID::PID(float Kp, float Ki, float Kd, float SamplingTime){
	_Kp = Kp;
	_Ki = Ki;
	_Kd = Kd;
	_SamplingTime = SamplingTime;
	_error[0] = 0.0;
	_error[1] = 0.0;
	_error[2] = 0.0;
	_uk[0] = 0.0;
	_uk[1] = 0.0;	
	_PIDmax = 0.0;
	_PIDmin = 0.0;
	_coef[0] = _Kp + _Ki*(_SamplingTime/1000.0) + (_Kd/(_SamplingTime/1000.0));
	_coef[1] = 0.0 - _Kp - 2*(_Kd/(_SamplingTime/1000.0));
	_coef[2] = _Kd/(_SamplingTime/1000.0);
  _maxError = 0.0;
} 


void PID::Begin(float Kp, float Ki, float Kd, float SamplingTime){
 _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _SamplingTime = SamplingTime;
  _error[0] = 0.0;
  _error[1] = 0.0;
  _error[2] = 0.0;
  _uk[0] = 0.0;
  _uk[1] = 0.0; 
  _PIDmax = 0.0;
  _PIDmin = 0.0;
  _coef[0] = _Kp + _Ki*(_SamplingTime/1000.0) + (_Kd/(_SamplingTime/1000.0));
  _coef[1] = 0.0 - _Kp - 2*(_Kd/(_SamplingTime/1000.0));
  _coef[2] = _Kd/(_SamplingTime/1000.0);
  _maxError = 0.0;
} 


float  PID::Compute(float input, float setpoint){
	_error[0] = setpoint - input;
  if(setpoint == 0.0 && _error[0] == 0.0){
    _uk[0] = 0.0;
  }else if(abs(_error[0]) < _maxError){
    _uk[0] = _uk[1];
  }else{
	  _uk[0] = _uk[1];
  	_uk[0] += _coef[0]*_error[0];
  	_uk[0] += _coef[1]*_error[1];
  	_uk[0] += _coef[2]*_error[2];
  	if( _uk[0] < _PIDmin ){ _uk[0] = _PIDmin;}
  	if( _uk[0] > _PIDmax ){ _uk[0] = _PIDmax;}
  }
	_uk[1] = _uk[0];
  _error[2] = _error[1];
  _error[1] = _error[0];
//  _error[0] = setpoint - input;
//  _error[1] += _error[0];
//  _uk[0] = _Kp*_error[0];
//  _uk[0] += (_Ki*_error[1])*(_SamplingTime/1000.0);
//  if( _uk[0] < _PIDmin ) _uk[0] = _PIDmin;
//  else if( _uk[0] > _PIDmax ) _uk[0] = _PIDmax;
	return _uk[0];
}


void PID::SetLimits(float PIDmax, float PIDmin){
  if( PIDmax > PIDmin){
  	_PIDmax = PIDmax;
  	_PIDmin = PIDmin;
  }else{
    _PIDmax = PIDmin;
    _PIDmin = PIDmax;
  }
}

void PID::SetMaxError(float MaxError){
  _maxError = MaxError;
}
