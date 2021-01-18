#include "Arduino.h"
#include "dcmotor.h"

// ----- Initialization and Default Values -----


DcMotor::DcMotor(int pinPWM, int pinDir){
	_pinPWM = pinPWM;
	_pinDir = pinDir;
	pinMode( _pinPWM, OUTPUT );
	digitalWrite(_pinPWM, LOW);
	pinMode(_pinDir,OUTPUT);
	digitalWrite(_pinDir, HIGH);
}


DcMotor::DcMotor(int pinPWM, int pinDir, int nbits){
	_pinPWM = pinPWM;
	_pinDir = pinDir;
	_PWM_MaxValue = (1<<nbits) - 1;
	pinMode( _pinPWM, OUTPUT );
	digitalWrite(_pinPWM, LOW);
	pinMode(_pinDir,OUTPUT);
	digitalWrite(_pinDir, HIGH);
}


void DcMotor::Begin(int pinPWM, int pinDir){
  _pinPWM = pinPWM;
  _pinDir = pinDir;
  pinMode( _pinPWM, OUTPUT );
  digitalWrite(_pinPWM, LOW);
  pinMode(_pinDir,OUTPUT);
  digitalWrite(_pinDir, HIGH);
}

void  DcMotor::Set(unsigned int PWMvalue, int _direction){
	analogWrite(_pinPWM,PWMvalue&_PWM_MaxValue);
	if( _direction&&1 ) digitalWrite(_pinDir, HIGH);
	else digitalWrite(_pinDir, LOW);
}

void  DcMotor::Set(int PWMvalue, int _direction){
  if(PWMvalue>0) analogWrite(_pinPWM,PWMvalue&_PWM_MaxValue);
  else analogWrite(_pinPWM,(0-PWMvalue)&_PWM_MaxValue);
  if( _direction&&1 ) digitalWrite(_pinDir, HIGH);
  else digitalWrite(_pinDir, LOW);
}

void  DcMotor::Set(float PWMvalue){
  int aux = (int)abs(PWMvalue);
  analogWrite(_pinPWM,aux&_PWM_MaxValue);
  if( PWMvalue >= 0.0 ) digitalWrite(_pinDir, LOW);
  else digitalWrite(_pinDir, HIGH);
}
