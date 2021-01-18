#ifndef DcMotor_h
#define DcMotor_h

#include "Arduino.h"

#define LATCHSTATE 3

class DcMotor
{
public:
  enum class Direction { NOROTATION = 0, CLOCKWISE = 1, COUNTERCLOCKWISE = -1};

  // ----- Constructor -----
  DcMotor(int pinPWM, int pinDir);		//Constructor
  DcMotor(int pinPWM, int pinDir, int nbits);  //also configures max PWM value
  
  // retrieve the current position
  void  set(unsigned int PWMvalue, int _direction);	//Sets PWM value directly
  void  set(int PWMvalue, int _direction);
  void  set(float PWMvalue);
  //void  set(float dutycycle, int direction);//Uses duty cycle to compute PWM value
  
  
private:
  int _pinPWM, _pinDir;    // Arduino pins used for the encoder. 
  int _PWM_MaxValue = 255;  // Max PWM value
  
};

#endif

// End
