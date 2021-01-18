/**
 * @file  rotary_encoder.h
 * @date  18-January-2021
 * @brief Library for using rotary encoders.
 *
 * This library was adaped from https://github.com/mathertel/RotaryEncoder
 * It uses gpio interruption capability and a lookup table to decode 
 * transitions. 
 *
 * @author
 * @author
 */

#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include "Arduino.h"

#define LATCHSTATE 3

class RotaryEncoder
{
public:
  enum class Direction { NOROTATION = 0, CLOCKWISE = 1, COUNTERCLOCKWISE = -1};

  // ----- Constructor -----
  RotaryEncoder(int encA, int encB);
  RotaryEncoder(void){};

  void Begin(int encA, int encB);
  
  // retrieve the current position
  long  getPosition();
  
  // simple retrieve of the direction the knob was rotated at. 0 = No rotation, 1 = Clockwise, -1 = Counter Clockwise
  Direction getDirection();

  // adjust the current position
  void setPosition(long newPosition);

  // retrieve number of pulses detected since last time
  long  getPulses();
  long  readPulses();

  // call this function every some milliseconds or by using an interrupt for handling state changes of the rotary encoder.
  void tick(void);

  // Returns the time in milliseconds between the current observed 
  unsigned long getMillisBetweenRotations() const;

private:
  int _encA, _encB; // Arduino pins used for the encoder. 
  
  volatile int8_t _oldState;
  
  volatile long _position;         // Internal position (4 times _positionExt)
  volatile long _positionPrev;      // Used only for direction checking
  unsigned long _positionPrev2;

  unsigned long _positionExtTime;     // The time the last position change was detected.
  unsigned long _positionExtTimePrev; // The time the previous position change was detected.
};

#endif /* ROTARY_ENCODER_H */
