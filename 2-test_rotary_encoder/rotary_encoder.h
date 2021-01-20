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

  // retrieve number of pulses detected since last time
  long  readPulses();

  // call this function every some milliseconds or by using an interrupt for handling state changes of the rotary encoder.
  void tick(void);

  long get_Counter(void){return this->Counter;};
  long get_Pulses(void){return this->Pulses;};
  long get_State(void){return this->ThisState | (this->OldState<<2);};

private:
  int EncA, EncB; // Arduino pins used for the encoder. 
  
  volatile int8_t ThisState, OldState;
  
  volatile long Counter, Pulses;         // Internal position (4 times CounterExt)
};

#endif /* ROTARY_ENCODER_H */
