#include "Arduino.h"
#include "rotary_encoder.h"


// The array holds the values for the entries where a position was decremented,
// a 1 for the entries where the position was incremented
// and 0 in all the other (no change or not valid) cases.

const int8_t KNOBDIR[] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0,
};


// positions: [3] 1 0 2 [3] 1 0 2 [3]
// [3] is the positions where my rotary switch detends
// ==> right, count up
// <== left,  count down


// ----- Initialization and Default Values -----

RotaryEncoder::RotaryEncoder(int encA, int encB)
{  
  // Remember Hardware Setup
  this->EncA = encA;
  this->EncB = encB;
  
  // Setup the input pins and turn on pullup resistor
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  // when not started in motion, the current state of the encoder should be 3
  this->OldState = 3;

  // start with position 0;
  this->Counter = 0;
  this->Pulses = 0;
} // RotaryEncoder()

// ----- Initialization and Default Values -----

void RotaryEncoder::Begin(int encA, int encB)
{
  // Remember Hardware Setup
  this->EncA = encA;
  this->EncB = encB;
  
  // Setup the input pins and turn on pullup resistor
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  // when not started in motion, the current state of the encoder should be 3
  this->OldState = 3;

  // start with position 0;
  this->Counter = 0;
  this->Pulses = 0;
} // Begin()


long  RotaryEncoder::readPulses()
{
  noInterrupts(); /* Disabling interuptions */
  this->Pulses = this->Counter;
  this->Counter = 0;
  interrupts();  /* Enabling interuptions */
  return this->Pulses;
} // readPulses()


void RotaryEncoder::tick(void)
{
  int encA = digitalRead(this->EncA);
  int encB = digitalRead(this->EncB);

  if(encA == LOW){
    encA = 0;
  }else{
    encA = 1;
  }

  if(encB == LOW){
    encB = 0;
  }else{
    encB = 1;
  }

  this->ThisState = encB | (encA << 1);
  if (this->OldState != this->ThisState)
  {
    this->Counter += KNOBDIR[this->ThisState | (this->OldState<<2)];   
    this->OldState = this->ThisState;
  } // if
  
} // tick()


// End
