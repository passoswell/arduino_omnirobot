#include "Arduino.h"
#include "rotary_encoder.h"


// The array holds the values for the entries where a position was decremented,
// a 1 for the entries where the position was incremented
// and 0 in all the other (no change or not valid) cases.

const int8_t KNOBDIR[] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
  -1,  0,  0,  1,
0,  1, -1,  0  };


// positions: [3] 1 0 2 [3] 1 0 2 [3]
// [3] is the positions where my rotary switch detends
// ==> right, count up
// <== left,  count down


// ----- Initialization and Default Values -----

RotaryEncoder::RotaryEncoder(int encA, int encB) {
  
  // Remember Hardware Setup
  _encA = encA;
  _encB = encB;
  
  // Setup the input pins and turn on pullup resistor
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  // when not started in motion, the current state of the encoder should be 3
  _oldState = 3;

  // start with position 0;
  _position = 0;
  _positionPrev = 0;
  _positionPrev2 = 0;
} // RotaryEncoder()

// ----- Initialization and Default Values -----

void RotaryEncoder::Begin(int encA, int encB) {
  
  // Remember Hardware Setup
  _encA = encA;
  _encB = encB;
  
  // Setup the input pins and turn on pullup resistor
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);

  // when not started in motion, the current state of the encoder should be 3
  _oldState = 3;

  // start with position 0;
  _position = 0;
  _positionPrev = 0;
  _positionPrev2 = 0;
} // RotaryEncoder()

long  RotaryEncoder::getPosition() {
  return _position;
} // getPosition()

void RotaryEncoder::setPosition(long newPosition) {
  // only adjust the external part of the position.
//  _position = ((newPosition<<2) | (_position & 0x03L));
  _position = newPosition;
  _positionPrev = newPosition;
  _positionPrev2 = newPosition;
} // setPosition()

long  RotaryEncoder::getPulses() {
  long pulses =  _position - _positionPrev2;
  _positionPrev2 = _position;
  return pulses;
} // getPosition()


long  RotaryEncoder::readPulses() {
  long pulses =  _position;
  _position = 0;
  return pulses;
} // readPulses()


RotaryEncoder::Direction RotaryEncoder::getDirection() {

    RotaryEncoder::Direction ret = Direction::NOROTATION;
    long positionDif;
    
    positionDif = (long) _positionPrev - _position;
    
    if( positionDif > 0 ) ret = Direction::COUNTERCLOCKWISE;
    if( positionDif < 0 ) ret = Direction::CLOCKWISE;
    if( positionDif == 0 ) ret = Direction::NOROTATION;
    
    _positionPrev = _position;
    
    return ret;
}



void RotaryEncoder::tick(void)
{
  int encA = digitalRead(_encA);
  int encB = digitalRead(_encB);
  int8_t thisState = encB | (encA << 1);

  if (_oldState != thisState) {
    _position += KNOBDIR[thisState | (_oldState<<2)];
    _positionExtTimePrev = _positionExtTime;
    _positionExtTime = millis();    
    _oldState = thisState;
  } // if
} // tick()

unsigned long RotaryEncoder::getMillisBetweenRotations() const
{
  return _positionExtTime - _positionExtTimePrev; 
}


// End
