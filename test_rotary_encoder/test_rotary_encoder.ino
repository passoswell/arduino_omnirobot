#include <Wire.h>
#include "rotary_encoder.h"


#define LedPin            LED_BUILTIN
#define LedInterval       500  /*!< LED blink time */
#define EncoderInterval   1000 /*!< Time interval between encoder readings */
#define PulseInterval     1    /*!< Time interval for quadrature signal generation */
#define EncoderPin1       A2 /*!< Input for encoder signal */
#define EncoderPin2       A3 /*!< Input for encoder signal */
#define QuadGenPin1       10 /*!< Output for genersted quadrature signal */
#define QuadGenPin2       11 /*!< Output for genersted quadrature signal */


/**
 * @brief Periodically blinks the built in LED.
 */
void LED_Blink(void);

/**
 * @brief Uses gpio to generate quadrature signal.
 */
void ENCODER_GeneratePulses(void);

/**
 * @brief Reads the number of pulses from encoder.
 */
int ENCODER_Read(void);


// Setup a RoraryEncoder:
RotaryEncoder encoder(EncoderPin1, EncoderPin2);


void setup() {
  Serial.begin(115200);
  Serial.println("Test_Rotary_Encoder");
  pinMode(QuadGenPin1,OUTPUT);
  pinMode(QuadGenPin2,OUTPUT);
  pinMode(LedPin,OUTPUT);
  digitalWrite(QuadGenPin1, HIGH);
  digitalWrite(QuadGenPin2, HIGH);
  digitalWrite(LedPin, LOW);
}

void loop() {

  static int LedState = 0;
  static int LedCounter = 0;

  /* Reading encoder pins and counting pulses */
  encoder.tick();
  /* Generating quadrature signal */
  ENCODER_GeneratePulses();
  /* Periodically reading the encoder pulse counter */
  ENCODER_Read();
  /* Blinking a LED */
  LED_Blink();

}// loop

int ENCODER_Read(void){
  static unsigned long previousMillis = 0, currentMillis;
  static int pos = 0;
  int newPos = 0;

  currentMillis = millis();
  if (currentMillis - previousMillis >= EncoderInterval) {
    previousMillis = currentMillis;
    newPos = encoder.getPulses();
    Serial.println(newPos);
  }
  return(newPos);
}

void ENCODER_GeneratePulses(void){
  static unsigned long previousMillis = 0, currentMillis;
  static int PulseState = 0;
 
  currentMillis = millis();
  if (currentMillis - previousMillis >= PulseInterval) {
    previousMillis = currentMillis;
    
    switch(PulseState){
    case 0:
        digitalWrite(QuadGenPin1, HIGH);
        digitalWrite(QuadGenPin2, HIGH);
        PulseState = 1;
    break;
    case 1:
        digitalWrite(QuadGenPin1, LOW);
        digitalWrite(QuadGenPin2, HIGH);
        PulseState = 2;
    break;
    case 2:
        digitalWrite(QuadGenPin1, LOW);
        digitalWrite(QuadGenPin2, LOW);
        PulseState = 3;
    break;
    case 3:
        digitalWrite(QuadGenPin1, HIGH);
        digitalWrite(QuadGenPin2, LOW);
        PulseState = 0;
    break;
    }//switch
    
  }//if
}//ENCODER_GeneratePulses

void LED_Blink(void){
  static unsigned long previousMillis = 0, currentMillis;
  static int LedState = 0;
    
  currentMillis = millis();
  if (currentMillis - previousMillis >= LedInterval) {
    previousMillis = currentMillis;
      LedState ^= 1;
      if (LedState) {
        digitalWrite(LedPin, HIGH);
      } else {
        digitalWrite(LedPin, LOW);
      }//else 
    }//if
}//LED_Blink
