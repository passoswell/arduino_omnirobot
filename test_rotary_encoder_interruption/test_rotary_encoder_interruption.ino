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

  /*
   * The code below configures interruption on ATMEGA328p gpio pins,
   * which makes it incompatible with Arduino standard, and will
   * work only with A2 and A3 pins on Arduino UNOs. You must
   * read the datasheet or search for tutorials if using another
   * board and / or another pins.
   */
  PCICR |= (1 << PCIE1);    /* This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C. */
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  /* This enables the interrupt for pin 2 and 3 of Port C.*/

}

/**
 * @brief The Interrupt Service Routine for Pin Change Interrupt 1.
 *        This routine will only be called on any signal change on 
 *        A2 and A3 (PC2 and PC3 of ATMEGA328 chip): exactly where 
 *        we need to check.
 *        This code is outside of Arduino standard, and will
 *        work only with A2 and A3 pins on Arduino UNOs. You must
 *        read the datasheet or search for tutorials if using another
 *        board and / or another pins.
 */
 ISR(PCINT1_vect) {
  encoder.tick(); // just call tick() to check the state.
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
    newPos = encoder.readPulses();
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
