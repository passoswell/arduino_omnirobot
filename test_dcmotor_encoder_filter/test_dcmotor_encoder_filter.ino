#include <Wire.h>
#include "tracking_loop_filter.h"
#include "rotary_encoder.h"
#include "dcmotor.h"


#define LedPin            LED_BUILTIN
#define LedInterval       500  /*!< LED blink time */
#define EncoderInterval   20   /*!< Time interval between encoder readings */
#define MotorInterval     20   /*!< Time interval between changes in motor speed */

#define PPR               937 //937.2//Pulser Per Revolution
#define PULSE2RAD         6.70420967e-3 //(2*PI)/PPR
#define WHEEL_RADIUS      2.5 //cm

#define RPM2RAD           M_PI/30.0     /* conversion from RPM to rad/s */
#define RAD2RPM           30.0/M_PI     /* conversion from rad/s to RMP */ 
#define RAD2DEG           180.0/M_PI    /* Radians to degrees */ 
#define DEG2RAD           M_PI/180.0    /* Degrees to radians */


#define SLPpin1           10        /* Enable pin 1 */
#define SLPpin2           11        /* Enable pin 2 */
#define Encoder1PinA      A0        /* Encoder input A for motor 1 */
#define Encoder1PinB      A1        /* Encoder input B for motor 1 */
#define Encoder2PinA      A2        /* Encoder input A for motor 2 */
#define Encoder2PinB      A3        /* Encoder input B for motor 2 */
#define Encoder3PinA      A4        /* Encoder input A for motor 3 */
#define Encoder3PinB      A5        /* Encoder input B for motor 3 */
#define Motor1Dir         7         /* Motor 1 direction output pin */
#define Motor1PWM         6         /* Motor 1 PWM output pin */
#define Motor2Dir         8         /* Motor 2 direction pin */
#define Motor2PWM         9         /* Motor 2 PWM output pin */
#define Motor3Dir         12        /* Motor 3 direction pin */
#define Motor3PWM         3         /* Motor 3 PWM output pin */

/**
 * @brief Periodically blinks the built in LED.
 */
void LED_Blink(void);

/* Setup a RoraryEncoder for pins (A,B): */
RotaryEncoder encoder1(Encoder1PinA, Encoder1PinB);
RotaryEncoder encoder2(Encoder2PinA, Encoder2PinB);
RotaryEncoder encoder3(Encoder3PinA, Encoder3PinB);

/* Setup of motors (pinPWM, pinDir): */
DcMotor motor1(Motor1PWM, Motor1Dir);
DcMotor motor2(Motor2PWM, Motor2Dir);
DcMotor motor3(Motor3PWM, Motor3Dir);


TrackingLoopFilterInt_t EncoderFilter;


void setup() {
  Serial.begin(115200);
  Serial.println("Test_Rotary_Encoders_Motors");
  pinMode(SLPpin1,OUTPUT);
  digitalWrite(SLPpin1, HIGH);
  pinMode(SLPpin2,OUTPUT);
  digitalWrite(SLPpin2, HIGH);
  pinMode(LedPin,OUTPUT);
  digitalWrite(LedPin, HIGH);

  TrackingLoopFilterInt_Init(&EncoderFilter, 40.0, 900.0, (float) EncoderInterval / 1000.0);

  /*
   * The code below configures interruption on ATMEGA328p gpio pins,
   * which makes it incompatible with Arduino standard, and will
   * work only with A2 and A3 pins on Arduino UNOs. You must
   * read the datasheet or search for tutorials if using another
   * board and / or another pins.
   */
  PCICR |= (1 << PCIE1);    /* This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C. */
  PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);  /* This enables the interrupt for pin 0 and 1 of Port C. */
  PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);  /* This enables the interrupt for pin 2 and 3 of Port C. */
  PCMSK1 |= (1 << PCINT12) | (1 << PCINT13);  /* This enables the interrupt for pin 4 and 5 of Port C. */

}

/**
 * @brief The Interrupt Service Routine for Pin Change Interrupt 1.
 *        This routine will only be called on any signal change on 
 *        pins A0, A1, A2, A3, A4 and A5 from Arduin UNO: exactly where 
 *        we need to check.
 *        This code is outside of Arduino standard, and will
 *        work only with pins A0, A1, A2, A3, A4 and A5 on Arduino UNOs. You must
 *        read the datasheet or search for tutorials if using another
 *        board and / or another pins.
 */
ISR(PCINT1_vect) {
  encoder1.tick(); // just call tick() to check the state.
  encoder2.tick();
  encoder3.tick();
}

void loop() {

  static unsigned long previousMillis = 0, currentMillis;
  static long pulses;
  static int64_t tpulses = 0;
  static unsigned int printCounter = 0;
  float dt = 0.0;

/*
  Ler encoder (pulsos)
  Calcular erro de posição (pulsos reais - estimados)
  Usar PI para estimar velocidade (pulsos/tempo)
  integrar velocidade para calcular próxima posição estimada
*/
  currentMillis = millis();
  if (currentMillis - previousMillis >= EncoderInterval) {
    previousMillis = currentMillis;
    dt = (float) EncoderInterval / 1000.0;

    /* Reading encoder counter */
    pulses = encoder1.readPulses();

    /* Accumulating encoder pulses over time */
    tpulses += pulses;

    TrackingLoopFilterInt_Run(&EncoderFilter, tpulses);
    

    printCounter++;
    if(printCounter >= (500 / EncoderInterval))
    {
      printCounter = 0;
    
      /*Encoder counter */
      Serial.print(pulses);
      Serial.print("   ");
      Serial.print((int32_t)EncoderFilter.Error);
      Serial.print("   ");
      Serial.print((int32_t)EncoderFilter.AccError);
      Serial.print("   ");
      /* Unfiltered wheel speed in RPM */
      Serial.print(((float)pulses * PULSE2RAD * RAD2RPM) / dt);
      Serial.print("                       ");
      Serial.print("   ");
      /* Pulses per second filter output 1 */
      Serial.print(EncoderFilter.DiffOutput * PULSE2RAD * RAD2RPM);
      Serial.print("   ");
      /* Pulses per second filter output 2 */
      Serial.print(EncoderFilter.DiffOutAcc * PULSE2RAD * RAD2RPM);
      Serial.print("   ");
      Serial.println();
    }

    
    motor1.set(100);//turn clockwise with PWM = 100
    pulses = encoder2.getPulses();
    //Serial.print(pulses);
    //Serial.print(" ");
    motor2.set(0);
    pulses = encoder3.getPulses();
    //Serial.println(pulses);
    motor3.set(0);
  }//if
  
  /* Blinking a LED */
  LED_Blink();

}// loop

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
