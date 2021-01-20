#include <Wire.h>
#include "rotary_encoder.h"
#include "tracking_loop_filter.h"
#include "dcmotor.h"
#include "pid.h"


#define N_MOTORS          3


#define LedPin            LED_BUILTIN
#define LedInterval       500  /*!< LED blink time */
#define EncoderInterval   20   /*!< Time interval between encoder readings */
#define MotorInterval     20   /*!< Time interval between changes in motor speed */

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

/* PID gains for the motor controller */
#define Kpm               0.032547     /* PID proportional gain for motor control */
#define Kim               0.1//0.32631 /* PID integral gain for motor control */
#define Kdm               0.0          /* PID derivative gain for motor control */


//Helpful constants
#define MOTOR_PPR         937           //Pulses per revolution for the encoder
#define VOLTAGE2PWM       21.25       //PWM_MAX/VoutMax
#define RPM2RAD           PI/30.0     //conversion from RPM to rad/s
#define RAD2RPM           30.0/PI     //conversion from rad/s to RMP
#define PULSE2RAD         (2 * PI / MOTOR_PPR)   //6.70420967e-3 //(2*PI)/PPR
#define PULSES2RPM        3.201707577 //((1/MOTOR_PPR)/(encoderInterval/1000))*60

/**
 * @brief Periodically blinks the built in LED.
 */
void LED_Blink(void);

/* Setup a RoraryEncoder for pins (A,B): */
RotaryEncoder Encoder[N_MOTORS];


TrackingLoopFilterInt_t EncoderFilter[N_MOTORS];

/* Setup of motors (pinPWM, pinDir): */
DcMotor Motor[N_MOTORS];

/* Setup of PID controllers (Kp, Ki, Kd, SamplingTime): */
PID PIDx[N_MOTORS];

float MotorsSpeedSetpoint[N_MOTORS] = {50.0,50.0,-50.0};
float MotorsSpeedMeasured[N_MOTORS];
float PIDvalue[N_MOTORS] = {0};
float PWMvalue[N_MOTORS];


void setup() {
  Serial.begin(115200);
  Serial.println("Test_Rotary_Encoders_Motors");
  pinMode(SLPpin1,OUTPUT);
  digitalWrite(SLPpin1, HIGH);
  pinMode(SLPpin2,OUTPUT);
  digitalWrite(SLPpin2, HIGH);
  pinMode(LedPin,OUTPUT);
  digitalWrite(LedPin, HIGH);

  Encoder[0].Begin(Encoder1PinA, Encoder1PinB);
  Encoder[1].Begin(Encoder2PinA, Encoder2PinB);
  Encoder[2].Begin(Encoder3PinA, Encoder3PinB);

  TrackingLoopFilterInt_Init(&EncoderFilter[0], 40.0, 900.0, (float) EncoderInterval / 1000.0);
  TrackingLoopFilterInt_Init(&EncoderFilter[1], 40.0, 900.0, (float) EncoderInterval / 1000.0);
  TrackingLoopFilterInt_Init(&EncoderFilter[2], 40.0, 900.0, (float) EncoderInterval / 1000.0);

  
  PIDx[0].Begin(Kpm, Kim, Kdm, EncoderInterval);
  PIDx[0].SetLimits(400, -400);
  PIDx[1].Begin(Kpm, Kim, Kdm, EncoderInterval);
  PIDx[1].SetLimits(400, -400);
  PIDx[2].Begin(Kpm, Kim, Kdm, EncoderInterval);
  PIDx[2].SetLimits(400, -400);
  
  Motor[0].Begin(Motor1PWM, Motor1Dir);
  Motor[0].Set(0, 1);
  Motor[1].Begin(Motor2PWM, Motor2Dir);
  Motor[1].Set(0, 1);
  Motor[2].Begin(Motor3PWM, Motor3Dir);
  Motor[2].Set(0, 1);
  

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
  Encoder[0].tick(); // just call tick() to check the state.
  Encoder[1].tick();
  Encoder[2].tick();
}

void loop() {

  static unsigned long previousMillis = 0, currentMillis;
  static unsigned int printCounter = 0;
  static long pulses;
  static int64_t tpulses[3] = {0,0,0};
  float dt = 0.0;

  currentMillis = millis();
  if (currentMillis - previousMillis >= EncoderInterval)
  {
    previousMillis = currentMillis;
    dt = (float) EncoderInterval / 1000.0;

    printCounter++;
    for(int i = 0; i < N_MOTORS; i++)
    {      
      /* Motor control */
      pulses = Encoder[i].readPulses();
      tpulses[i] += pulses;
      TrackingLoopFilterInt_Compute(&EncoderFilter[i], tpulses[i]);
      MotorsSpeedMeasured[i] = (float)EncoderFilter[i].DiffOutput * PULSE2RAD * RAD2RPM;
      //MotorsSpeedMeasured[i] = (float)pulses * PULSE2RAD * RAD2RPM / dt;
      PIDvalue[i] = PIDx[i].Compute(MotorsSpeedMeasured[i], MotorsSpeedSetpoint[i]);
      PWMvalue[i] = PIDvalue[i] * VOLTAGE2PWM;
      Motor[i].Set(PWMvalue[i]);   
      if(printCounter >= (500 / EncoderInterval))
      {
        Serial.print(MotorsSpeedSetpoint[i]);
        Serial.print(" ");   
        Serial.print(MotorsSpeedMeasured[i]);
        Serial.print("   ");
        if(i >= N_MOTORS-1) 
        {
          Serial.println();
          printCounter = 0;
        }
      }
    }
    
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
