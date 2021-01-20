#include <math.h>
#include <Wire.h>
#include "rotary_encoder.h"
#include "tracking_loop_filter.h"
#include "dcmotor.h"
#include "pid.h"
#include "myalgebra.h"


#define N_MOTORS          3


#define LedPin            LED_BUILTIN
#define LedInterval       500  /*!< LED blink time */
#define EncoderInterval   20   /*!< Time interval between encoder readings */
#define MotorInterval     20   /*!< Time interval between changes in motor speed */
#define PoseInterval      200  /*!< Pose will be checked every 200ms */

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
//#define Kpm               0.032547  /* PID proportional gain for motor control */
//#define Kim               0.32631   /* PID integral gain for motor control */
//#define Kdm               0.0       /* PID derivative gain for motor control */
/* PID gains for the motor controller */
#define Kpm               0.03657     /* PID proportional gain for motor control */
#define Kim               0.48033     /* PID integral gain for motor control */
#define Kdm               0.0         /* PID derivative gain for motor control */


//Helpful constants
#define MOTOR_PPR         937           //Pulses per revolution for the encoder
#define VOLTAGE2PWM       21.25       //PWM_MAX/VoutMax
#define RPM2RAD           (PI/30.0)     //conversion from RPM to rad/s
#define RAD2RPM           (30.0 / M_PI)     //conversion from rad/s to RMP
#define PULSE2RAD         (2 * M_PI / MOTOR_PPR)   //6.70420967e-3 //(2*PI)/PPR
#define PULSES2RPM        3.201707577 //((1/MOTOR_PPR)/(encoderInterval/1000))*60
#define RAD2DEG           (180.0 / M_PI)
#define DEG2RAD           (M_PI / 180.0)


/* Sine and cosine of important angles */
#define SIN0              0
#define COS0              1
#define SIN30             0.5
#define COS30             0.866025403
#define SIN60             COS30
#define COS60             SIN30
#define SIN90             1
#define COS90             0

#define SIN120            SIN60
#define COS120            -COS60
#define SIN150            SIN30
#define COS150            -COS30
#define SIN180            0.0
#define COS180            -1.0

#define SIN210            -SIN150
#define COS210            COS150
#define SIN240            -SIN120
#define COS240            COS120
#define SIN270            -1
#define COS270            0

#define SIN300            -SIN60
#define COS300            COS60
#define SIN330            -SIN30
#define COS330            COS30
#define SIN360            0
#define COS360            1


/* Known base parameters */
#define Rwheels           0.025  //Radius of base wheels(m)
#define L                 0.0875 //Distance between wheels and the center of the base(m)
#define Gamma             0      //Angle between the main wheel and its free wheels

/* PID gains for the pose controller */
#define Kpb               2.0
#define Kib               0.0//0.0*(poseInterval/1000.0)



/**
 * @brief Periodically blinks the built in LED.
 */
void LED_Blink(void);

/**
 * @brief Prints data to serial port.
 */
void PRINTserial(void);

/**
 * @brief Converts floating point number to fixed point with 10 integers and 6 decimal places.
 */
int float2int10q6(float number);

/**
 * @brief Sends a 16 bits variable to serial port.
 */
void send2serial(float number);

/**
 * @brief Receives and decodes data from serial port.
 */
void handleSerial(void);

/**
 * @brief Prints on serial port a 2d array (matrix).
 */
void printMatrix(float* A, int m, int n, String label);


/* Setup a RoraryEncoder for pins (A,B): */
RotaryEncoder Encoder[N_MOTORS];
TrackingLoopFilterInt_t EncoderFilter[N_MOTORS];
/* Setup of motors (pinPWM, pinDir): */
DcMotor Motor[N_MOTORS];
/* Setup of PID controllers (Kp, Ki, Kd, SamplingTime): */
PID PIDx[N_MOTORS];
/* Variables for driving and controlling the motors */
float MotorsSpeedSetpoint[N_MOTORS] = {0.0, 0.0, 0.0};
float MotorsSpeedMeasured[N_MOTORS];
float PIDvalue[N_MOTORS] = {0};
float PWMvalue[N_MOTORS];


/* Variables for the kinematic model */
/* MCD = (1/Rwheels)*[-sin(alfa1) cos(alfa1) L;-sin(alfa2) cos(alfa2) L;-sin(alfa3) cos(alfa3) L] */
float MCD[9] = {-SIN180,COS180,L,-SIN300,COS300,L,-SIN60,COS60,L}; /* Direct kinematic model */
float MCI[9]; /* Inverse kinematic model */
float costeta = 1.0;
float sinteta = 0.0;
/* Rteta = [cos(teta) sin(teta) 0;-sin(teta) cos(teta) 0;0 0 1]; */
float Rteta[9]; /* ROtato matrix */
float invRteta[9]; /* invRteta = Rteta' */
float auxMatrix[9];
float Posed[3] = {0.0, 0.0, 0.0}; /* Desired pose for the base */
float Poseb[3] = {0.0, 0.0, 0.0}; /* Pose measured using the encoders */
float RobotSpeedSetpoint[3] = {0.0, 0.0, 0.0};
float RobotSpeedMeasured[3] = {0.0, 0.0, 0.0};
float auxVector[3];


/* Variables for the pose control */
float errbmin = 0.00001;
float errb[3] = {0.0};
float cumerrb[3] = {0.0};
int movimentFinished = false; /* Flags if moviment was finished successfuly */


/* FLags for printing data on serial port */
char isPrintEnabled = true;
char isPrintSlowMode = true;
char isPrintReadableMode = true;



void setup() {
  Serial.begin(115200);
  
  pinMode(SLPpin1,OUTPUT);
  digitalWrite(SLPpin1, HIGH);
  pinMode(SLPpin2,OUTPUT);
  digitalWrite(SLPpin2, HIGH);
  pinMode(LedPin,OUTPUT);
  digitalWrite(LedPin, HIGH);

  Encoder[0].Begin(Encoder1PinA, Encoder1PinB);
  Encoder[1].Begin(Encoder2PinA, Encoder2PinB);
  Encoder[2].Begin(Encoder3PinA, Encoder3PinB);

  TrackingLoopFilterInt_Init(&EncoderFilter[0], 40.0, 900.0, (float) EncoderInterval * 0.001);
  TrackingLoopFilterInt_Init(&EncoderFilter[1], 40.0, 900.0, (float) EncoderInterval * 0.001);
  TrackingLoopFilterInt_Init(&EncoderFilter[2], 40.0, 900.0, (float) EncoderInterval * 0.001);
  
  PIDx[0].Begin(Kpm, Kim, Kdm, EncoderInterval);
  PIDx[0].SetLimits(12, -12);
  PIDx[0].SetMaxError(1);
  PIDx[1].Begin(Kpm, Kim, Kdm, EncoderInterval);
  PIDx[1].SetLimits(12, -12);
  PIDx[1].SetMaxError(1);
  PIDx[2].Begin(Kpm, Kim, Kdm, EncoderInterval);
  PIDx[2].SetLimits(12, -12);
  PIDx[2].SetMaxError(1);
  
  Motor[0].Begin(Motor1PWM, Motor1Dir);
  Motor[0].Set(0, 1);
  Motor[1].Begin(Motor2PWM, Motor2Dir);
  Motor[1].Set(0, 1);
  Motor[2].Begin(Motor3PWM, Motor3Dir);
  Motor[2].Set(0, 1);
  

  scaleMatrix(MCD, MCD, 3, 3, 1/Rwheels);
  copyMatrix(MCD,3,3,MCI);
  if(!invertMatrix(MCI, 3)){
    Serial.println("Failed to invert a matrix");
    while(1);
  }
  /*
  printMatrix(MCD, 3, 3, "MCD");
  printMatrix(MCI, 3, 3, "MCI");
  while(1);
  */
  costeta = 1.0;
  sinteta = 0.0;
/* Rteta= [cos(teta) sin(teta) 0;-sin(teta) cos(teta) 0;0 0 1]; */
  Rteta[0] = costeta;
  Rteta[1] = sinteta;
  Rteta[2] = 0.0;
  Rteta[3] = -sinteta;
  Rteta[4] = costeta;
  Rteta[5] = 0.0;
  Rteta[6] = 0.0;
  Rteta[7] = 0.0;
  Rteta[8] = 1.0;
//invRteta = Rteta'
  transposeMatrix( Rteta, 3, 3, invRteta );  

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
  PCICR &= ~(1 << PCIE1);
  Encoder[0].tick(); // just call tick() to check the state.
  Encoder[1].tick();
  Encoder[2].tick();
  PCICR |= (1 << PCIE1);
}

void loop() {

  static unsigned long previousMillis = 0, previousMillisBase = 0, currentMillis;
  long pulses;
  long aux32;
  static int64_t tpulses[N_MOTORS];
  float dt = 0.0, normVb, normVb_r, auxf;

  currentMillis = millis();
  if (currentMillis - previousMillis >= EncoderInterval)
  {
    previousMillis = currentMillis;
    dt = (float) EncoderInterval * 0.001;

    for(int i = 0; i < N_MOTORS; i++)
    {
      /* Motor control */
      pulses = Encoder[i].readPulses();
//      tpulses[i] += pulses;
//      TrackingLoopFilterInt_Compute(&EncoderFilter[i], tpulses[i]);
//      MotorsSpeedMeasured[i] = (float)EncoderFilter[i].DiffOutput * PULSE2RAD * RAD2RPM;
      MotorsSpeedMeasured[i] = (float)pulses * PULSE2RAD * RAD2RPM / dt;
      PIDvalue[i] = PIDx[i].Compute(MotorsSpeedMeasured[i], MotorsSpeedSetpoint[i]);
      PWMvalue[i] = PIDvalue[i] * VOLTAGE2PWM;
      Motor[i].Set(PWMvalue[i]);
    }

    PRINTserial();
    
  }//if

  currentMillis = millis();
  if (currentMillis - previousMillisBase >= PoseInterval)
  {
    previousMillisBase = currentMillis;

    /* 
     * Computing base speeds from measured wheel speeds
     * RobotSpeedMeasured = invRteta*MCI*MotorsSpeedMeasured*RPM2RAD;
     * invRteta = Rteta'
     * teta = Poseb[2];
     */
    costeta = cosf(Poseb[2]);
    sinteta = sinf(Poseb[2]);
    invRteta[0] = costeta;
    invRteta[1] = -sinteta;
    invRteta[3] = +sinteta;
    invRteta[4] = costeta;
    transposeMatrix( invRteta, 3, 3, Rteta ); 
    scaleMatrix( MotorsSpeedMeasured, RobotSpeedMeasured, 3, 1, (float)RPM2RAD );
    multiplyMatrix( MCI, RobotSpeedMeasured, 3, 3, 1, auxVector );
    multiplyMatrix( invRteta, auxVector, 3, 3, 1, RobotSpeedMeasured );    
    scaleMatrix( RobotSpeedMeasured, auxVector, 3, 1, PoseInterval*0.001 );
    addMatrix( Poseb, auxVector, 3, 1, Poseb );

    /* Limiting orientation angle between 0 and 2 * PI */
    auxf  = Poseb[2] * 0.5 * M_1_PI;
    aux32 = auxf;
    auxf -= aux32;
    auxf *= 2 * M_PI;
    Poseb[2] = auxf;
    
  }

  handleSerial();
  /* Blinking a LED */
  LED_Blink();


}// loop

void PRINTserial(void)
{
   static int printCounter = 0;
   int aux;
    
    if( isPrintEnabled == true ){

      if(isPrintSlowMode == true)
      {
        printCounter++;
        if(printCounter >= 50){
          printCounter = 0;
        }
      }
      else
      {
        printCounter = 0;
      }

      if(isPrintReadableMode == false)
      {
        if((isPrintSlowMode == false) || (isPrintSlowMode == true && printCounter == 0))
        {
          Serial.write('S'); /* Start byte */
          
          send2serial(MotorsSpeedSetpoint[0]); /* RPM */
          send2serial(MotorsSpeedSetpoint[1]); /* RPM */
          send2serial(MotorsSpeedSetpoint[2]); /* RPM */
  
          send2serial(MotorsSpeedMeasured[0]); /* RPM */
          send2serial(MotorsSpeedMeasured[1]); /* RPM */
          send2serial(MotorsSpeedMeasured[2]); /* RPM */
          
          send2serial(PIDvalue[0]); /* Volts */
          send2serial(PIDvalue[1]); /* Volts */
          send2serial(PIDvalue[2]); /* Volts */
          
          send2serial(RobotSpeedSetpoint[0]); /* m/s */
          send2serial(RobotSpeedSetpoint[1]); /* m/s */
          send2serial(RobotSpeedSetpoint[2]); /* rad/s */
          
          send2serial(RobotSpeedMeasured[0]); /* m/s */
          send2serial(RobotSpeedMeasured[1]); /* m/s */
          send2serial(RobotSpeedMeasured[2]); /* rad/s */
          
          send2serial(Poseb[0]); /* m */
          send2serial(Poseb[1]); /* m */
          send2serial(Poseb[2]); /* rad */
  
          Serial.write('A'); /* End byte */
        }

      }
      else
      {
        
        if((isPrintSlowMode == false) || (isPrintSlowMode == true && printCounter == 0))
        {
          Serial.println();
          printMatrix(MotorsSpeedSetpoint, 1, 3, "Motor speed setpoints (RPM)");
          printMatrix(MotorsSpeedMeasured, 1, 3, "Motor speeds (RPM)");
          printMatrix(PIDvalue, 1, 3, "Motor control signals (V)");
          copyMatrix(RobotSpeedSetpoint, 1, 3, auxVector);
          auxVector[2] *= RAD2RPM; /* Converting robot angular speed from rad/s to RPM for display purposes only*/
          printMatrix(auxVector, 1, 3, "Robot speed setpoints (m/s; m/s; RPM)");
          copyMatrix(RobotSpeedMeasured, 1, 3, auxVector);
          auxVector[2] *= RAD2RPM; /* Converting robot angular speed from rad/s to RPM for display purposes only*/
          printMatrix(auxVector, 1, 3, "Robot speeds (m/s; m/s; RPM)");
          copyMatrix(Poseb, 1, 3, auxVector);
          auxVector[2] *= RAD2DEG; /* Converting robot angular speed from rad to DEG for display purposes only*/
          printMatrix(auxVector, 1, 3, "Robot Pose (m, m, DEG)");
          Serial.println();
        }

      }
      
    }

}

int float2int10q6(float number){
  float aux = number*64.0;
  return( (int)aux);
}

void send2serial(float number){
  /* Used to convert binary to float on unreadable transmissions */
  typedef union {
    float floatingPoint;
    byte binary[4];
  }binaryFloat;

  binaryFloat auxbif;
  auxbif.floatingPoint = number;
  Serial.write(auxbif.binary, 4);
}



void handleSerial(void) 
{
  /* Used to convert binary to float on unreadable receptions */
  typedef union {
    float floatingPoint;
    byte binary[4];
  }binaryFloat;
  
  binaryFloat auxbif;
   static char state = 0;
   char incomingCharacter;
   int aux;
   int setpoint[3] = {0,0,0};

   if( state == 0 )
   {
     if(Serial.available() > 0) 
     {
       //Serial.readBytes(&incomingCharacter, 1);
       incomingCharacter = Serial.read();
       switch (incomingCharacter)
       {
         case 'S':
           state = 1;
           break;
         case '0':
           isPrintEnabled = false;
           break;
         case '1':
           isPrintEnabled = true;
           isPrintSlowMode = true;
           isPrintReadableMode = false;
           break;
         case '2':
           isPrintEnabled = true;
           isPrintSlowMode = false;
           isPrintReadableMode = false;
           break;
         case '3':
           isPrintEnabled = true;
           isPrintSlowMode = true;
           isPrintReadableMode = true;
           break;
         default:
           break;
        }
     }
   }else if( state == 1 )
   {
     /* Unreadable reception */
     if(isPrintReadableMode == false)
     {
       /* Expecting exactly 3 floating point variables (12 bytes) */
       if(Serial.available() >= 12) 
       {
         for(int i = 0; i < 3; i++)
         {
           Serial.readBytes(auxbif.binary, 4);
           RobotSpeedSetpoint[i] = auxbif.floatingPoint;
         }
         /* Computing motor speeds from desired base speed */
         RobotSpeedSetpoint[2] *= RPM2RAD; /* Converting angular speed from RPM to rad/s */
         multiplyMatrix( MCD, RobotSpeedSetpoint, 3, 3, 1, MotorsSpeedSetpoint );
         /* Converting motor speeds from rad/s to RPM */
         scaleMatrix( MotorsSpeedSetpoint, MotorsSpeedSetpoint, 3, 1, (float)RAD2RPM );
         state = 0;
       }
     }
     /* Readable reception */
     else
     {
       /* Expecting at least 0.0;0.0;0.0; (12 chars) */
       if(Serial.available() >= 12) 
       {
         RobotSpeedSetpoint[0] = Serial.parseFloat();
         RobotSpeedSetpoint[1] = Serial.parseFloat();
         RobotSpeedSetpoint[2] = Serial.parseFloat();
         /* Computing motor speeds from desired base speed */
         RobotSpeedSetpoint[2] *= RPM2RAD; /* Converting angular speed from RPM to rad/s */
         multiplyMatrix( MCD, RobotSpeedSetpoint, 3, 3, 1, MotorsSpeedSetpoint );
         /* Converting motor speeds from rad/s to RPM */
         scaleMatrix( MotorsSpeedSetpoint, MotorsSpeedSetpoint, 3, 1, (float)RAD2RPM );
         state = 0;
       }
     }
   }else{
     state = 0;
   }
}


void printMatrix(float* A, int m, int n, String label){
  // A = input matrix (m x n)
  int i, j;
  Serial.println(label);
  for (i = 0; i < m; i++)
  {
    for (j = 0; j < n; j++)
    {
      Serial.print(A[n * i + j],2);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void LED_Blink(void){
  static unsigned long previousMillis = 0, currentMillis;
  static int LedState = 0;
    
  currentMillis = millis();
  if (currentMillis - previousMillis >= LedInterval) 
  {
    previousMillis = currentMillis;
      LedState ^= 1;
      if (LedState) {
        digitalWrite(LedPin, HIGH);
      } else {
        digitalWrite(LedPin, LOW);
      }//else 
    }//if
}//LED_Blink
