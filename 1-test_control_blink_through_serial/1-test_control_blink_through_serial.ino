#include <Wire.h>


#define LedPin            LED_BUILTIN
#define MaxLedInterval    2500 /*!< Max LED blink time */
#define MinLedInterval    50   /*!< Min LED blink time */
#define OffLedInterval    0    /*!< Command turns off LED */
#define OnLedInterval     1    /*!< Command turns on LED */

#define serialInterval   100   /*!< Time between checks for serial data */

/**
 * @brief Periodically blinks the built in LED.
 */
void LED_Blink(void);

/**
 * @brief Reads and decods commands from serial port.
 */
void handleSerial(void);

/**
 * @brief Send o serial port the LED interval.
 */
void PRINT_LedInterval(void);





char LedState = LOW;              /* LedState used to set the LED */
char SerialState = LOW;           /* Serial starts off */
char PRINTisenabled = false;
unsigned long previousMillis = 0; /* will store last time LED was updated */
unsigned long LedInterval = 2500; /* interval at which to blink (milliseconds) */

void setup() {
  Serial.begin(115200);
  Serial.println("Type 'W' to wakeup serial, 'S' to shut it down");
  Serial.println("Type '0' to turn off LED, '1' to turn it on");
  Serial.println("Type '+' to increase blink speed, '-' to reduce it");
  Serial.println(LedInterval);
  pinMode(LedPin,OUTPUT);
}

void loop() {
  /* Read and decode commands from serial port */
  handleSerial();
  /* Print LED time interval */
  PRINT_LedInterval();
  /* Blinking a LED */
  LED_Blink();

}// loop


void handleSerial(void) {
   char incomingCharacter;
   if (Serial.available() > 0) {
     //Serial.readBytes(&incomingCharacter, 1);
     incomingCharacter = Serial.read();
     switch (incomingCharacter) {
       case '0':
        LedInterval = OffLedInterval;
       break;
       case '1':
        LedInterval = OnLedInterval;
       break;
       case '+':
        LedInterval = LedInterval + 50;
        if (LedInterval >= MaxLedInterval)
           LedInterval = MaxLedInterval;
        else if(LedInterval <= MinLedInterval)
           LedInterval = MinLedInterval;
       break;
   
       case '-':
        LedInterval = LedInterval - 50;
        if (LedInterval <= MinLedInterval)
           LedInterval = MinLedInterval;
        else if (LedInterval >= MaxLedInterval)
           LedInterval = MaxLedInterval;
       break;
       case 'W':
        PRINTisenabled = true;
       break;
       case 'S':
        PRINTisenabled = false;
       break;
       default:
       break;
      }
   }
}

void PRINT_LedInterval(void){
  static unsigned long previousMillis = 0, currentMillis;

  if( PRINTisenabled != false ){
    currentMillis = millis();  
    if(currentMillis - previousMillis >= serialInterval){
      previousMillis = currentMillis;
      /* Prints the raw bytes */
      //Serial.write((unsigned char)(LedInterval)&0xFF);
      //Serial.write((unsigned char)(LedInterval>>8)&0xFF);
      /* Prints a formated string */
      Serial.println(LedInterval);
    }
  }
}

void LED_Blink(void){
  static unsigned long previousMillis = 0, currentMillis;
  static int ledState = 0;
    
  if( LedInterval >= MinLedInterval ){
    currentMillis = millis();
    if (currentMillis - previousMillis >= LedInterval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      ledState ^= 1;
      digitalWrite(LedPin, ledState);
    }
  }else{
    digitalWrite(LedPin, LedInterval&1);
  }
}//LED_Blink
