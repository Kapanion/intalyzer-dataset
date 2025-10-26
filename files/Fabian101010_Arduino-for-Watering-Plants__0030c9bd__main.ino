#include <avr/sleep.h>
#include "customFunctions.h"
#include <avr/interrupt.h>

#define sensor1Power 9
#define sensor2Power 8
#define sensor1analog A0
#define sensor2analog A1
#define relay1 12
#define relay2 11
#define button 2

int minMoist = 25;
int moistVal1;
int moistVal2;

int sleepCounter = 75;
int sleepCounterMultiplier = 12;

//test ints
int wokeCounter = 0;

char buffer1[100];
char buffer2[100];

volatile bool buttonPressed = false;


// code for activiating the real time counter, for sleep mode
void RTC_init(void)
{
  while (RTC.STATUS > 0) ;    /* Wait for all register to be synchronized */
 
  RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;        // Run low power oscillator (OSCULP32K) at 1024Hz for long term sleep
  RTC.PITINTCTRL = RTC_PI_bm;              // PIT Interrupt: enabled */
  RTC.PITCTRLA = RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm;     // Set period 8 seconds (see data sheet) and enable PIC                      
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm;          // Clear interrupt flag by writing '1' (required) 
}

void setup() {

  //run the real time counter function
  RTC_init();

  Serial.begin(9600);
  
  // define pins
  pinMode(sensor1Power, OUTPUT);
  pinMode(sensor2Power, OUTPUT);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(button, INPUT_PULLUP);

  // write pins
  digitalWrite(sensor1Power, LOW);
  digitalWrite(sensor2Power, LOW);
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);

  // sleep mode stuff
  attachInterrupt(digitalPinToInterrupt(button), wakeUp, LOW);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();    
}

void wakeUp() {
  // this executes if the button is pressed, and creates a flag for the pump 
  buttonPressed = true; 
}

void loop() {

  for (int m = 0; m < sleepCounter; m++){
    sleep_cpu();
  }

  // set sleepCounter back to normal
  sleepCounter = 75;


  // if buttom was pressed, start the pumps for one sec
  if (buttonPressed == true) {
    Serial.print(millis());
    digitalWrite(relay1, LOW);
    digitalWrite(relay2, LOW);
    delay(1000);
    digitalWrite(relay1, HIGH);
    digitalWrite(relay2, HIGH);
    buttonPressed = false; // set flag back to false so the if clause is no longer true
    sleepCounter = 75;
  }
  // sign for wake up
  digitalWrite(LED_BUILTIN, HIGH); 
  delay(200); 
  digitalWrite(LED_BUILTIN, LOW);

  // measure each sensor once to save power
  moistVal1 = getMoistVal(sensor1Power, sensor1analog, 1);
  moistVal2 = getMoistVal(sensor2Power, sensor2analog, 1);
  
  // serial print the values, flushing so the output gets clean printed 
  /*
  sprintf(buffer1, "Sensor 1 Value: %d %% moisture // Current millis: %lu", moistVal1, millis());
  Serial.println(buffer1);
  Serial.flush();
  sprintf(buffer2, "Sensor 2 Value: %d %% moisture", moistVal2);
  Serial.println(buffer2);
  Serial.flush();
  */

// if the first measurement was dry, measure again to confirm
if (moistVal1 < minMoist || moistVal2 < minMoist) {

    // measure multiple times to get accurate reading
    moistVal1 = getMoistVal(sensor1Power, sensor1analog, 3);
    moistVal2 = getMoistVal(sensor2Power, sensor2analog, 3);

    // used later for watering
    bool is1moist = moistVal1 >= minMoist;
    bool is2moist = moistVal2 >= minMoist;

    // if either value is not true, than the sleep counter gets multiplies (the next measurment will take one hour)
    if(!is1moist || is2moist) {
      sleepCounter = sleepCounter * sleepCounterMultiplier;
    }

    // if the soil is dry, activiate the first pump
    if(!is1moist) {
      digitalWrite(relay1, LOW);
      delay(4500);
      digitalWrite(relay1, HIGH);
    }
    // if the soil is dry, activiate the first pump
    if(!is2moist) {
      digitalWrite(relay2, LOW);
      delay(4500);
      digitalWrite(relay2, HIGH);
    }
  } 
}
