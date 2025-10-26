#include "LetESP32.h"

const char* ssid = "NETGEAR35";
const char* password = "magicalfinch482";
const char* ws = "ws://192.168.1.2:1234";
LetESP32 tracer(ssid, password, ws, "7sgC703x");

#define SIZE 6000 

// Volatile keyword stops the compiler optimising
// the arrays away
volatile float a[SIZE];
volatile float b[SIZE];
volatile float c;

//Q5 interrupts

hw_timer_t *timer;
timer = timerBegin(1, 80, true); 

void IRAM_ATTR timerISR() {
        // ISR body
}

void dotProduct() {
   for(int i=0; i<SIZE; i++) {
      c += a[i] * b[i];
   }	
}

void randomise(){
   for(int i=0; i<SIZE; i++) {
      a[i] = esp_random();
      b[i] = esp_random();
   }	
}

void setup() {
  randomise();
}

void loop() {
  timerAttachInterrupt(timer, &timerISR, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
  dotProduct();
  delayMicroseconds(750);
}