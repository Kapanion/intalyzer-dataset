#include <LowPower.h>
#include <SPI.h>
#include <Wire.h>
#include "SensirionI2CScd4x.h"
#include <Arduino.h>
#include "history_buffer.h"
#include "display.h"
#include "trend.h"

#include <YetAnotherPcInt.h>

SensirionI2CScd4x scd4x;

#define INPUT_PIN_INT   2

bool should_display = false;
long prev_isr;

void isr(void)
{
  if (millis() - prev_isr < 300) return;
  
  should_display = true;
  prev_isr = millis();
}

void setup(void)
{
  Serial.begin(9600);

  Serial.println("setting up");
  
  Wire.begin();

  scd4x.begin(Wire);
  scd4x.stopPeriodicMeasurement();

  pinMode(INPUT_PIN_INT, INPUT_PULLUP);
  PcInt::attachInterrupt(INPUT_PIN_INT, isr, FALLING);

  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);

  // set ADC reference to 1.1v
  analogReference(INTERNAL);
  analogRead(A1);
}

Display display;
Trend trend;
HistoryBuffer history;

void show_history(void)
{
  if (should_display) {
    Serial.println("got interrupt");
    should_display = false;
    display.history(history);
  }
}

void sleep(void)
{
  show_history();
  for(int i = 0; i < 15; ++i)
  {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    show_history();
  }
}

void loop(void)
{
  uint16_t co2;
  float dummy;
  uint16_t ready;
  
  scd4x.wakeUp();
  
  scd4x.measureSingleShot();
  
  scd4x.getDataReadyStatus(ready);
  while((ready & 0x07FF) == 0) {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    scd4x.getDataReadyStatus(ready);
  }

  scd4x.readMeasurement(co2, dummy, dummy);

  digitalWrite(A0, LOW);
  
  scd4x.powerDown();

  co2 = constrain(co2, 0, 9999);
  
  char level;
  if (co2 < 600) {
    level = 0;
  } else if (co2 < 1000) {
    level = 1;
  } else if (co2 < 1400) {
    level = 2;
  } else {
    level = 3;
  }
  
  float volt = (analogRead(A1)) / 318.8;
  digitalWrite(A0, HIGH);
  float percent = (volt - 2.4) / 0.6;
  percent = constrain(percent, 0.0, 0.99);

  history.write(co2);

  display.update(co2, trend.get(co2), level, volt, percent);

  sleep();
}
