// Faucet Add-On System for Smart Water Management
#include <LiquidCrystal.h>

#define flowSensor 2
#define triggerPin 3
#define echoPin 4
#define relayPin 5
#define buzzer 6
#define led 7

volatile int flowCount;
float flowRate;
unsigned long totalMilliLitres;
unsigned long oldTime = 0;

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

void setup() {
  pinMode(flowSensor, INPUT);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(led, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(flowSensor), flow, RISING);

  lcd.begin(16, 2);
  Serial.begin(9600);

  digitalWrite(relayPin, LOW); // Pump initially off
}

void loop() {
  unsigned long currentMillis = millis();
  
  // ----- Running Mode -----
  if (currentMillis - oldTime > 1000) {
    detachInterrupt(digitalPinToInterrupt(flowSensor));
    flowRate = (flowCount * 60 / 7.5); // L/h
    totalMilliLitres += (flowCount * 1000 / 7.5) / 60;
    flowCount = 0;
    oldTime = currentMillis;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Flow: ");
    lcd.print(flowRate);
    lcd.print(" L/h");

    lcd.setCursor(0, 1);
    lcd.print("Used: ");
    lcd.print(totalMilliLitres / 1000);
    lcd.print(" L");

    if (totalMilliLitres >= 5000) { // 5L Limit
      digitalWrite(relayPin, LOW); // Turn off pump
      tone(buzzer, 1000);
      digitalWrite(led, HIGH);
    }
    attachInterrupt(digitalPinToInterrupt(flowSensor), flow, RISING);
  }

  // ----- Filling Mode -----
  long duration;
  digitalWrite(triggerPin, LOW); delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH); delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;

  if (distance < 5) { // Tank full
    digitalWrite(relayPin, LOW);
    tone(buzzer, 1000);
    digitalWrite(led, HIGH);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Tank Full!");
  } else {
    digitalWrite(relayPin, HIGH);
    noTone(buzzer);
    digitalWrite(led, LOW);
  }

  delay(500);
}

void flow() {
  flowCount++;
}
