#include "EmonLib.h"

#define VOLT_CAL 550.0 //VALOR DE CALIBRAÇÃO (DEVE SER AJUSTADO EM PARALELO COM UM MULTÍMETRO)
EnergyMonitor energyMonitor;

int buzzer = 8;
int ledGreen = 13;
int ledYellow = 11;
int ledRed = 12;
int waterSensor = 2;

int pulseCount;
float waterFlow = 0;
int waitingWater = 0;
bool alertEnabled = false;

void setup() {
    pinMode(buzzer, OUTPUT);
    pinMode(ledGreen, OUTPUT);
    pinMode(ledYellow, OUTPUT);
    pinMode(ledRed, OUTPUT);

    pinMode(waterSensor, INPUT);

    Serial.begin(9600);
    energyMonitor.voltage(2, VOLT_CAL, 1.7);
    attachInterrupt(0, pulseIncrement, RISING);
}

void loop() {
  if(alertEnabled == false){
      alertOff();
  }

  int currentWaterFlow = getWaterFlow();
  int currentVoltage = getCurrentVoltage();

  Serial.print("Debug: Voltage ");
  Serial.print(currentVoltage);
  Serial.print(" waterFlow:");
  Serial.print(currentWaterFlow);
  Serial.println("L/m");

  if(currentVoltage > 100){
      if(alertEnabled != true && currentWaterFlow < 1){
          waitingWater++;
          ledYellowOn();
          alertOn();
          delay(2000);
          alertOff();

          if(waitingWater < 30){
              return;
          }
      }

      if(currentWaterFlow < 1){
          ledsOff();
          alertOn();
          ledRedOn();
      }else{
          ledGreenOn();
          alertOff();

          Serial.println("Caixa enchendo com sucesso");
      }
  }else{
      waitingWater = 0;
      alertOff();
      ledsOff();
  }

}

void alertOn() {
     alertEnabled=true;
    digitalWrite(buzzer, LOW);
}

void alertOff() {
     alertEnabled=false;
    digitalWrite(buzzer, HIGH);
}

void ledGreenOn(){
	digitalWrite(ledGreen, HIGH);
	digitalWrite(ledYellow, LOW);
	digitalWrite(ledRed, LOW);
}

void ledYellowOn(){
	digitalWrite(ledGreen, LOW);
	digitalWrite(ledYellow, HIGH);
	digitalWrite(ledRed, LOW);
}

void ledRedOn(){
	digitalWrite(ledGreen, LOW);
	digitalWrite(ledYellow, LOW);
	digitalWrite(ledRed, HIGH);
}

void ledsOff(){
	digitalWrite(ledGreen, LOW);
	digitalWrite(ledYellow, LOW);
	digitalWrite(ledRed, LOW);
}

float getCurrentVoltage(){
  energyMonitor.calcVI(17,2000);

  float supplyVoltage   = energyMonitor.Vrms;
  delay(1000);

  return supplyVoltage;
}

int getWaterFlow){
    pulseCount = 0;
    sei();
    delay(1000);
    cli();
    waterFlow = pulseCount / 7.5;

  return waterFlow;
}

void pulseIncrement(){
     pulseCount++;
}

