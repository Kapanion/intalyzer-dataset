#include "Queue.h"
int valueLength = 5;
float lastLightValue = 0;
int writeIndex = 0;
int sensorThreshHold = 800;
float period;
bool pressed = false;
bool release = false;
bool isSteady = false;
bool getLight = false;
Queue timeValues;
Queue fallingValues;
long startTime;
int Button = 2;
int Relay = 4;
int Arm = 5;
void setup() {
//button setup
  pinMode(Button, INPUT_PULLUP);
//set up pins to sensors and motors
  pinMode(Relay,OUTPUT);
  pinMode(Arm,OUTPUT);
  digitalWrite(Arm,LOW);
  //pull up for the button, interupt on falling edge
  attachInterrupt(digitalPinToInterrupt(Button),press,FALLING);
  Serial.begin(9600);
}

void loop() {
//read the light value from the light sensor
  int light = analogRead(A1);
//get the time
  float time = millis();
  //this is used to get light data
  //if(getLight){
    // Serial.print("LIGHT: , ");
    // Serial.print(time);
    // Serial.print(" , ");
    // Serial.print(light);
    // Serial.println();
 // }
 //on rising edge, light first detected 
  if(lastLightValue < 800 && light >= 800){
    //release frisbee after a certain amount of time has passed, is steady, and nextIsThrowArm
    if(isSteady && timeValues.nextIsThrowArm(fallingValues) && millis() - startTime > 0){
    //delay to throw at 90 degree angle
      delay(((timeValues.average())/2));
    //start motor to release the frisbee, thne stop it
      digitalWrite(Arm,HIGH);
      delay(100);
      digitalWrite(Arm,LOW);
      //turn arm motors off
      digitalWrite(Relay,LOW);
      release = false;
      pressed = false;
      getLight = false;
    }
    //push time to Queue of shadow starts
     timeValues.push(time); 
    //check steadiness
    isSteady = timeValues.checkSteadiness(20);
  //   Serial.print("RISING: , ");
  //  for(int i = 0; i < 5; i++){
  //     Serial.print(timeValues.array[i]);
  //     Serial.print(" , ");
  //     }  
      //Serial.print(isSteady);
      //Serial.println();
//     Serial.print("PERIODS: , ");
//     for(int i = 0; i < 4; i++){
//       Serial.print(timeValues.periods[i]);
//       Serial.print(" , ");
//     }  
//       Serial.print(timeValues.average());
//       Serial.print(" , ");
//       Serial.println();

//   }
  if(lastLightValue >= 800 && light < 800){
    //push time to end of shadow times
    fallingValues.push(time);
  //   Serial.print("FALLING: , ");
  //  for(int i = 0; i < 5; i++){
  //     Serial.print(fallingValues.array[i]);
  //     Serial.print(" , ");
  //     }
  //     Serial.println();
  }
  lastLightValue = light;
}

void press(){
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  //button debounce
  if (interrupt_time - last_interrupt_time > 250){ 
  
  //Serial.println("Button Pressed");
  //turn arm motor on
  if(!pressed){
    digitalWrite(4,HIGH);
    pressed = true;
    getLight = true;
    
  }
  else if(pressed){
    release = true;
    pressed = false;

  }
  }
  last_interrupt_time = interrupt_time;
}


