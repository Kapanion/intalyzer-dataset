#include <TimerOne.h>
#define PULSE 300

int channels[6] = {1200,1175,1200,1300,1200,1200};
char chr;
String str;
void setup(void){
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  Timer1.initialize(18000);
  Timer1.attachInterrupt(relay);
  Serial.begin(57600);
  delay(5000);
}

void pulse() {
  digitalWrite(10, LOW);
  delayMicroseconds(PULSE);
  digitalWrite(10,HIGH);
}

void channel(int delay) {
  pulse();  
  delayMicroseconds(delay);
}

void loop(void){
/*  while( Serial.available() ){
    chr=(char)Serial.read();
    if (chr=='\n'){
      upArray(str);
    }
    else {str+=chr;}
  }*/
  /*channels[4]=1200;
  channels[2]=1300;
  delay(2000);
  channels[2]=1200;
  delay(500);
  channels[4]=1275;
  delay(3000);
  channels[4]=1200;
  delay(500);*/
}
    
void relay(void){
  for(int i=0; i<6; i++){
    channel(channels[i]);
  }
}
void upArray(String upStr){
  int index=atoi(upStr.substring(0,1).c_str());
  int val=atoi(upStr.substring(1).c_str());
  channels[index]=val+700;
}
