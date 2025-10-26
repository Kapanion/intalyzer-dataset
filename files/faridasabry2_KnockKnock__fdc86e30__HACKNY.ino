///HackNY 2016 Hackathon

#include <Servo.h>
#include <SoftwareSerial.h>
Servo myservo;  // create servo object to control a servo

#include <SoftwareSerial.h>// import the serial library
SoftwareSerial mySerial(10, 11); // RX, TX

int pos = 0;    // variable to store the servo position

char receivedChar;
String text;

const long interval = 50;  
unsigned long previousMillis = 0;

void setup() {
  Serial.begin(115200);
 mySerial.begin(9600);//bluetooth serial
  Serial.println("HackNY");
  mySerial.println("HackNY");
    myservo.attach(9);  // attaches the servo on pin 9 to the servo object
attachInterrupt(0,interrupt_knock,RISING);//KNOCK sensor to detect           

}
void interrupt_knock(){

      unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    previousMillis = currentMillis;    // save the last time
       mySerial.println("D");
   Serial.println("D");
  return;
}
  
  }
void loop()
{ //Serial.println(analogRead(A0));
 /* delay(2000);
 mySerial.println("D");
   Serial.println("D");*/
   
 if (mySerial.available())
   {
      receivedChar = mySerial.read();
      Serial.print(receivedChar);
if(receivedChar == ';')
    {
       if(text == "op1")
       {  
          mySerial.print("here");
          Serial.print("here");
          SERVODIS();
       }
      text="";
  }
  else
  {
text+=receivedChar;
  }
} 
}

void SERVODIS(){
    for (pos = 0; pos <= 90; pos += 5) { // goes from 0 degrees to 900 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
  }        
  for (pos = 90; pos >= 0; pos -= 5) { // goes from 90 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  
}
