#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

//ultrasonics sensor 
#define echoPinR 12
#define trigPinR 3 
#define echoPinL 4 
#define trigPinL 9 
#define pwmL 5
#define dirL 8
#define pwmR 6
#define dirR 11

int speed = 60;
int range = 50;

// defines variables
long durationR, durationL; 
double distanceR, distanceL; 

//PID constants
double kp = 24;
double ki = 0;
double kd = 0;
 
unsigned long currentTime, previousTime;
double elapsedTime;
double error, errorR;
double lastError, lastErrorR;
int output1, output2, setPoint;
double cumError, rateError, rateErrorR;

void setup() {
  setPoint = 20;                      //set point is 20cm distance 
  //motor signal pins
  pinMode(pwmR, OUTPUT); 
  pinMode(dirR, OUTPUT); 
  pinMode(pwmL, OUTPUT); 
  pinMode(dirL, OUTPUT); 
  //US sensor pins
  pinMode(trigPinR, OUTPUT); 
  pinMode(echoPinR, INPUT); 
  pinMode(trigPinL, OUTPUT); 
  pinMode(echoPinL, INPUT); 
  Serial.begin(9600);
  //imu
  Serial.println("Orientation Sensor Test"); Serial.println("");
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  //interrupt for making 2 point contact
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), blink, LOW);
}

void loop() {

  wall_finder();
}

//First stage of Aligning i.e. finding wall 
void wall_finder(){ 
  
  double Q = angle();
  if ((20 > Q) && (Q > 3)){

    analogWrite(pwmL, 0); 
    digitalWrite(dirL, HIGH);   
    analogWrite(pwmR, 0);
    digitalWrite(dirR, HIGH);  
    delay(2);
    Serial.print(" wall found ");
    Serial.print(Q);
    PID();
  }

  else{
    analogWrite(pwmL, speed);  
    digitalWrite(dirL, HIGH);   
    analogWrite(pwmR, speed);  
    digitalWrite(dirR, LOW);  
    delay(2);
    Serial.print(" detecting wall ");
    delay(10);
  }
}
//Second stage of Aligning i.e aligning bot perpendicular to wall
void PID() {
  double distanceL = distL ();
  double distanceR = distR ();
  distanceR = constrain(distanceR, 0, 2000);
  distanceL = constrain(distanceL, 0, 2000);
  output1 = computePIDL(distanceL);
  output2 = computePIDR(distanceR);

  if ((distanceR > range+10) && (distanceL > range+10))//forward
  {
    analogWrite(pwmL, speed);  
    digitalWrite(dirL, HIGH);
    analogWrite(pwmR, speed);   
    digitalWrite(dirR, HIGH);   
    delay(2);
    Serial.print(" out of range ");
  }
  else{

    if (output2 >= 0){
      analogWrite(pwmR, output2);
      digitalWrite(dirR, LOW);
    }
    else{
      analogWrite(pwmR, -output2);
      digitalWrite(dirR, HIGH);
    }
    
    if (output1 >= 0){
      analogWrite(pwmL, output1);
      digitalWrite(dirL, LOW);
    }
    else{
      analogWrite(pwmL, -output1);
      digitalWrite(dirL, HIGH);
    }
    Serial.print(" in range ");

  }
   Serial.print("setPoint ");Serial.print(setPoint);Serial.print(" distanceL ");Serial.print(distanceL);Serial.print(" distanceR ");Serial.print(distanceR);Serial.print("  ");Serial.print("  ");Serial.print("error ");Serial.print(error);
   Serial.print("  ");Serial.print("  ");Serial.print("output1 ");Serial.println(output1);
}

//3rd stage of aligning ie.e bot moves forward to make 2 point contact
void blink() {    
  analogWrite(pwmL, 50); 
  digitalWrite(dirL, HIGH);  
  analogWrite(pwmR, 50);   
  digitalWrite(dirR, HIGH);  
  delay(2);
  Serial.println(" interrupt ");
  //2 point contact made successfully
  while (digitalRead(7) == 1){
    analogWrite(pwmL, 0);  
    digitalWrite(dirL, HIGH);   
    analogWrite(pwmR, 0);  
    digitalWrite(dirR, HIGH);  
    Master_Shutdown();   
  }
     // to stop all other codes
}

//Fourth and last stage i.e. shutdown

void Master_Shutdown(void)
{
  analogWrite(pwmR, 0);
  analogWrite(pwmL, 0);
}
  
//All the functions for calculations

long distL () {
  digitalWrite(trigPinL, LOW);                    //Left trigger
  delayMicroseconds(2);
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);

  durationL = pulseIn(echoPinL, HIGH);            // Reads the echoPin, returns the sound wave travel time in microseconds
  long L = durationL * 0.034 / 2;
  return L;
}

double computePIDL(double distanceL){


  currentTime = millis();                         //get current time
  elapsedTime = currentTime - previousTime;       //compute time elapsed from previous computation

  error = setPoint - distanceL;                   // determine error
  cumError += error * elapsedTime;                // compute integral
  rateError = (error - lastError)/elapsedTime;    // compute derivative

  double output1 = kp*error + ki*cumError + kd*rateError; //PID output               

  lastError = error;                              //remember current error
  previousTime = currentTime;                     //remember current time

  return output1;                                 //have function return the PID output
}


long distR () {

  digitalWrite(trigPinR, LOW);                    //right trigger
  delayMicroseconds(2);
  digitalWrite(trigPinR, HIGH);                   // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);

  durationR = pulseIn(echoPinR, HIGH);
  long R = durationR * 0.034 / 2;                 // Speed of sound wave divided by 2 (go and back)
  return R;
}

double computePIDR(double distanceR){
  errorR = setPoint - distanceR;                  // determine error
  cumError += error * elapsedTime;                // compute integral
  rateErrorR = (errorR - lastErrorR)/elapsedTime; // compute derivative

  double output2 = kp*errorR + ki*cumError + kd*rateErrorR; //PID output               

  lastErrorR = errorR;                            //remember current error
  previousTime = currentTime;                     //remember current time

  return output2;                                 //have function return the PID output
}

double angle(){

  Serial.print(" calculating ");
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.println(event.orientation.x, 4);
  
  delay(100);
  return event.orientation.x;
}

