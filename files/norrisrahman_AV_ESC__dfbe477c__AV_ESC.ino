
#define ARDUINO_BLACKPILL_F411CE
#define USE_USBCON

#include <SoftwareSerial.h>
#include <ros.h>
#include <Servo.h>
#include <std_msgs/Int16.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"

 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;

Servo servo;
Servo motor;
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A PA0 // Hijau
#define ENC_IN_RIGHT_A PA1 // Hijau
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B PB13 // Kuning
#define ENC_IN_RIGHT_B PB14 // Kuning


#define MOTOR PB5
#define SERVO PB6
 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

int angle = 30;
 
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 angleCount;
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
ros::Publisher anglePub("angle_pub", &angleCount);

 
// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}

void steering( const geometry_msgs::Twist& cmd_msg){
  
  int gas = int(cmd_msg.linear.x);
  angle = int(cmd_msg.angular.z);

  angleCount.data = angle;
  
  motor.writeMicroseconds(gas); // constant speed (1480: 2m/s, 1450: 0.5m/s, 1500: stop)
  servo.write(angle); //set servo angle, should be from 0-180
  // for (int pos = 0; pos <= 180; pos += 1) {
  //   servo.write(pos);
  //   delay(15);
  // }

}


ros::Subscriber<geometry_msgs::Twist> sub("car/cmd_vel", steering);
 
void setup() {
 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  // attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  // attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  motor.attach(MOTOR);
  servo.attach(PB6); //attach it to pin 9//UP DOWN
  
  // ROS Setup
  // nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.advertise(anglePub);
}
 
void loop() {
   
  // Record the time
  currentMillis = millis();
 
  // If 100ms have passed, print the number of ticks
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;

    angleCount.data = angle;
     
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
    anglePub.publish( &angleCount);
    nh.spinOnce();
  }
}