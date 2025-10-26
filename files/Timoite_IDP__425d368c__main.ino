#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include "Adafruit_TCS34725.h"
#include <Wire.h>
Servo mech_servo;


#define MAX_RANG (520)  //the max measurement value of the module is 520cm(a little bit longer than effective max range) 
#define ADC_SOLUTION (1023.0)  //ADC accuracy of Arduino UNO is 10bit 
#define ULTRASONIC_SAMPLE_PERIOD (10) // sample period in ms

#define DEBUG false	// enables serial monitor output.
#define DEBUG_SERIAL if(DEBUG)SerialNina

int sensityPin = A0;  // ultrasonic input

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Define motors: left and right
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1); // Left Motor
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2); // Right Motor

// Define sensor pins
// changed all int to uint8_t
const uint8_t sensorFarLeft = 2;  // Now at the back (near the wheels)
const uint8_t sensorLeft = 3;
const uint8_t sensorRight = 4;
const uint8_t sensorFarRight = 5;   // Now at the back (near the wheels)
const uint8_t button = 6;
const uint8_t LED_Red = 7;
const uint8_t LED_Green = 8;
const uint8_t LED_Blue = 9;
const uint8_t servo_pin = 10;
const uint8_t loop_speed_test_pin = 11;

//cube color, 1 for red, 2 for black, 3 for nothing, 0 for error
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_614MS, TCS34725_GAIN_1X);
const uint8_t cube_color = 0;

const uint8_t main_speed = 255;
const uint8_t slow_speed = 130;
const int delay_time = 1; // Time that will be delayed every single time

uint8_t mode = 0;   // Mode state: 0=off, 1=forward, 2=backward
unsigned long first_press_time = millis();
const float signal_distance = 10.0;

float dist_t, sensity_t; 

// map_names = [1,2,3,4,...20]
const uint8_t number_of_connections[20] = {1,1,1,1,1,1,1,3,3,3,3,3,3,3,3,3,3,3,2,2}; // This one keeps number of connections each graph has. 
// Compass directions: 1 - North, 2 - East, 3 - South, 4 - West.

const float distances_from_bays[20] = {0, 12, 0, 10, 8, 8, 10, 10.0 ,0,0,10 ,10 ,0,0,10 , 15.0 ,0,0,0,0}; // Put as a coordinate graph number - 1

const uint8_t better_map_of_directions[20][20] = { // First coordinate is current graph, second is next graph. Result is the compass direction
{5,0,0,0,0,  0,0,1,0,0,     0,0,0,0,0,  0,0,0,0,0},
{0,5,0,0,0,  0,0,0,0,1,     0,0,0,0,0,  0,0,0,0,0},
{0,0,5,0,0,  0,0,0,0,0,     1,0,0,0,0,  0,0,0,0,0},
{0,0,0,5,0,  0,0,0,3,0,     0,0,0,0,0,  0,0,0,0,0},
{0,0,0,0,5,  0,0,0,0,0,     0,0,0,1,0,  0,0,0,0,0},

{0,0,0,0,0,  5,0,0,0,0,     0,0,0,0,0,  0,0,2,0,0},
{0,0,0,0,0,  0,5,0,0,0,     0,0,0,0,0,  0,1,0,0,0},
{3,0,0,0,0,  0,0,0,2,0,     0,1,0,0,0,  0,0,0,0,0}, //8
{0,0,0,1,0,  0,0,4,0,2,     0,0,0,0,0,  0,0,0,0,0},
{0,3,0,0,0,  0,0,0,4,0,     2,0,0,0,0,  0,0,0,0,0},

{0,0,3,0,0,  0,0,0,0,4,     0,0,0,0,1,  0,0,0,0,0}, //11
{0,0,0,0,0,  0,0,3,0,0,     0,0,2,0,0,  0,0,0,1,0},
{0,0,0,0,0,  0,0,0,0,0,     0,4,0,2,0,  0,0,1,0,0},
{0,0,0,0,3,  0,0,0,0,0,     0,0,4,0,2,  0,0,0,0,0}, //14
{0,0,0,0,0,  0,0,0,0,0,     3,0,0,4,0,  0,0,0,0,1},

{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,0,  0,2,3,4,0},
{0,0,0,0,0,  0,3,0,0,0,     0,0,0,0,0,  4,0,0,0,2}, //17
{0,0,0,0,0,  4,0,0,0,0,     0,0,3,0,0,  1,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,3,0,0,0,  2,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,3,  0,4,0,0,0}};
/**
const int map_of_sizes[20][20] = { // First coordinate is current graph, second is goal graph, and result is distance between dots
{0,0,0,0,0,  0,0,350,0,0,     0,0,0,0,0,  0,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,350,     0,0,0,0,0,  0,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     350,0,0,0,0,  0,0,0,0,0},
{0,0,0,0,0,  0,0,0,330,0,     0,0,0,0,0,  0,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,305,0,  0,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,0,  0,0,450,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,0,  0,360,0,0,0},

{350,0,0,0,0,  0,0,0,710,0,     0,850,0,0,0,  0,0,0,0,0}, //8
{0,0,0,330,0,  0,0,710,0,325,     0,0,0,0,0,  0,0,0,0,0},
{0,350,0,0,0,  0,0,0,325,0,     1035,0,0,0,0,  0,0,0,0,0},
{0,0,350,0,0,  0,0,0,0,1035,     0,0,0,0,850,  0,0,0,0,0}, //11
{0,0,0,0,0,  0,0,850,0,0,     0,0,1020,0,0,  0,0,0,640,0},
{0,0,0,0,0,  0,0,0,0,0,     0,1020,0,335,0,  0,0,370,0,0},
{0,0,0,0,305,  0,0,0,0,0,     0,0,335,0,720,  0,0,0,0,0}, //14
{0,0,0,0,0,  0,0,0,0,0,     850,0,0,720,0,  0,0,0,0,640},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,0,  0,420,380,900,0},
{0,0,0,0,0,  0,360,0,0,0,     0,0,0,0,0,  420,0,0,0,510}, //17
{0,0,0,0,0,  450,0,0,0,0,     0,0,370,0,0,  380,0,0,0,0},

{0,0,0,0,0,  0,0,0,0,0,     0,640,0,0,0,  900,0,0,0,0},
{0,0,0,0,0,  0,0,0,0,0,     0,0,0,0,640,  0,510,0,0,0}};
**/
int paths_matrix[24][8]={ //List of paths from two pairs of nodes
  {0,0,0,0,0,0,0,0},
  {1,8,9,4,9,0,0,0},
  {1,8,12,13,14,5,14,0},
  {1,8,12,13,18,6,18,0},
  {1,8,12,19,16,17,7,17},
  {3,11,10,9,4,9,0,0},
  {3,11,15,14,5,14,0,0},
  {3,11,15,14,13,18,6,18},
  {3,11,15,20,17,7,17,0},
  {4,9,8,1,8,0,0,0}, //9
  {4,9,10,11,3,11,0,0},
  {5,14,13,12,8,1,8,0},
  {5,14,15,11,3,11,0,0},
  {6,18,13,12,8,1,8,0},
  {6,18,13,14,15,11,3,11},
  {7,17,16,19,12,8,1,8},
  {7,17,20,15,11,3,11,0},
  {1,8,9,10,2,10,0,0},
  {3,11,10,2,10,0,0,0},
  {4,9,10,11,15,14,5,14}, //19
  {5,14,13,18,6,18,0,0},
  {6,18,16,17,7,17,0,0},
  {7,17,20,15,11,10,9,4}, // Might cause problems as [current_graph_number+1] undefined for final node
  {2,10,9,4,9,0,0,0}
};

uint8_t current_graph_number = 0; // we always start from second element of array. 
uint8_t current_compass = 1; // In default situation starts from going to the North
uint8_t current_scenario = 1; // Starts from straight line
bool this_is_the_end = false; // Becomes true when we reach final destination and need to reverse or go backwards
bool part_two = false; // Becomes true when robot has completed first run
unsigned long time_of_last_junction_detected;

bool moving;  // True if moving, for flashing LED.
uint8_t current_path[] = {2,10,9,4,9,0,0,0}; // Initial path, will be updated when we reach the end of the path. Maximum length is 8.
uint8_t bay_array[] = {4,5,6,7,2}; // Bays that we need to visit
uint8_t current_bay_number = 0; 

const uint8_t distance_history_length = 10;
float distance_history[distance_history_length];  // Stores the last few readings of the distance sensor, for averaging. (queue structure)
uint8_t distance_history_pointer = 0; // points to head of the queue.
uint8_t distance_history_datapoints = 0; // increases until full

// Comparing latest value to the average:
// robot max speed around 20cm/s, but will be less when going into a bay.
// max deviation = fudge_factor * ((num_samples/2) * max_distance_per_sample (assuming constant speed))
const float max_acceptable_deviation = 3 * (0.5 * distance_history_length * 20 * delay_time / 1000);
float current_wall_distance = 0; // Use this for all distance measurements, updated only when value is acceptable.

int16_t v_left_prev = 0;
int16_t v_right_prev = 0;
int8_t direction_left_prev = 0; // 1 is forwards, 2 is backwards, 0 is not yet defined.
int8_t direction_right_prev = 0;



void new_path_define(int x);
void new_path(int big_goal_graph);
void move(int16_t speed, float rotation_fraction);
void stop();
void button_press_ISR();
void straight_junction();
void simple_mode_of_motion();
void left_junction();
void right_junction();
void backwards();
void straight();
void backwards_left_junction();
void backwards_right_junction();
void stop_and_grab();
void stop_and_release();
void color_detection();
int spike_in_distance();
bool junction_detected();

int16_t current_speed = 0; 
float current_rot_frac = 0; 


void reset(){
  // Reboots the arduino
  CPU_CCP = 0xD8;
  WDT.CTRLA = 0x4;
  while (true){}
}

ISR(TCB0_INT_vect){
  // Timer ISR for flashing blue LED when moving
  flash_led();
  // Clear interrupt flag
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

void measure_distance(){
  // Take distaance reading with ultrasonic sensor.
  distance_history_pointer = (distance_history_pointer + 1) % distance_history_length;
  distance_history[distance_history_pointer] = (analogRead(sensityPin) / ADC_SOLUTION) * MAX_RANG ;
  if (distance_history_datapoints < distance_history_length){
    distance_history_datapoints ++; // increase unless full
  }
  // Serial.println("distance: " + String(distance_history[distance_history_pointer]));
  // Clear interrupt flag
  //TCB1.INTFLAGS = TCB_CAPT_bm;
}

void flash_led(){
  // Flash LED if moving
  if (moving){
    if (digitalRead(LED_Blue)){
      digitalWrite(LED_Blue, 0);
    }
    else{
      digitalWrite(LED_Blue, 1);
    }
  }
  else{
    digitalWrite(LED_Blue, 0);
  }
}

void stop(){
  // Stop the car by releasing both motors
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  moving = false;
}


void button_press_ISR(){
  // Debounce button
  unsigned long new_time = millis();
  if (millis() > (first_press_time + 300)){
    if (mode == 0) {
      mode = 1; 
    } else {
      //mode = 0;
      /*
      stop();
      for(int i = 0; i < 2; i++){
        mech_servo.write(240);
        delayMicroseconds(300);
        mech_servo.write(270);
        delayMicroseconds(300);
      }
      */
      reset();  // To stop the robot, reset arduino.
    }
    first_press_time = new_time;
  }
}




void new_path_define(int x){
  // Update to a new path; Used when current path is completed by car.
	for (int i = 0; i < 8; ++i) {
	current_path[i]=paths_matrix[x][i];
	}
}

void new_path(int big_goal_graph){
  int x = current_path[current_graph_number];
	//Hard coding paths between nodes
  // Update the path according to:
  // 1. Next goal graph big_goal_graph
  // 2. Current position x 
  // bays in the first run, or previous goal graph in the second run, where cubes are not found
	if (x==1){
		if (big_goal_graph==4){
			new_path_define(1);
		}else if (big_goal_graph==5){
			new_path_define(2);
		}else if (big_goal_graph==6){
			new_path_define(3);
		}else if (big_goal_graph==7){
			new_path_define(4);
    }else if (big_goal_graph==2){
      new_path_define(17);
		}else{
			new_path_define(0);
		}}
	if (x==3){
		if (big_goal_graph==4){
			new_path_define(5);
		}else if (big_goal_graph==5){
			new_path_define(6);
		}else if (big_goal_graph==6){
			new_path_define(7);
		}else if (big_goal_graph==7){
			new_path_define(8);
    }else if (big_goal_graph==2){
      new_path_define(18);
		}else{
			new_path_define(0);
		}}
	if (x==4){
		if (big_goal_graph==1){
			new_path_define(9);
		}else if (big_goal_graph==3){
			new_path_define(10);
		}else if (big_goal_graph==5){
			new_path_define(19);
		}else{
			new_path_define(0);
		}}
	if (x==5){
		if (big_goal_graph==1){
			new_path_define(11);
		}else if (big_goal_graph==3){
			new_path_define(12);
		}else if (big_goal_graph==6){
			new_path_define(20);
		}else{
			new_path_define(0);
		}}
	if (x==6){
		if (big_goal_graph==1){
			new_path_define(13);
		}else if (big_goal_graph==3){
			new_path_define(14);
		}else if (big_goal_graph==7){
			new_path_define(21);
		}else{
			new_path_define(0);
		}}
	if (x==7){
		if (big_goal_graph==1){
			new_path_define(15);
		}else if (big_goal_graph==3){
			new_path_define(16);
		}else if (big_goal_graph==4){
			new_path_define(22);
		}else{
			new_path_define(0);
		}
	}
}

void move(int16_t speed, float rotation_fraction) {
  // This function describes every possible motion configuration in the most convinient (I think) way.
  // Speed that you give is the maximal speed of two wheels. 
  // Rotation fraction shows to which extent you rotate. 
  // rotation fraction = 0 => you go forward or backwards. 
  // Rotation fraction -1 means you move clockwise, 1 means you move anticklokwise.
  int16_t other_speed = 0;
  int16_t v_left = 0;
  int16_t v_right = 0;
  int8_t direction_left = 0;  // 1 is forwards, 2 is backwards, 0 is not yet defined.
  int8_t direction_right = 0;
  moving = true;

  if (speed != current_speed || current_rot_frac != rotation_fraction) {

    if (rotation_fraction > 0) {
      other_speed = speed - 2*rotation_fraction*speed;
    } else {
      other_speed = speed + 2*rotation_fraction*speed;
    }

    if (((rotation_fraction>0) && (speed<0)) || ((rotation_fraction<0) && (speed>0))){
      v_left = speed;
      v_right = other_speed;
    } else {
      v_right = speed;
      v_left = other_speed;
    }
    //DEBUG_SERIAL.print("Speed is: ");
    //DEBUG_SERIAL.println(speed);
    //DEBUG_SERIAL.print("Rotation fraction is: ");
    //DEBUG_SERIAL.println(rotation_fraction);
    if (v_left != v_left_prev){
      myMotor1->setSpeed(abs(v_left)); 
      v_left_prev = v_left;
      //DEBUG_SERIAL.print(v_left);
    }
    //DEBUG_SERIAL.print(" ");
    if (v_right != v_right_prev){
      myMotor2->setSpeed(abs(v_right));
      v_right_prev = v_right;
      //DEBUG_SERIAL.print(v_right);
    }
    

    
    
    

    if (v_left > 0){
      direction_left = 1;
    } else{
      direction_left = 2;
    }
    if (v_right > 0){
      direction_right = 1;
    } else{
      direction_right = 2;
    }

    if (mode != 0) {
      if (direction_left != direction_left_prev){
        //DEBUG_SERIAL.print(".");
        if (v_left > 0) {
          myMotor1->run(FORWARD);
        } else {
          myMotor1->run(BACKWARD);
        }
        direction_left_prev = direction_left;
      }
      if (direction_right != direction_right_prev){
        //DEBUG_SERIAL.print(".");
        if (v_right > 0) {
          myMotor2->run(FORWARD);
        } else {
          myMotor2->run(BACKWARD);
        }
        direction_right_prev = direction_right;
      }
      //DEBUG_SERIAL.println("");
    } else {
        stop();
    }
    current_rot_frac = rotation_fraction;
    current_speed = speed;
  }
}

void last_bay(){
  straight_junction(); // Ends when white line is over
  //delay(500);	//Keeps driving for 500ms
  measure_distance();
  while (distance_history[distance_history_pointer] > distances_from_bays[1]){
    delay(1);
    measure_distance();
  }
  stop();
  delay(5000);	//Stops for 5s
  this_is_the_end = true; 
  current_compass = 1; //change compass by 180 degrees or from 3 to 1
  move(-main_speed, 0.0);
  delay(1500);
  new_path_define(23);
  current_graph_number = 0;
  current_bay_number=0;
  part_two = true;
  
}

void straight_junction(){ // This function must go on as long as you are in the junction
  //bool sensor_sequence[] = {1,0,1,0,1};
  bool sensor_sequence[] = {1,0,1};
  if (this_is_the_end == false){
    DEBUG_SERIAL.println("Go straight in junction");
    while ((digitalRead(sensorFarRight) == 1) || (digitalRead(sensorFarLeft) == 1)) {
      straight(); 
      //DEBUG_SERIAL.println("on it!");
    }
    DEBUG_SERIAL.println("done?");
  } else if(current_path[current_graph_number] == 8 || current_path[current_graph_number] == 1) { // Turn 180 degrees anticlockwise at a T-junction, i.e. stop turning after crossing second white line.
    move(-main_speed, -1);
    for (int i = 0; i < 3; i++){
      while (digitalRead(sensorLeft) == sensor_sequence[i]) {
        delay(10); // delay in case of "bounce" in line sensor readings.
      }
    }
    this_is_the_end = false;
    for (int i = 0; i < 500/delay_time; i ++) {
        straight();
        delay(delay_time);
      }
    
  } else {  // Turn 180 degrees clockwise at a T-junction, i.e. stop turning after crossing second white line.
    DEBUG_SERIAL.println("Turning 180");
    if (current_path[current_graph_number] == 2) {
      delay(500);
    } else {
      move(-main_speed, 1);
      for (int i = 0; i < 3; i++){
        while (digitalRead(sensorRight) == sensor_sequence[i]) {
          delay(10); // delay in case of "bounce" in line sensor readings.
        }
      }
      this_is_the_end = false;
      for (int i = 0; i < 500/delay_time; i ++) {
          straight();
          delay(delay_time);
        }
    }

    
  }
  //DEBUG_SERIAL.println("Straight junction is done");
  delay(delay_time);
}

void simple_mode_of_motion(){
  // Determine the mode of motion to take using:
  // 1. Current graph (junction that is to be reached by car)
  // 2. Next graph (next junction to go once the current graph is reached)
  // Current compass and next compass is used to determine where to turn

  // Navigation debug system
  int y = better_map_of_directions[current_path[current_graph_number]-1][current_path[current_graph_number+1]-1];
  DEBUG_SERIAL.print("Current graph: ");
  DEBUG_SERIAL.println(current_path[current_graph_number]);
  DEBUG_SERIAL.print("Next graph: ");
  DEBUG_SERIAL.println(current_path[current_graph_number + 1]);
  DEBUG_SERIAL.print("Goal compass: ");
  DEBUG_SERIAL.println(y);
  DEBUG_SERIAL.print("Current compass: ");
  DEBUG_SERIAL.println(current_compass);

  // Junction actions to take according to different conditions
  if (current_path[current_graph_number] == 19 || current_path[current_graph_number] == 20 || ((current_path[current_graph_number] == 8 && current_path[current_graph_number-1] == 1) || (current_path[current_graph_number] == 11 && current_path[current_graph_number - 1] == 3)) && (y == 1)){
    DEBUG_SERIAL.println("Skip this junction");
    if ((current_path[current_graph_number] == 8 || current_path[current_graph_number] == 11) && (y == 1)) {
      for (int i = 0; i < 500/delay_time; i++) {
        straight();
        delay(delay_time);
      }
    }
  } else if (current_path[current_graph_number] == 2 && current_compass == 3) {
    last_bay();
  } else if ((4 + y - current_compass) % 4 == 3) { // Turn left
    DEBUG_SERIAL.print("Left junction is started... ");
    left_junction();
    DEBUG_SERIAL.println("Left junction is done!!!");
  } else if ((4 + y - current_compass) % 4 == 1) {// Turn right
    DEBUG_SERIAL.print("Right junction is started... ");
    right_junction();
    DEBUG_SERIAL.println("Right junction is done!!!");
  } else if ((4 + y - current_compass ) % 4 == 0) { // Go straight
    straight_junction();
  } else { // Go backwards
    this_is_the_end = true;
    if ((current_path[current_graph_number] != 1 && current_path[current_graph_number] != 3)){
      DEBUG_SERIAL.print("NOW WE GO BACKWARDS");
      DEBUG_SERIAL.println(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);
      stop_and_grab();
      DEBUG_SERIAL.println("Picking up a block...");
      color_detection();
      DEBUG_SERIAL.println(analogRead(sensityPin) * MAX_RANG / ADC_SOLUTION);
      DEBUG_SERIAL.print("NOW WE GO BACKWARDS");
    } else {
      stop_and_release();
      // stop and release
      DEBUG_SERIAL.println("At the end, releasing block...");
      } 
    }

}

void left_junction(){ // This function must go on as long as you are in the junction
  if (this_is_the_end == false) {
    if (current_path[current_graph_number] == 8 || current_path[current_graph_number] == 11 || current_path[current_graph_number] == 12 || current_path[current_graph_number] == 15) {
      move(main_speed, -1.0); // Rotation fraction can be changed according to actual performance
    }
    else {
      move(main_speed, -0.7);
    }

    // process of turning determination; involves Left sensor detects whether it has been crossing the destination line of turning left
    while (digitalRead(sensorLeft) == 1) {}
    while (digitalRead(sensorLeft) == 0) {} // While right sensor is outside of its first line, move it to the line
    while (digitalRead(sensorLeft) == 1) {}

    //Extra Delay added for the car to deal with overshoot problem
    move(main_speed, 0.7); // Rotation fraction can be changed according to actual performance
    delay(30); // delay time can also be changed

    
    // to cope with the problem of detecting two extra junctions previously;
    // probably not needed anymore
    //if (current_path[current_graph_number] == 6 || current_path[current_graph_number] == 7 || current_path[current_graph_number] == 12){ //Problematic bays, please wait before making any decisions //CHANGED
    if (true){
      for (int i = 0; i < 500/delay_time; i++) {
        straight();
        delay(delay_time);
      }
    }
  } else {
    backwards_left_junction();
  }
}

void right_junction(){ // This function must go on as long as you are in the junction
  if (this_is_the_end == false) {
    if (current_path[current_graph_number] == 8 || current_path[current_graph_number] == 11 || current_path[current_graph_number] == 12 || current_path[current_graph_number] == 15) {
      move(main_speed, 1.0);
    }
    else {
      move(main_speed, 0.7);
    }

    // process of turning determination; involves Right sensor detects whether it has been crossing the destination line when turning right
    while (digitalRead(sensorRight) == 1) {}
    while (digitalRead(sensorRight) == 0) {}
    while (digitalRead(sensorRight) == 1) {} 
    move(main_speed, 0.7);
    delay(30);
    //if (current_path[current_graph_number] == 6 || current_path[current_graph_number] == 7 || current_path[current_graph_number] == 12){ //Problematic bays, please wait before making any decisions
    if (true){
      for (int i = 0; i < 500/delay_time; i++) {
        straight();
        delay(delay_time);
      }
    }

  } else {
    backwards_right_junction();
  }
}


void backwards(){
  bool right = digitalRead(sensorRight);
  bool left = digitalRead(sensorLeft);  
  if (right && !left) { //M fgf ove left
  // rotate fraction it set to a small value to ensure robot corrects iteself instead of purely rotating
    move(-main_speed, 0.1); // Anti-Clockwise
  } else if (!right && left) { //Move to the right
    move(-main_speed, -0.1); // Clockwise
  } else if (!right && !left) { // Includes both going 
    move(-main_speed, 0);
  } else { // Both are white, so we need time delay and going straightforward for short period of time ignoring all sensors. 
    for (int i = 0; i < 125/delay_time; ++i) {
      move(-main_speed, 0);
      delay(delay_time);
    }
  }
}

void straight(){ // Regular function for going straightforward
  uint8_t sp = main_speed; // Speed of the car is defined by main_speed by default
  if (number_of_connections[current_path[current_graph_number]-1] == 1 && (current_path[current_graph_number] != 2) && (current_path[current_graph_number] != 1) && (current_path[current_graph_number] != 3)) {
    sp = 200; // An exception for the a path tested, where the speed is reduced to 200
  }
  bool right = digitalRead(sensorRight); // Read the sensor
  bool left = digitalRead(sensorLeft);
  
  if (this_is_the_end == false) { // If we are not at the end of the path
  // implementation of the car's line-following algorithm; 
    if (right && !left) { //Move right
      move(sp, 0.35);
    } else if (!right && left) { //Move to the left
      move(sp, -0.35);
    } else if (!right && !left) { // Includes both going 
      move(sp, 0.0);
    } else { // Both are white, so we need time delay and going straightforward for short period of time ignoring all sensors. 
      for (int i = 0; i < 125/delay_time; ++i) {
        move(sp, 0);
        delay(delay_time);
      }
    }
  } else { // If we are at the end of the path, go backwards
    backwards();
  }
}

void backwards_left_junction(){ // Rotate clockwise until certain results
  DEBUG_SERIAL.print("You have entered the left junction");

  float rot = 1.0; 
  if (current_path[current_graph_number] == 17 && current_compass == 1 || current_path[current_graph_number] == 14 && current_compass == 1) {
    // if the robot is at the problematic bays, the rotation fraction is set to a smaller value to ensure the robot corrects itself instead of purely rotating
    rot = 0.75;
  }
  rot = 0.75;

  // another way of implementing turning while detecting if the turning process is completed.
  while (digitalRead(sensorRight) == 1) { 
    move(main_speed, rot);
    delay(delay_time);
  }
  delay(5);
  while (digitalRead(sensorRight) == 0) {
    move(main_speed, rot);
    delay(delay_time);
  }
  delay(5);
  while (digitalRead(sensorRight) == 1) {
    move(main_speed, rot);
    delay(delay_time);
  }

  // Tell the robot: you don't need to take backward action until another end is met
  this_is_the_end = false;
    time_of_last_junction_detected = millis(); // Update the time of last junction detected
  while (millis() - time_of_last_junction_detected < 500) { // Go straight for 500ms
    straight();
  }
}

void backwards_right_junction(){ // Rotate anticlockwise until certain reusult.

  // similar to the previous function, but the rotation fraction is set to a negative value for opposite rotation
  
  float rot = -1.0; 
  if (current_path[current_graph_number] == 17 && current_compass == 1 || current_path[current_graph_number] == 14 && current_compass == 1) {
    rot = -0.75;
  }
  rot = -0.75;

  DEBUG_SERIAL.print("You have entered the right junction");
  // another way of implementing turning while detecting if the turning process is completed.
  while (digitalRead(sensorLeft) == 1) { 
    move(main_speed, rot);
    delay(delay_time);
  }
  delay(5);
  while (digitalRead(sensorLeft) == 0) {
    move(main_speed, rot);
    delay(delay_time);
  }
  delay(5);
  while (digitalRead(sensorLeft) == 1) {
    move(main_speed, rot);
    delay(delay_time);
  }
  // Tell the robot: you don't need to take backward action until another end is met
  this_is_the_end = false;
  time_of_last_junction_detected = millis(); // Update the time of last junction detected
  while (millis() - time_of_last_junction_detected < 500) { // Go straight for 500ms
    straight();
  }
}

void stop_and_grab(){
  // actions to take:
  // 1. stop the car from moving
  // 2. tell servo to grab the block
  stop();
  DEBUG_SERIAL.println("grabbing block");


  // set the default position of the servo
  int pos = 270;
  

  // slow the servo down to grab the block to make grabbing process not screwed up by hurrying
  for (pos; pos >=5; pos -= 1) { // hand goes from 270 degrees to 5 degrees, grabbing the block
    // in steps of 1 degree
    // DEBUG_SERIAL.println("current pos: " + String(pos));
    mech_servo.write(pos);              // tell servo to go to position in variable 'pos'
    if (pos > 45){
      delay(5);
    }else{
      delay(15);
    }
    //if(pos%10==0){
    //  DEBUG_SERIAL.println("written mech done") ;}                      // waits 15 ms for the servo to reach the position
  }


  DEBUG_SERIAL.println("Block was grabbed");
}


void stop_and_release(){
  // actions to take:
  // 1. stop the car from moving
  // 2. tell servo to release the block
  stop();

  // fast release. The position value is renewed fastly and finish the function execution before the servo reaches the position
  // the servo then tried the match the position AFTER the pos value is renewed to 270 (when the next function is taking place)
  // this works well as it saves time by taking most of hand's position reset motion to backwarding process; no need to extra coding in backward motion
  for (int pos = 5; pos <= 270; pos += 44) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    mech_servo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15 ms for the servo to reach the position
  }

  // The LED color indicator is dimmed
  digitalWrite(LED_Red, 0);
  digitalWrite(LED_Green, 0);

  // Taking another new path by updating a new current bay number to take
  if (current_path[current_graph_number]==7){
	  current_bay_number=0;
  }else{
  	current_bay_number = current_bay_number + 1;
  }
  // update the path to the next bay
  new_path(bay_array[current_bay_number]);
  current_graph_number = 0;

  // detect whether it is at the dropping point. If so, the robot will take a 180 degree turn
  int y = better_map_of_directions[current_path[current_graph_number]-1][current_path[current_graph_number+1]-1];
  if (y == 1 && current_path[current_graph_number] != 5 && current_path[current_graph_number] != 7) { // Do turning or smth
    move(-main_speed, 0);
    delay(400);
    straight_junction();
  }
}


void color_detection() {
  // actions to take:
  // 1. detect the color;
  // 2. lighting up the LED according to the color detected;
  // 3. update the path according to the color detected
  // different stretegy for the first run and the second run
  delay(500);
  uint16_t r, g, b, c, colorTemp, lux;
  unsigned long start_time = millis();

  // detect the color of the block
  while (millis() - start_time < 3000){

    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);
    //DEBUG_SERIAL.print("colour: (lux, temp) ");
    //DEBUG_SERIAL.print(lux);
    //DEBUG_SERIAL.print(", ");
    //DEBUG_SERIAL.println(colorTemp);

    if (lux > 4000) {
      DEBUG_SERIAL.println("red");
      new_path(3);
      current_graph_number = 0;
      digitalWrite(LED_Red, 1);
      delay(5000);
      return;
    } else if (lux < 4000 && colorTemp > 10000) { // change to less than 4000?
      DEBUG_SERIAL.println("black");
      new_path(1);
      current_graph_number = 0;
      digitalWrite(LED_Green, 1);
      delay(5000);
      return;
    } else {
      //DEBUG_SERIAL.println("no block");
      // add wait 10 seconds then decide no block
      delay(100);
    }
  }

  // At the second run, if the robot cannot detect the color, it will assume the block is absent and go to the next bay
  if (part_two){
	stop_and_release();
  }

  // At the first run, if the robot cannot detect the color, it will assume the block is absent and go to the dropping point
  else{
  	DEBUG_SERIAL.println("can't tell colour, guess black");
  	new_path(1);
  	current_graph_number = 0;
  	digitalWrite(LED_Green, 1);
  	//digitalWrite(LED_Red, 1);
  	delay(5000);
  }
  
}

/**void reverse(){
  move(main_speed, 1);
  while (digitalRead(sensorRight) == 1) {}
  while (digitalRead(sensorRight) == 0) {}
  DEBUG_SERIAL.println("You have entered the line after the reverse junction");
  while (digitalRead(sensorRight) == 1) {}
} **/ // If you ever decide to turn by 180 degrees call this function


/* deprecated function to deal with spike value read by the ultrasonic sensor
it is not used in the final version of the code, as it is no longer a problem for the robot

int spike_in_distance(){
  // Calculates if the ultrasonic sensor reading has spiked due to it seeing the cube instead of the wall.
  // Returns +1 is distance has increased significantly, -1 if decreased, and 0 otherwise.
  // length of distance history and sample period must be set to optimal value:
  //  If either of these are too short, by the time the code checks, may have been changed for a while.
  //  If too long, when correcting, may not see two changes occuring within a short time period.
  float average = 0;
  float recent_average = 0;
  for (int i = 0; i < distance_history_datapoints; i++){
    average += distance_history[i];
  }
  average /= distance_history_datapoints; // calculate mean
  current_wall_distance = distance_history[distance_history_pointer];
  if ((current_wall_distance - average) > max_acceptable_deviation){
    // Distance has suddenly increased, need to reset the history so that further changes can be detected
    distance_history[0] = current_wall_distance; // use last value as first in the reset history
    distance_history_pointer = 1;
    distance_history_datapoints = 1;
    return 1;
  } else if ((current_wall_distance - average) < -max_acceptable_deviation){
    // Distance has decreased significantly, same process as above.
    distance_history[0] = current_wall_distance; // use last value as first in the reset history
    distance_history_pointer = 1;
    distance_history_datapoints = 1;
    return -1;
  } else{
    return 0;
  }
  
}
*/


bool junction_detected(){
  // determine if the car has reached a junction
  // if so, return true, otherwise return false
  bool is_a_bay = ((number_of_connections[current_path[current_graph_number]-1] == 1) && (current_path[current_graph_number] != 2) && (current_path[current_graph_number] != 1) && (current_path[current_graph_number] != 3) 
  || (current_path[current_graph_number] == 16 && current_compass == 1) 
  || (current_path[current_graph_number] == 8 && current_compass == 4) 
  || (current_path[current_graph_number] == 12 && current_compass == 4) 
  || (current_path[current_graph_number] == 16 && current_compass == 1) 
  || (current_path[current_graph_number] == 15 && current_compass == 2) 
  || (current_path[current_graph_number] == 11 && current_compass == 2));  // (and not the starting point)
  if (((digitalRead(sensorFarRight) || digitalRead(sensorFarLeft)))|| (is_a_bay && current_wall_distance < distances_from_bays[current_path[current_graph_number]-1]) || ((current_path[current_graph_number] == 1 || current_path[current_graph_number] == 3) && digitalRead(sensorLeft) && digitalRead(sensorRight))) {
    return true;
  } else {
    return false; 
  }
}


void setup() {
  // Initialize the motor shield
  pinMode(NINA_RESETN, OUTPUT);
  digitalWrite(NINA_RESETN, LOW);
  DEBUG_SERIAL.begin(115200); // Start DEBUG_SERIAL communication
  DEBUG_SERIAL.println("Starting...");

  // Set sensor pins as input
  pinMode(sensorFarLeft, INPUT);
  pinMode(sensorLeft, INPUT);
  pinMode(sensorRight, INPUT);
  pinMode(sensorFarRight, INPUT);
  pinMode(button, INPUT);
  
  // Set LED pins as output
  pinMode(LED_Red, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  pinMode(LED_Blue, OUTPUT);
  digitalWrite(LED_Red, 0);
  digitalWrite(LED_Green, 0);
  digitalWrite(LED_Blue, 0);
  // Set up the servo
  mech_servo.attach(servo_pin);
  // Start value of the servo
  mech_servo.write(270); // start vertically, angle is anticlockwise from just past the closed position.
  pinMode(loop_speed_test_pin, OUTPUT);
  digitalWrite(loop_speed_test_pin, 0);
  

  // Initialize the Motor Shield
  if (!AFMS.begin()) {
    DEBUG_SERIAL.println("Motor Shield not found.");
    while (1); // Halt if shield not found
  }
  DEBUG_SERIAL.println("Motor Shield initialized.");

  // Button Interrupt:
  attachInterrupt(digitalPinToInterrupt(button), button_press_ISR, RISING);

  // Timer Interrupt for LED flashing:
  // Timer A (used as clock for B) clocked at 250kHz.
  TCB0.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
  TCB0.CCMP = 62500; // Value to compare with. 62500 gives 2Hz.
  TCB0.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer

  // Timer interrupt for taking distance readings:
  //TCB1.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
  //TCB1.CCMP = 125 * ULTRASONIC_SAMPLE_PERIOD; // Value to compare with. 125 gives 1ms
  //TCB1.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
  //TCB1.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer

  current_compass = 1;
}

void loop() {
  // Read the sensors
  bool farLeft = digitalRead(sensorFarLeft);
  bool left = digitalRead(sensorLeft);
  bool right = digitalRead(sensorRight);
  bool farRight = digitalRead(sensorFarRight);

  // Main loop
  if (mode != 0) {  
    measure_distance(); // Take distance reading
    current_wall_distance = distance_history[distance_history_pointer];
    straight();
    if (junction_detected()){ // When junction is detected, we need to 1) Do the junction to certain side, 2) Change the compass and 3) Change certain graph and 
      Serial.println(current_path[current_graph_number]);
      time_of_last_junction_detected = millis();
      simple_mode_of_motion(); // This function does corresponding turn and ends when the junction is done
      current_compass = better_map_of_directions[current_path[current_graph_number]-1][current_path[current_graph_number + 1]-1];
      current_graph_number = current_graph_number + 1; // Because finished simple_mode_of_motion means that we have gone through the graph and we need to get to the new graph
    }
    

    delay(delay_time); // Delay to prevent the loop from running too fast

    // blinking the LED to test the speed of the loop
    digitalWrite(loop_speed_test_pin, 1);
    digitalWrite(loop_speed_test_pin, 0);
  } else {
    stop();
  }


}
