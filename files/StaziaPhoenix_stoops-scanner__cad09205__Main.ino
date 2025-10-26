#include "Linescanner.h"
#include "Servo.h"
#include "PIDController.h"
#include "math.h"

#define END 129
#define NONE 130
#define NOLINE 131
#define center 90
#define left 150
#define right 30
#define setpoint 64
#define adj 65
#define numPixels 128
#define leftThresh 54
#define rightThresh 74

#define sync 12
#define CLK 11
#define data A0
#define motor 3
#define led 13
#define servoPin 9

byte debug = 0;
byte go = 0;
byte lineWidth;

int pixels[numPixels];
byte digital[numPixels];

/* PID info */
PIDController controller;
float error;
byte prev_angle = 0;

const unsigned int expose = 7390;
Servo servo;
byte angle = 90;

//int trigger = 2;
int threshold;
byte speedy = 0;

// Byte (one character) read in from Virtual Serial COMM (Bluetooth)
byte inByte = 0;

// Acknowledgment variable (while-loop sentinel)
byte ack = 0;

Linescanner cam(CLK,sync,data);

void setup() {
  //attachInterrupt(digitalPinToInterrupt(trigger), killSwitch, CHANGE);
  
  pinMode(CLK,OUTPUT);
  pinMode(sync,OUTPUT);
  
  // Configure digital for OUTPUT
  pinMode(led,OUTPUT);
  digitalWrite(led,HIGH);
  
  analogWrite(motor,0);
  // Open up Bluetooth Virtual Serial COMM port
  Serial.begin(9600);

  servo.attach(servoPin);
  servo.write(angle);

  Serial.print("LineWidth = ");
  Serial.println(lineWidth = setLineWidth());
}

byte getAck() {
  Serial.write("\r\nPress <c> for command list\r\n");
    Serial.write(">");
    byte inByte = Serial.read();

    if ( inByte == 'c' ) {
      Serial.write(inByte);
      ack = 1;
      printNewLn();
      printCmdList();
      printNewCmdLn();
      return 1;
    }   
    return 0;
}

void loop() {
  while (!ack) {
    ack = getAck();    
    delay(3000);
  }

  doSerialCmd(getSerialCmd());
//  Serial.println(digitalRead(trigger));
}



// ---------------- CONTROL ---------------- //

/*
 * filters the analog data from Linescanner into digital HIGH/LOW, where
 * high indicates the line and low occurs else
 */
void filter() {
  for (int i = 0; i < numPixels; i++) {
    if (pixels[i] > threshold) {
      digital[i] = 1;
    } else {
      digital[i] = 0;
    }
  }
}

void getLine() {
  cam.scan(expose);
  cam.read(pixels);
  threshold = cam.calibrate(expose, pixels); // position 2
  filter();
  if (debug) {
    cam.printLine(digital);
  }
}

int setLineWidth() {
  getLine();

  int leftIdx = findLeftEdge(0);
  int rightIdx = findRightEdge(numPixels-1);

  return rightIdx - leftIdx;
}

void calibrate() {
  go = 0;
  while (!doSerialCmd(getSerialCmd())) {
    PID();
  }
}

void run() {
  go = 1;
  while(!doSerialCmd(getSerialCmd())) {
    PID();
  }
}

float calcError(int process_var) {
  int error_pix = process_var-setpoint;
  return atan2(error_pix,adj)*(180/3.141592);
}

bool checkCases( int leftIdx, int rightIdx) {
  if (debug) {
    Serial.println("leftIdx, rightIdx = ");
    Serial.println(leftIdx);
    Serial.println(rightIdx);
    Serial.println(rightIdx-leftIdx);
  }
  
  // lost sight of the line, stay turning in previous direction
  if (leftIdx == NOLINE || (rightIdx-leftIdx) > 80) {
    if (angle > 90) {
      servo.write(left);
      if (debug) {
        Serial.println("I am running left!");
      }
    } else {
      servo.write(right);
      if (debug) {
        Serial.println("I am running right!");
      }
    }
    return true;
  }

  // intersection, stay true
  if ( (rightIdx-leftIdx) > (lineWidth+2) ) {
    if (debug) {
      Serial.println("Intersection");
    }
    return true;
  }
  
  // if none of these, adjust according to PID
  return false;
}

void adjustSpeed(int derror) {
  if ((abs(derror) < 25)) {
    analogWrite(motor,77); // 30%
  } else {
    //analogWrite(motor,51); // 20%
    analogWrite(motor,0);
  }
}

int findRightEdge(int startIdx) {
  if (digital[startIdx] == 1 && digital[startIdx-1]== 1) {
    //Serial.println("At end end of right");
    return 128;
  }
  for (int i = startIdx; i > 4; i--) {
    if (digital[i] == 0 && digital[i-1] == 1 && digital[i-2] == 1 && 
        digital[i-3] == 1) {
      return i-1;
    }
  }
  // if no line
  //Serial.println("Could not find right edge");
  return NOLINE;
}

int findLeftEdge(int startIdx) {
  if (digital[startIdx] == 1 && digital[startIdx+1] == 1) {
    //Serial.println("At end end of left");
    return 0;
  }
  for (int i = startIdx; i < numPixels-4; i++) {
    if (digital[i] == 0 && digital[i+1] == 1 && digital[i+2] == 1 && 
        digital[i+3] == 1) {
      return i+1;
    }
  }
  // if no line
  //Serial.println("Could not find left edge");
  return NOLINE;
}

void PID() {
  getLine();

  int leftIdx = findLeftEdge(0);
  int rightIdx = findRightEdge(numPixels-1);

  if (checkCases(leftIdx, rightIdx)) { return; }
  
  error = calcError((rightIdx-leftIdx)/2 + leftIdx);

  angle = 90 + controller.pid(error, speedy);

  if (go) {
    analogWrite(motor, speedy);
  }
  if (debug) {
     Serial.print("Angle  = ");
     Serial.println(angle);
  }
  //if (go) { adjustSpeed(angle-prev_angle); }

  servo.write(angle);
  prev_angle = angle;
}

// ---------------- END CONTROL ---------------- //

// ---------------- COMMANDS ---------------- //

// Prints a modified new line
void printNewLn() {
  Serial.write("\r\n");
}


// Prints a new command line cursor
void printNewCmdLn() {
  printNewLn();
  Serial.write(">");
}

void incPid_t(byte feed) {
  if (feed == 'p') {
    controller.incP_t();
    Serial.print("kP_t is ");
    Serial.println(controller.getP_t());
  } else if (feed == 'd') {
    controller.incD_t();
    Serial.print("kD_t is ");
    Serial.println(controller.getD_t());
  }
}

void decPid_t(byte feed) {
  if (feed == 'p') {
    controller.decP_t();
    Serial.print("kP_t is ");
    Serial.println(controller.getP_t());
  } else if (feed == 'd') {
    controller.decD_t();
    Serial.print("kD_t is ");
    Serial.println(controller.getD_t());
  }
}

void incPid_l(byte feed) {
  if (feed == 'p') {
    controller.incP_l();
    Serial.print("kP_l is ");
    Serial.println(controller.getP_l());
  } else if (feed == 'd') {
    controller.incD_l();
    Serial.print("kD_l is ");
    Serial.println(controller.getD_l());
  }
}

void decPid_l(byte feed) {
  if (feed == 'p') {
    controller.decP_l();
    Serial.print("kP_l is ");
    Serial.println(controller.getP_l());
  } else if (feed == 'd') {
    controller.decD_l();
    Serial.print("kD_l is ");
    Serial.println(controller.getD_l());
  }
}

void centerServo() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.print("Servo alignment: CENTER "); Serial.print(center); Serial.println();
  servo.write(center);
}

void leftServo90() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.print("Servo alignment: LEFT "); Serial.print(left); Serial.println();
  servo.write(left);
}

void rightServo90() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("Servo alignment: RIGHT "); Serial.write(right); Serial.write("\n\r");
  servo.write(right);
}

//// Turns LED ON and writes to Serial
void pwm100() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    motor is ON!");
  analogWrite(motor,speedy);
}
//
//// Turns LED OFF and writes to Serial
//void pwm90() {
//  digitalWrite(led, LOW);
//  delay(300);
//  digitalWrite(led, HIGH);
//  Serial.write("    PWM 230 (90%) motor is ON!");
//  analogWrite(motor,230);
//}
//
//// Turns LED OFF and writes to Serial
//void pwm80() {
//  digitalWrite(led, LOW);
//  delay(300);
//  digitalWrite(led, HIGH);
//  Serial.write("    PWM 204 (80%) motor is ON!");
//  analogWrite(motor,204);
//}
//
//// Turns LED OFF and writes to Serial
//void pwm70() {
//  digitalWrite(led, LOW);
//  delay(300);
//  digitalWrite(led, HIGH);
//  Serial.write("    PWM 179 (70%) motor is ON!");
//  analogWrite(motor,179);
//}
//
//// Turns LED OFF and writes to Serial
//void pwm60() {
//  digitalWrite(led, LOW);
//  delay(300);
//  digitalWrite(led, HIGH);
//  Serial.write("    PWM 153 (60%) motor is ON!");
//}
//
//// Turns LED OFF and writes to Serial
//void pwm50() {
//  digitalWrite(led, LOW);
//  delay(300);
//  digitalWrite(led, HIGH);
//  Serial.write("    PWM 128 (50%) motor is ON!");
//  analogWrite(motor,128);
//}

////// Turns LED OFF and writes to Serial
//void pwm40() {
//  digitalWrite(led, LOW);
//  delay(300);
//  digitalWrite(led, HIGH);
//  Serial.write("    PWM 102 (40%)");
//  speedy = 102;
//}

// Turns LED OFF and writes to Serial
void pwm30() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 77 (30%)");
  speedy = 77;
}

// Turns LED OFF and writes to Serial
void pwm20() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 51 (20%)");
  speedy = 51;
}

// Turns LED OFF and writes to Serial
void pwm10() {
  digitalWrite(led, LOW);
  delay(300);
  digitalWrite(led, HIGH);
  Serial.write("    PWM 26 (10%)");
  speedy = 26;
}

// Turns LED OFF and writes to Serial
void pwm0() {
  go = 0;
  controller.setGo(0);
  digitalWrite(led, LOW);
  Serial.write("    PWM 0 (0%) motor is OFF!");
  analogWrite(motor,(speedy = 0));
}

// Prints the command list
void printCmdList() {
  Serial.write("COMMANDS:\r\n");
  //Servo
  Serial.write("   SERVO:\r\n");
  Serial.write("      <w>    Straight dir\r\n");
  Serial.write("      <a>    Left dir\r\n");
  Serial.write("      <d>    Right dir\r\n");

  //Serial.write("\r\n");

  //motor
  Serial.write("   motor:\r\n");
  Serial.write("      <o>    PWM 255 (100%)\r\n");
//  Serial.write("      <9>    PWM 230  (90%)\r\n");
//  Serial.write("      <8>    PWM 204  (80%)\r\n");
//  Serial.write("      <7>    PWM 179  (70%)\r\n");
//  Serial.write("      <6>    PWM 153  (60%)\r\n");
//  Serial.write("      <5>    PWM 128  (50%)\r\n");
//  Serial.write("      <4>    PWM 102  (40%)\r\n");
//  Serial.write("      <3>    PWM  77  (30%)\r\n");
//  Serial.write("      <2>    PWM  51  (20%)\r\n");
//  Serial.write("      <1>    PWM  26  (10%)\r\n");  
//  Serial.write("      <0>    PWM   0   (0%)\r\n");  
  
  Serial.write("      <c>    Command List\r\n"); 
  Serial.write("      <s>    run\r\n");
  Serial.write("      <b>    Calibrate\r\n");
  
  Serial.write("      <l>    Increase p_t by 1%\r\n");
  Serial.write("      <k>    Decrease p_t by 1%\r\n");
  Serial.write("      <m>    Inc d_t by 10%\r\n");
  Serial.write("      <n>    Dec d_t by 10%\r\n");
  Serial.write("      <u>    Increase p_l by 1%\r\n");
  Serial.write("      <y>    Decrease p_l by 1%\r\n");
  Serial.write("      <j>    Inc d_l by 1%\r\n");
  Serial.write("      <h>    Dec d_l by 1%\r\n");
  Serial.write("      <x>    Inc speed by 1 tick\r\n");
  Serial.write("      <z>    Dec speed by 1 tick\r\n");
  Serial.write("      <g>    Toggle debug statements\r\n");
  Serial.write("       >     Inc controller threshold by 0.1\r\n");
  Serial.write("       <     Dec controller threshold by 0.1\r\n");

  Serial.write("\n");
}

// Performs serial command based on user input
bool doSerialCmd( byte cmd ) {
  switch( cmd ) {
    // Turn LED HIGH
//    case('h'):
//      setPwmFrequency(motor, (mdivisor = mdivisor*2));
//      printNewCmdLn();
//      break;
    case ('w'):
      centerServo();
      printNewCmdLn();
      break;
    case ('d'):
      rightServo90();
      printNewCmdLn();
      break;
    case ('a'):
      leftServo90();
      printNewCmdLn();
      break;
    case ('1'):
      pwm10();
      printNewCmdLn();
      break;
    // Turn LED LOW
    case ('2'):
      pwm20();
      printNewCmdLn();
      break;
    // Turn LED LOW
    case ('3'):
      pwm30();
      printNewCmdLn();
      break;
    // Turn LED LOW
//    case ('4'):
//      pwm40();
//      printNewCmdLn();
//      break;
    // Turn LED LOW
//    case ('5'):
//      pwm50();
//      printNewCmdLn();
//      break;
//    // Turn LED LOW
//    case ('6'):
//      pwm60();
//      printNewCmdLn();
//      break;
//    // Turn LED LOW
//    case ('7'):
//      pwm70();
//      printNewCmdLn();
//      break;
//    // Turn LED LOW
//    case ('8'):
//      pwm80();
//      printNewCmdLn();
//      break;
//    // Turn LED LOW
//    case ('9'):
//      pwm90();
//      printNewCmdLn();
//      break;      
    case ('o'):
      pwm100();
      printNewCmdLn();
      break;      
    case ('0'):
      pwm0();
      printNewCmdLn();
      break;
    // Display CMD List
    case ('c'):
      printCmdList();
      printNewCmdLn();
      break;
    case ('s'):
      run();
      printNewCmdLn();
      break;
    case ('b'):
      calibrate();
      printNewCmdLn();
      break;
    case ('l'):
      incPid_t('p');
      printNewCmdLn();
      break;
    case ('k'):
      decPid_t('p');
      printNewCmdLn();
      break;
    case ('m'):
      incPid_t('d');
      printNewCmdLn();
      break;
    case ('n'):
      decPid_t('d');
      printNewCmdLn();
      break;
    case ('u'):
      incPid_l('p');
      printNewCmdLn();
      break;
    case ('y'):
      decPid_l('p');
      printNewCmdLn();
      break;
    case ('j'):
      incPid_l('d');
      printNewCmdLn();
      break;
    case ('h'):
      decPid_l('d');
      printNewCmdLn();
      break;
    case ('x'):
      if (speedy < 102) {
        speedy = speedy + 1;
      }
      Serial.print("Speed is ");
      Serial.println(speedy);
      printNewCmdLn();
      break;
    case ('z'):
      if (speedy > 0) {
        speedy = speedy - 1;
      }
      Serial.print("Speed is ");
      Serial.println(speedy);
      printNewCmdLn();
      break;
    case ('e'):
      return true;
    case ('g'):
      debug = debug^1;
      Serial.print("Debug is ");
      Serial.println(debug);
      printNewCmdLn();
    case (NONE):
      return false;
    case ('>'):
      controller.setThreshold(controller.getThreshold() + 0.1);
      Serial.println("PID thresh is ");
      Serial.println(controller.getThreshold());
      break;
    case ('<'):
      controller.setThreshold(controller.getThreshold() - 0.1);
      Serial.println("PID thresh is ");
      Serial.println(controller.getThreshold());
      break;
  }
  return false;
}

// Prompts User for input serial command
//    Returns serial command
byte getSerialCmd() {
  byte inByte;
  if (Serial.available()) {
    inByte = Serial.read();
    Serial.write(inByte);
    printNewLn();
    return inByte;
  } else {
    return NONE;
  }
}
