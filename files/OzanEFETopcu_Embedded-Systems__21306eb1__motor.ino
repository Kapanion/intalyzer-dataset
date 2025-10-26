#include <LiquidCrystal.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 37, en = 36, d4 = 35, d5 = 34, d6 = 33, d7 = 32;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
#define Motor_forward         0
#define Motor_return          1
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

// initialize x and y values
int xValue = 0;
int yValue = 0;
float voltage = 0;
float voltage1 = 0;
int btn3 =3;
int btn4 = 4;

void setup()
{
      lcd.begin(16, 2);
      Serial.begin(9600);
      attachInterrupt(digitalPinToInterrupt(btn3),left, RISING);
      attachInterrupt(digitalPinToInterrupt(btn4),right, RISING);
}

int xMap = 0;
int pwm_R=0;
int pwm_L=0; 
double motor_L, motor_R;

int left_count = 0;
int right_count = 0;
      
void loop()   
{       

 lcd.setCursor(0, 0);
 lcd.print("-100%    0      100%");
  
 lcd.setCursor(xMap, 1);
 lcd.print(" "); 
 xMap = 0;
 xValue = analogRead(A8);
 yValue = analogRead(A9);

 xMap =(xValue)*19/1023;

 lcd.setCursor(xMap, 1);
 lcd.print("|");

/*
if (xMap > 9){

  ////// Direction
       digitalWrite(Motor_R_dir_pin,Motor_return);  
       digitalWrite(Motor_L_dir_pin,Motor_return);
       float lol = xMap;
       lol = lol - 9;
       Serial.print(xMap);
       pwm_L = 40 + (lol*15);
       pwm_R = 40 + (lol*15);
  }

  if (xMap < 9){

  ////// Direction
       digitalWrite(Motor_R_dir_pin,Motor_forward);  
       digitalWrite(Motor_L_dir_pin,Motor_forward); 
       float lol = xMap;
       lol = lol - 9 ;
       lol = lol*(-1);

       Serial.print(lol);
       pwm_L = 40 + (lol*15);
       pwm_R = 40 + (lol*15);
  
  }

  
  if (xMap == 9){

  ////// Direction
      pwm_R=0;
      pwm_L=0; 
  }
      

////// Pwm
      analogWrite(Motor_L_pwm_pin,pwm_L);
      analogWrite(Motor_R_pwm_pin,pwm_R);            
    
 */

      float voltage5_1 = xValue * (255 / 1023.0);

      double ratio = (yValue/1023.0);


      if ((xValue >= 502 && xValue <= 505) && (yValue >= 494 && yValue < 500)){
              motor_L = 0;
              motor_R = 0;

      } else if (xValue < 502){
        
              digitalWrite(Motor_R_dir_pin,Motor_forward);  
              digitalWrite(Motor_L_dir_pin,Motor_forward); 

              
              motor_L = 100;
              motor_R = 100;
              
              
              
      }  else {
              motor_L = 100;
              motor_R = 100;
        
              digitalWrite(Motor_R_dir_pin,Motor_return);  
              digitalWrite(Motor_L_dir_pin,Motor_return);
              
      } 

   
     
       
        
      

      while(left_count<10){
         analogWrite(Motor_L_pwm_pin, motor_L);
         analogWrite(Motor_R_pwm_pin, motor_R);
         delay(1000);
      }
      
      

      /*Serial.print(motor_L);
      Serial.print(" ");
      Serial.print(motor_R);
      Serial.print(" ");
      Serial.print(xValue);
      Serial.print(" ");
      Serial.print(yValue);
      Serial.print(" ");
       Serial.print(" ");
      Serial.print(voltage5_1);
      Serial.print(" ");
      Serial.print(ratio);
      Serial.print(" ");
      Serial.print("\n");

      delay(1000);*/

      Serial.print(left_count);
      Serial.print(" ");
      Serial.print(right_count);
      Serial.print("\n");


      

                 
} 

void left(){
  left_count++;
  }
void right(){
  right_count++;
  }
