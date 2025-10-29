//WRONG CODE
#define PWMmax 255
#define M_Umax (PWmax)
#define u2pwm_(u) (u*M_Umax/U_PWR)
#define CNT_MAX 65535

 void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
  counter++;
  }else{
  counter--;
  }
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
  counter--;
  }else{
  counter++;
  }
  }


void pendulum_LQR(){

	float Kr[3] = {207.1411, 22.5873, 0.5742};
	
	float uk = Kr[0]*pendulum.position+Kr[1]*pendulum.velocity+Kr[2]*motor.velocity; // needs modification, we need to calculate motor velocity 
	
	uk = u2pwm_(uk);
	// anti-windup
	if(abs((int)uk) > M_Umax) 	uk = uk > 0 ? M_Umax : -M_Umax;
	
	// set pwm to motor
	motor_setPwm(uk); // needs modification
			
}



void setup() {
  Serial.begin (9600);

  pinMode(2, INPUT_PULLUP);  
  
  pinMode(3, INPUT_PULLUP); 

  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);
  
  }
   
  void loop() {
  if( counter != temp ){
  Serial.println (counter);
  
  temp = counter;
  }
  }


