volatile bool pedestrianRequest = false;

void setup() {
  //Traffic light 1
  pinMode(8, OUTPUT);  // Red
  pinMode(12, OUTPUT);  // Amber - I changed it from pin 3 to 12 and it fixed the problem of it activating the interrupt signal ¯\_(ツ)_/¯
  pinMode(4, OUTPUT);  // Green

  //Traffic light 2
  pinMode(5, OUTPUT);  // Red
  pinMode(6,OUTPUT);   // Amber
  pinMode(7, OUTPUT);  // Green
  
  //Pedestrian lights
  pinMode(2, INPUT);  // crossing button
  pinMode(9, OUTPUT);        // Red 
  pinMode(10, OUTPUT);       // Green

  attachInterrupt(digitalPinToInterrupt(2), changepedestrianstate, CHANGE);


  //Start all red lights
  digitalWrite(8, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(9, HIGH);
}

void loop() {
  lightset1();
  pedestrianlightcontrol();
  lightset2();
  pedestrianlightcontrol();
}

void lightset1() {
  //Prepare to drive
  digitalWrite(12, HIGH);
  delay(4000);
  // Set green light
  digitalWrite(4, HIGH);
  digitalWrite(12, LOW);
  digitalWrite(8, LOW);
  delay(20000);
  //Prepare to stop
  digitalWrite(4, LOW);
  digitalWrite(12, HIGH);
  delay(5000);
  //red light
  digitalWrite(12, LOW);
  digitalWrite(8, HIGH);
  delay(5000);
}

void lightset2() {
  //Prepare to drive
  digitalWrite(5, LOW);
  for (int i = 0; i <= 5; i++) {
    digitalWrite(6, HIGH);
    delay(600);
    digitalWrite(6, LOW);
    delay(600);
  };
  // Set green light
  digitalWrite(7, HIGH);
  delay(20000);
  //Prepare to stop
  digitalWrite(7, LOW);
  digitalWrite(6, HIGH);
  delay(5000);
  //red light
  digitalWrite(6, LOW);
  digitalWrite(5, HIGH);
  delay(5000);
}


void changepedestrianstate() {
  pedestrianRequest = true;
  }

void pedestrianlightcontrol() {
  if (pedestrianRequest == true) {
    
    pedestrianRequest = false;
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);   
    delay(15000);
    digitalWrite(9, HIGH);  
    digitalWrite(10, LOW);  
    delay(5000);
  }
}


