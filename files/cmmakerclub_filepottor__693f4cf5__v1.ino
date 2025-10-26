int x = 0;
void setup() {
  pinMode(A3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(0, y, RISING);
  pinMode(1, OUTPUT);
   pinMode(A2, OUTPUT);
}

void loop() {
  //t();
   digitalWrite(A2, LOW);
}
void t(){
    if(x>0){
    digitalWrite(1, LOW);
  }
  
  if(x<0){
    digitalWrite(1, HIGH);
  }
    if(x==0){
    digitalWrite(1, LOW);
    
  }
}
void y(){
   digitalWrite(A2, HIGH);
  if(digitalRead(A3)==1){
    x = x - 1;
  }
  if(digitalRead(A3)==0){
    x = x + 1;
  }
}


