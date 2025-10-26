// Rotary Encoder Inputs
#define CHA         2
#define CHB         3
#define ULT_SEN     9

#define VELOCITY    ( 330 / 10000.0 )   // The ultrasonic velocity (cm/us) in air

volatile int count = 0;                 // universal count
volatile byte INTFLAG = 0;              // interrupt status flag


void setup() { 
    pinMode(CHA, INPUT);
    pinMode(CHB, INPUT);
  
    Serial.begin(9600); 
    
    /********************************* TASK 1 *********************************/
    // Finish this function call so that a rising edge on CHA pin triggers an interrupt
    attachInterrupt(CHA, encRoutine, RISING);  
}

void loop() {
    
    if (INTFLAG) {
        Serial.println(count);       
        INTFLAG = 0; 
    }

    Serial.println(ultRead(ULT_SEN));

    delay(100);
}

/********************************* TASK 2 *********************************/
// Complete the routine function, which checks the pin state of the two encoder
// pins, updates the count accordingly, and sets the INTFLAG
void encRoutine() {

}

/********************************* TASK 3 *********************************/
// Complete this function so that it returns the distance between the sensor and the obstacle
volatile byte trigState = 0;
volatile unsigned long startTime = 0;
volatile unsigned long endTime = 0;
float ultRead(int pin) {
    // Set the sensor pin high for 10us and then low

    // Set the sensor pin as an input

    // Setup an interrupt to trigger on a changing edge

    // Wait for both the rising and falling edge to occur

    // Detach the interrupt pin
     
    // Return the measured distance in cm using the VELOCITY constant and time difference
    // Make sure you handle the possibility of the micros() counter overflow between the 
    // rising and falling edge

}

/********************************* TASK 3.5 *******************************/
// Record the time when the rising and falling edge occurs
void ultRiseRoutine() {
    
}
