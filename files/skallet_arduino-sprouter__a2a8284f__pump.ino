const int buttonPin = 2;     // the number of the pushbutton pin
const int relayPin =  12;      // the number of the LED pin
const int ledPin = 13;

const int MODE_AUTO = 0;
const int MODE_MANUAL = 1;

const int RELAY_ACTIVE = LOW;
const int RELAY_INACTIVE = HIGH;

const unsigned long SECOND = 1000;
const unsigned long HOUR = 3600 * SECOND;

const unsigned long TIME_ON = 10 * SECOND;
const unsigned long TIME_OFF = 1 * HOUR - TIME_ON;

// variables will change:
int buttonState = 0;
int selectedMode = 0;
volatile int floodAvailable = 0;

void setup() {
  Serial.begin(9600);
  pinMode(relayPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  digitalWrite(relayPin, RELAY_INACTIVE);
  delay(1 * SECOND);

  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    Serial.println("Manual mode");
    selectedMode = MODE_MANUAL;
  } else {
    Serial.println("Auto mode");
    selectedMode = MODE_AUTO;
    attachInterrupt(digitalPinToInterrupt(buttonPin), floodInterrupt, RISING);
  }

  floodAvailable = 1;
  digitalWrite(ledPin, HIGH);
}

void flood() {
  if (floodAvailable > 0) {
    floodAvailable = 0;
    digitalWrite(ledPin, LOW);
    Serial.println("Flooding");
    digitalWrite(relayPin, RELAY_ACTIVE);
    delay(TIME_ON);
    digitalWrite(relayPin, RELAY_INACTIVE);
    floodAvailable = 1;
    digitalWrite(ledPin, HIGH);
  }
}

void floodRequest() {
  Serial.println("Flood request");
  flood();
}

void floodInterrupt() {
  if (digitalRead(relayPin) == RELAY_INACTIVE) {
    if (floodAvailable == 1) {
      floodAvailable = 0;
      digitalWrite(relayPin, RELAY_ACTIVE);
    }
  } else {
    floodAvailable = 1;
    digitalWrite(relayPin, RELAY_INACTIVE);
  }
}

void loop() {
  if (selectedMode == MODE_MANUAL) {
    buttonState = digitalRead(buttonPin);

    if (buttonState == HIGH) {
      digitalWrite(relayPin, LOW);
    } else {
      digitalWrite(relayPin, HIGH);
    }
  } else if (selectedMode == MODE_AUTO) {
    floodRequest();
    delay(TIME_OFF);
  }
}
