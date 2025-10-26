#include <LiquidCrystal.h>
#include <Narcoleptic.h>

volatile int roundNum = 1;
volatile int stitchNum = 0;
volatile int lastActivityTime = 0;
volatile int lastPressStart = 0;

const int AWAKE = 0;
const int HALF_SLEEP = 1;
const int FULL_SLEEP = 2;

const int BACKLIGHT_PIN = 9;
const int PUSH_BUTTON = 2;

const int NO_PRESS = 0;
const int SHORT_PRESS = 1;
const int MEDIUM_PRESS = 2;
const int LONG_PRESS = 3;

volatile int pressState = NO_PRESS;

volatile int sleepState = AWAKE;

void updateLastActivityTime();
int timeSinceLastActivity();

void setupButton();
void setupDisplay();
void buttonStateChange();
void updateDisplay(boolean);

void checkSleep();
boolean isSleeping();

void triggerHalfSleep();
void triggerFullSleep();
void triggerWake();

void onWake();

LiquidCrystal lcd(12, 11, 6, 5, 4, 3);

void setup() {
  setupButton();
  setupDisplay();
  updateLastActivityTime();
}

void setupButton() {
  pinMode(PUSH_BUTTON, INPUT);
  attachInterrupt( 0, buttonStateChange, CHANGE );
}

void setupDisplay() {
  pinMode(BACKLIGHT_PIN, OUTPUT);   // sets the pin as output
  analogWrite( BACKLIGHT_PIN, 255 );
  lcd.begin(16,2);
  updateDisplay(true);
}

void loop() {
  int duration = 0;
  delay(200);
  if ( lastPressStart > 0 ) {
    duration = millis() - lastPressStart;
    if ( pressState == MEDIUM_PRESS && duration > 5000 ) {
      roundNum = 1;
      stitchNum = 0;
      updateDisplay(true);
      pressState = NO_PRESS;
    } else if ( pressState == SHORT_PRESS && duration > 2000 ) {
      ++roundNum;
      stitchNum = 0;
      updateDisplay(true);
      pressState = MEDIUM_PRESS;
    }
  }

  checkSleep();
}

void buttonStateChange() {
  updateLastActivityTime();

  int duration = 0;
  int state = digitalRead( PUSH_BUTTON );

  if ( state == 1 ) {
    if ( isSleeping() ) {
      triggerWake();
      lastPressStart = 0;
      return;
    }
    pressState = SHORT_PRESS;
    lastPressStart = millis();
  } else if ( state == 0 ) {
    if ( lastPressStart == 0 ) {
      return;
    }
    duration = millis() - lastPressStart; 
    lastPressStart = 0;
 
    if ( pressState == SHORT_PRESS && duration > 250 ) {
      ++stitchNum;
      updateDisplay(false);
    }
  }

}

void updateDisplay(boolean full) {
  updateLastActivityTime();
  if ( full ) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Row:");
    lcd.setCursor(0,1);
    lcd.print("Stitch:");
  }
  lcd.setCursor(5,0);
  lcd.print( roundNum );
  lcd.setCursor(8,1);
  lcd.print( stitchNum );
}

void checkSleep() {
  int idle = timeSinceLastActivity();

  if ( idle > ( 30 * 1000 ) ) {
    triggerFullSleep();
  } else if ( idle > ( 10 * 1000 ) ) {
    triggerHalfSleep();
  }

  if ( sleepState == FULL_SLEEP ) { 
    Narcoleptic.delay( 2000 );
  }
}


void updateLastActivityTime() {
  lastActivityTime = millis();
}

int timeSinceLastActivity() {
  return millis() - lastActivityTime;
}

void triggerHalfSleep() {
  sleepState = HALF_SLEEP;
  analogWrite( BACKLIGHT_PIN, 100 );
}

void triggerFullSleep() {
  sleepState = FULL_SLEEP;
  analogWrite( BACKLIGHT_PIN, 0 );
  lcd.noDisplay();
}

void triggerWake() {
  sleepState = AWAKE;
  lcd.display();
  analogWrite( BACKLIGHT_PIN, 255 );
}

boolean isSleeping() {
  return (sleepState != AWAKE);
}

void onWake() {
  analogWrite( 9, 255 );
}


