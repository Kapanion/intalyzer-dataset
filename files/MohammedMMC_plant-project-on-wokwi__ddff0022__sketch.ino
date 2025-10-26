#include "DHT.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define DHT22_PIN 28

#define RGB_R_PIN 6
#define RGB_G_PIN 7
#define RGB_B_PIN 8

#define ENCODER_CLK 2
#define ENCODER_DT  3
#define ENCODER_BTN 9

#define SOIL_SENSOR 20
#define MOTION_SENSOR 22
#define BUZZER_PIN 13

int minimum_water = 400;

DHT dht(DHT22_PIN, DHT22);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

int led_r = 0, led_g = 0, led_b = 0;

char selectedColor = 'G'; // R, G and B
bool isColorSelected = false;

void setup() {
  // put your setup code here, to run once:
  dht.begin();
  pinMode(DHT22_PIN, INPUT);
  Serial1.begin(115200);
  Serial1.println("Hello, Raspberry Pi Pico W!");

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial1.println("SSD1306 allocation failed");
    for(;;);
  }

  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);

  pinMode(SOIL_SENSOR, INPUT);
  pinMode(MOTION_SENSOR, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(ENCODER_CLK, INPUT);
  pinMode(ENCODER_DT, INPUT);
  pinMode(ENCODER_BTN, INPUT_PULLUP);

  digitalWrite(BUZZER_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), readEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_BTN), buttonEncoder, FALLING);
}


void loop() {
  // put your main code here, to run repeatedly:
  displayScreen();
  rgblight();
  if (minimum_water >= analogRead(SOIL_SENSOR) && digitalRead(MOTION_SENSOR)) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  delay(10); // this speeds up the simulation
}

void buttonEncoder() {
  if (digitalRead(ENCODER_BTN) == LOW) {
    isColorSelected = !isColorSelected;
  }
}

void readEncoder() {
  int dtValue = digitalRead(ENCODER_DT);
  if (dtValue == HIGH) {
    if(selectedColor == 'R') {
      if (isColorSelected) {
        if (led_r < 255) led_r += 5;
      } else {selectedColor = 'G';}
    } else if (selectedColor == 'G') {
      if (isColorSelected) {
        if (led_g < 255) led_g += 5;
      } else {selectedColor = 'B';}
    } else if (selectedColor == 'B') {
      if (isColorSelected) {
        if (led_b < 255) led_b += 5;
      } else {selectedColor = 'R';}
    }
  }
  if (dtValue == LOW) {
    if(selectedColor == 'R') {
      if (isColorSelected) {
        if (led_r > 0) led_r -= 5;
      } else {selectedColor = 'B';}
    } else if (selectedColor == 'G') {
      if (isColorSelected) {
        if (led_g > 0) led_g -= 5;
      } else {selectedColor = 'R';}
    } else if (selectedColor == 'B') {
      if (isColorSelected) {
        if (led_b > 0) led_b -= 5;
      } else {selectedColor = 'G';}
    }
  }
}


void rgblight() {
  analogWrite(RGB_R_PIN, led_r);
  analogWrite(RGB_G_PIN, led_g);
  analogWrite(RGB_B_PIN, led_b);
}

void displayScreen() {
  // Get Humidity and Temperature
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Clear the screen
  display.clearDisplay();
  display.setTextColor(WHITE);

  // Display (Humidity and Temperature) 
  display.setTextSize(1);
  display.setCursor(5,0);         
  display.println("Humidity:");
  display.setCursor(SCREEN_WIDTH / 2 + 20,0);
  display.println("Temp:");
  display.setCursor(10,10);         
  display.println(String(h, 1) + " %");   
  display.setCursor(SCREEN_WIDTH / 2 + 15,10);
  display.println(String(t, 1) + " C");

  display.drawLine(SCREEN_WIDTH / 2, 0, 
    SCREEN_WIDTH / 2, 25, WHITE);
  display.drawLine(0, 25, 
    SCREEN_WIDTH, 25, WHITE);

  for (int i = 1; i < 3; i++) {
    display.drawLine(SCREEN_WIDTH / 3 * i, 25, 
      SCREEN_WIDTH / 3 * i, SCREEN_HEIGHT, WHITE);
  }
  if (isColorSelected) {
    display.fillRoundRect(
    SCREEN_WIDTH / 3 * (selectedColor == 'G' ? 1 : selectedColor == 'B' ? 2 : 0) + 3,
    SCREEN_HEIGHT / 2 - 3,
    SCREEN_WIDTH / 3 - 5,
    SCREEN_WIDTH / 3 - 8,
    3, WHITE);
  } else {
    display.drawRoundRect(
    SCREEN_WIDTH / 3 * (selectedColor == 'G' ? 1 : selectedColor == 'B' ? 2 : 0) + 3,
    SCREEN_HEIGHT / 2 - 3,
    SCREEN_WIDTH / 3 - 5,
    SCREEN_WIDTH / 3 - 8,
    3, WHITE);
  }

  display.setTextSize(2);
  display.setCursor(0 + 16, (SCREEN_HEIGHT + 10) / 2);
  display.setTextColor(selectedColor == 'R' && isColorSelected ? BLACK : WHITE);
  display.println("R");
  display.setCursor(SCREEN_WIDTH / 3 + 16, (SCREEN_HEIGHT + 10) / 2);
  display.setTextColor(selectedColor == 'G' && isColorSelected ? BLACK : WHITE);
  display.println("G");
  display.setCursor(SCREEN_WIDTH / 3 * 2 + 16, (SCREEN_HEIGHT + 10) / 2);
  display.setTextColor(selectedColor == 'B' && isColorSelected ? BLACK : WHITE);
  display.println("B");

  display.setTextSize(1);
  display.setCursor(0 + 18, (SCREEN_HEIGHT + 45) / 2);
  display.setTextColor(selectedColor == 'R' && isColorSelected ? BLACK : WHITE);
  display.println(String(led_r));
  display.setCursor(SCREEN_WIDTH / 3 + 18, (SCREEN_HEIGHT + 45) / 2);
  display.setTextColor(selectedColor == 'G' && isColorSelected ? BLACK : WHITE);
  display.println(String(led_g));
  display.setCursor(SCREEN_WIDTH / 3 * 2 + 18, (SCREEN_HEIGHT + 45) / 2);
  display.setTextColor(selectedColor == 'B' && isColorSelected ? BLACK : WHITE);
  display.println(String(led_b));

  display.display();
}