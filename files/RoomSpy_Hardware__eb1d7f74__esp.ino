#include <WiFi.h>
#include <HTTPClient.h>

// WiFi Configuration
const char* ssid = "[WIFI SSID]";             // Replace with your WiFi SSID
const char* password = "[WIFI PASSWORD]";  // Replace with your WiFi Password

// API Configuration
const char* serverUrl = "https://[BACKEND API URL]";  // Replace with your API URL

// Motion Sensor Configuration
volatile bool motionDetected = false;  // Flag for motion detection
const int motionPin = 34;              // Motion sensor pin
unsigned long motionTimestamps[10];    // Array to store motion timestamps (rolling window)
int motionIndex = 0;                   // Index for rolling window
const int motionWindowSize = 10;       // Size of the motion window

// LED and Error Pin Configuration
const int wifiLed = 2;    // Built-in LED
const int errorLed = 12;  // Error LED pin
bool wifiConnected = false;

// Occupancy State
bool roomOccupied = false;         // Current occupancy status
unsigned long lastMotionTime = 0;  // Last time motion was detected
// Add at the top with other globals:
unsigned long lastUpdateTime = 0;    // Tracks last status update
const long updateInterval = 1*60*1000;   // Update interval (60 seconds)


// Handle motion interrupt
void IRAM_ATTR handleMotion() {
  motionDetected = true;  // Set the flag when motion is detected
}

const char* RoomName = "Damietta";

void blinkLED(int pin, int delayMs, int count = 1) {
  for (int i = 0; i < count; i++) {
    digitalWrite(pin, HIGH);
    delay(delayMs);
    digitalWrite(pin, LOW);
    delay(delayMs);
  }
}

void connectToWiFi() {
  WiFi.begin(ssid, password);  // Start WiFi connection
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    blinkLED(wifiLed, 250);  // Blink the built-in LED while connecting
    Serial.print(".");
  }

  wifiConnected = true;
  digitalWrite(wifiLed, HIGH);  // Keep the LED solid when connected
  Serial.println("\nWiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void sendStatusData(bool occupied) {
  if (WiFi.status() == WL_CONNECTED) {  // Check WiFi connection
    HTTPClient http;
    http.begin(serverUrl);                               // Specify the URL
    http.addHeader("Content-Type", "application/json");  // Set content type

    // Prepare JSON payload
    String payload = "{\"RoomName\": \"" + String(RoomName) + "\", \"occupied\": " + String(occupied ? "true" : "false") + ", \"timestamp\": " + String(millis()) + "}";

    // Send HTTP POST request
    int httpResponseCode = http.POST(payload);

    // Debugging
    if (httpResponseCode > 0) {
      Serial.println(occupied ? "Room Occupied Status Sent" : "Room Unoccupied Status Sent");
      Serial.print("Response Code: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Error sending data. HTTP Response Code: ");
      Serial.println(httpResponseCode);
      blinkLED(errorLed, 500);  // Blink error LED on failure
    }

    http.end();  // Close connection
  } else {
    Serial.println("WiFi not connected. Unable to send data.");
    blinkLED(errorLed, 500);  // Blink error LED when WiFi is disconnected
  }
}

void updateRoomStatus() {
  unsigned long currentTime = millis();

  // Check if the room is occupied
  int motionCount = 0;
  for (int i = 0; i < motionWindowSize; i++) {
    if (currentTime - motionTimestamps[i] <=1*60*1000) {  // Within 5 minutes 5*60*1000
      motionCount++;
    }
  }

  bool occupied = motionCount >= 1;  // Room occupied if motion detected 3+ times in 5 minutes

  // If the room state changes, send data
  if (occupied != roomOccupied) {
    roomOccupied = occupied;
    sendStatusData(roomOccupied);
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Initialize LEDs
  pinMode(wifiLed, OUTPUT);
  pinMode(errorLed, OUTPUT);

  // Initialize Motion Sensor
  pinMode(motionPin, INPUT);  // Configure the motion pin as input
  attachInterrupt(digitalPinToInterrupt(motionPin), handleMotion, RISING);

  // Connect to WiFi
  connectToWiFi();

  // Initialize motion timestamps
  for (int i = 0; i < motionWindowSize; i++) {
    motionTimestamps[i] = 0;
  }
}

void loop() {
  unsigned long currentTime = millis();

  // Check for motion detection
  // Handle motion detection
  if (motionDetected) {
    Serial.println("Motion detected!");
    motionTimestamps[motionIndex] = currentTime;
    motionIndex = (motionIndex + 1) % motionWindowSize;
    lastMotionTime = currentTime;
    motionDetected = false;
  }

  // Check and update room status every second
  if (currentTime - lastMotionTime >= 100) {
    updateRoomStatus();
  }

    // Send periodic updates regardless of state change
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;
    if (wifiConnected) {
      sendStatusData(roomOccupied);
    }
  }

  // Reconnect to WiFi if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiConnected) {
      wifiConnected = false;
      digitalWrite(wifiLed, LOW);  // Turn off LED if disconnected
    }
    Serial.println("WiFi disconnected. Reconnecting...");
    connectToWiFi();
  }
}
