#include <SD.h>
#include <SPI.h>
#include <TMRpcm.h>

#define SD_ChipSelectPin 10   // SD Card CS Pin
#define AUDIO_INPUT A0        // Analog Pin for Audio Input
#define LED_PIN 4             // LED Indicator Pin
#define BUTTON_PIN 2          // Push Button Pin

TMRpcm audio;
TMRpcm music;
bool recmode = false;
const char* filename = "output.wav";  // Always write to this file
const char* tts = "tts.wav";
unsigned long lastButtonPress = 0;    // Timestamp for debounce delay
File audioFile;

void setup() {
    Serial.begin(115200);
    pinMode(AUDIO_INPUT, INPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    music.speakerPin = 9;
    music.setVolume(5);
    music.quality(1);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button, FALLING);

    // Initialize SD Card
    if (!SD.begin(SD_ChipSelectPin)) {
        Serial.println("SD card initialization failed!");
        while (true) {
            digitalWrite(LED_PIN, HIGH);
            delay(250);
            digitalWrite(LED_PIN, LOW);
            delay(250);
        }
    }

    Serial.println("READY");  // Signal Python that Arduino is ready
    audio.CSPin = SD_ChipSelectPin;

}

void loop() {
    if (Serial.available()) {
        String receivedText = Serial.readStringUntil('\n');
        Serial.print("Received text: ");
        Serial.println(receivedText);
        // Here you can add code to use the received text, e.g., play it using a text-to-speech module
    }
}

void button() {
    unsigned long currentTime = millis();

    // Ignore button press if within debounce delay (500ms)
    if (currentTime - lastButtonPress < 500) {
        return;
    }

    lastButtonPress = currentTime; // Update timestamp

    if (!recmode) {
        recmode = true;
        digitalWrite(LED_PIN, HIGH);

        // If file exists, delete it
        if (SD.exists(filename)) {
            SD.remove(filename);
        }

        // Start recording
        audio.startRecording(filename, 16000, AUDIO_INPUT);
        Serial.println("Recording Started...");
    }
    else {
        recmode = false;
        digitalWrite(LED_PIN, LOW);

        // Stop recording
        audio.stopRecording(filename);
        Serial.println("Recording Stopped!");

        music.play("tts.wav");
        // Automatically send the file over Serial
        sendWavFile(filename);
    }
}

void sendWavFile(const char* fileName) {
    if (!SD.exists(fileName)) {
        Serial.println("FILE NOT FOUND");
        return;
    }

    audioFile = SD.open(fileName, FILE_READ);
    if (!audioFile) {
        Serial.println("ERROR OPENING FILE");
        return;
    }

    Serial.println("SENDING");  // Indicate start of file transfer

    while (audioFile.available()) {
        byte buffer[64];  // Send in chunks of 64 bytes
        int bytesRead = audioFile.read(buffer, sizeof(buffer));
        Serial.write(buffer, bytesRead);
    }

    audioFile.close();
    Serial.println("DONE");  // Indicate end of transfer
}
