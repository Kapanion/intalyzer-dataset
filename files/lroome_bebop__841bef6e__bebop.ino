#include <BfButton.h>

#include <Arduino.h>
#include "AudioGeneratorMP3.h"
#include "AudioOutputI2S.h"
#include "AudioFileSourceSD.h"
#include "driver/i2s.h"
#include <SD.h>
#include "FastLED.h"
#include "soc/rtc_wdt.h"

#include <MacroDebugger.h>

#define DEMO1

#define I2S_DOUT 27
#define I2S_LRC 25
#define I2S_BCLK 26

#define SS 5
#define MOSI 23
#define MISO 19
#define SCK 18

#define INITIAL_GAIN 1.0
#define MAX_GAIN 4.0
#define MIN_GAIN 0.0
#define GAIN_STEP 0.2
volatile float gain = INITIAL_GAIN;

#define FIRST_TRACK 0
#define LAST_TRACK 2
volatile int song = 0;
volatile int track = FIRST_TRACK;

#define FIRST_PATTERN 1
#define LAST_PATTERN 1
volatile int pattern = FIRST_PATTERN;

#define FX_GROUP 10


//Initialize ESP8266 Audio Library classes
AudioGeneratorMP3 *mp3;
AudioFileSourceSD *file;

// sound effect 
AudioGeneratorMP3 *mp3_fx;
AudioFileSourceSD *file_fx;

AudioOutputI2S *out;

volatile bool playing = 0;
volatile uint32_t lighting = 0;
volatile byte loadTrack = 0;
volatile byte requestStop = 0;
volatile uint32_t pauseLocation = 0;
volatile int beats = 0;
volatile int lastBeat = -1;
volatile int color_loops = 0;
volatile int row_loops = 0;
volatile int col_loops = 0;
volatile int curr_dot = 0;
volatile int audio_mode = 0;
volatile uint8_t sound_effect_song_interrupted = -1;

volatile uint8_t currentSong = 0;
volatile uint8_t futureSong = 0;

hw_timer_t *My_timer = NULL;

void IRAM_ATTR onTimer() {
  beats += 1;
}

// tasks
TaskHandle_t ButtonLoopHandle;
TaskHandle_t LEDControllerHandle;

void resetBeats() {
  beats = 0;
  lastBeat = -1;
  pauseLocation = 0;
  color_loops = 0;
  row_loops = 0;
}

void buttonTask() {
  xTaskCreatePinnedToCore(
    buttonLoop,        /* Task function. */
    "ButtonLoop",      /* name of task. */
    10000,             /* Stack size of task */
    NULL,              /* parameter of the task */
    1,                 /* priority of the task */
    &ButtonLoopHandle, /* Task handle to keep track of created task */
    0);                /* pin task to core 1 */
}

void ledTask() {
  xTaskCreatePinnedToCore(
    LEDController,        /* Task function. */
    "LEDController",      /* name of task. */
    10000,                /* Stack size of task */
    NULL,                 /* parameter of the task */
    1,                    /* priority of the task */
    &LEDControllerHandle, /* Task handle to keep track of created task */
    0);                   /* pin task to core 1 */
}


void initializeTasks() {
  // tasks
  buttonTask();
  ledTask();

  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, 250000, true);
  timerSetAutoReload(My_timer, true);
  timerAlarmEnable(My_timer); //start timer

}

void sdSetup() {
  Serial.print("Initializing SD card...");
  //SPI.begin(SCK, MISO, MOSI);
  if (!SD.begin(SS)) {
    Serial.println("initialization failed!");
  }
  Serial.println("initialization done.");
}

void audioSetup() {
  out = new AudioOutputI2S();
  out->SetPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  out->SetGain(MIN_GAIN);
  
  mp3 = new AudioGeneratorMP3();
  file = new AudioFileSourceSD();

  mp3_fx = new AudioGeneratorMP3();
  file_fx = new AudioFileSourceSD();
  file->open("/group0/1.mp3");

  mp3->begin(file, out);  //Start playing the track loaded
  mp3->stop();
  out->SetGain(INITIAL_GAIN);
  file->close();
}

/**************************** Other Actions ***********************/

void audioUp() {
  Serial.println("Increasing volume");
  gain += GAIN_STEP;
  if (gain > MAX_GAIN) gain = MAX_GAIN;
  out->SetGain(gain);
}

void audioDown() {
  Serial.println("Decreasing volume");
  gain -= GAIN_STEP;
  if (gain < MIN_GAIN) gain = MIN_GAIN;
  out->SetGain(gain);
}


void nextSong() {
  Serial.println("nextSong<<");
  audio_mode++;
  if (audio_mode > LAST_TRACK) audio_mode = FIRST_TRACK;
  if (mp3->isRunning()) pauseLocation = file->getPos();
  loadTrack = audio_mode;
  lighting = audio_mode;
  playing = 1;
  Serial.println("nextSong>>");
}


void previousSong() {
  track--;
  if (track < FIRST_TRACK) track = LAST_TRACK;
  if (mp3->isRunning()) pauseLocation = file->getPos();
  loadTrack = track;
  lighting = track;
  playing = 1;
}

void nextLEDPattern() {
  pattern = FIRST_PATTERN;
}

void previousLEDPattern() {
  pattern = FIRST_PATTERN;
}

void actionsOn() {
  playing = 1;
  lighting = track;
  timerAlarmEnable(My_timer);
}

void actionsOff() {
  playing = 0;
  lighting = 0;
  curr_dot = 0;
  col_loops = 0;
  row_loops = 0;

  timerAlarmDisable(My_timer);
}

/**************************** End Other Actions ***********************/

/************************** FX Actions *************************/

void soundEffect() {
  // save current song and position
  pauseMain();
  if(mp3->isRunning()) {mp3->stop();}
  // pick and play random sound effect  
  
  char buffer[20];
  sprintf(buffer, "/group%d/%d.mp3", FX_GROUP, random(1,15));
  Serial.println(buffer);
  
  if (file_fx->isOpen()) {file_fx->close();}
  Serial.println("was closed");
  file_fx->open(buffer);
  Serial.println("was opened");
  out->begin();
  mp3_fx->begin(file_fx, out);  //Start playing the track loaded
 
}


void stop_fx() {
  mp3_fx->stop();
  out->stop();
  if (file_fx->isOpen()) {file_fx->close();}
}

/**************************  End FX Actions *************************/


/************************** Song Actions *************************/

// stop playing the track
void stop() {
  actionsOff();
  requestStop = 0;
  resetBeats();
  mp3->stop();
  futureSong = 0;
  currentSong = 0;
  Serial.println("Stopped");
  Serial.println(beats);
}



void pauseMain() {
  if (playing) {
    Serial.println("Pause");
    pauseLocation = file->getPos();
    Serial.println(pauseLocation);
    out->stop();
    Serial.println("Pause");
  }
}

void resumeMain() {
if (pauseLocation > 0) {
    Serial.println("Resume");
    out->begin();
    actionsOn();

    pauseLocation = 0;
  }  
}

void playMain() {
  Serial.println("Play");
  loadTrack = track;
  resetBeats();
  actionsOn();
}

// play or pause the track
void playPause() {
  if (playing) {
    pauseMain();
  } else if (pauseLocation > 0) {
    resumeMain();
  } else {
    playMain();
  }
}

void setTrack(int pos) {
  
  //Stop the current track if playing
  if (mp3->isRunning()) {mp3->stop();}
  
  char buffer[20];
  sprintf(buffer, "/group%d/%d.mp3", audio_mode, track);
  Serial.println(buffer);
  if (file->isOpen()) { file->close();}
  file->open(buffer);
  if (pos > 0) { Serial.println(file->seek(pos, SEEK_SET));}
  
  mp3->begin(file, out);  //Start playing the track loaded
  Serial.printf("is running: %d", mp3->isRunning());

  lighting = track;
  loadTrack = 0;
  playing = (track != 0);
}

/************************** End Song Actions *************************/

void changeSongAndLights() {
  if (futureSong != currentSong) {
    track = futureSong;
    lighting = futureSong;
    currentSong = futureSong;
    pauseLocation = file->getPos();
    loadTrack = 1;   
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  randomSeed(analogRead(0));
  rtc_wdt_protect_off();
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  buttonSetup();
  lightsSetup();
  sdSetup();
  audioSetup();

  // this has to be the last call
  initializeTasks();

  // remove these lines before shipping.
  #ifdef DEMO
  //track = 1;
 // setTrack(0);
  #endif 
}

void loop() {
  // put your main code here, to run repeatedly:
  //
  if (lastBeat != beats) {
   // Serial.print("beat ");
   // Serial.println(beats);
    lastBeat = beats;
  }
  changeSongAndLights();

  if (loadTrack) setTrack(pauseLocation);  //Load the track we want to play

  if (playing && mp3->isRunning()) {
    if (!mp3->loop() || requestStop) {
      stop();
    }
    
  } else if (mp3_fx->isRunning()) {
    if (!mp3_fx->loop()) {
      stop_fx();
      setTrack(pauseLocation);
      resumeMain();
    }
  }
}