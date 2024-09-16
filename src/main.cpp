#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>       // https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>        // https://github.com/adafruit/Adafruit_Sensor
#include <WiFiManager.h>            // https://github.com/tzapu/WiFiManager
#include "AudioTools.h"
#include "AudioLibs/AudioSourceLittleFS.h"
#include "AudioCodecs/CodecMP3Helix.h"

const char *startFilePath="/";
const char* ext="mp3";
AudioSourceLittleFS source(startFilePath, ext);
AnalogAudioStream analog;
MP3DecoderHelix decoder;
AudioPlayer player(source, analog, decoder);

// PINS
const int RED   = 12;
const int GREEN = 13;
const int BLUE  = 15;
const int TONE  = 3;
const int MYSDA = 2;
const int MYSCL = 14;


void playSound(const char* fileName) {
  source.setPath(fileName);
  player.begin();
}

Adafruit_MPU6050 mpu;

// MPU data
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
// Gestures, adjust thresholds as needed
const float GESTURE_THRESHOLD = 5.0;
const float CIRCULAR_THRESHOLD = 2.0;
const int   GESTURE_DURATION = 500; // milliseconds
unsigned long gestureStartTime = 0;
bool gestureInProgress = false;
bool untouched = true;
int loopcount = 0;

void startDeepSleep(){
	Serial.println("Going to deep sleep...");
	ESP.deepSleep(5 * 1000); yield();
}

/************************************ LightFX ******************************/
void lighthouse(const int ledPin){
  // increase the LED brightness
  for(int dutyCycle = 0; dutyCycle < 255; dutyCycle++){   
    // changing the LED brightness with PWM
    analogWrite(ledPin, dutyCycle);
    delay(1);
  }

  // decrease the LED brightness
  for(int dutyCycle = 255; dutyCycle > 0; dutyCycle--){
    // changing the LED brightness with PWM
    analogWrite(ledPin, dutyCycle);
    delay(1);
  }
}

void alarm(const int ledPin, const int freq, const int count){
  Serial.println("ALARM!");
  for (int i=0; i < count; i++){
    digitalWrite(ledPin,1);
    tone(TONE, freq) ;
    delay (500); 
    noTone(TONE);
    digitalWrite(ledPin,0);
  }
}
/************************************ SoundFX ******************************/ 

void soundloop(){
  player.copy();
}

/************************************ Gestures ******************************/ 
void up(){
  Serial.println("up");
  lighthouse(RED);
  playSound("charge.mp3");
}

void down(){
  Serial.println("down");
  lighthouse(RED);
  playSound("magic.mp3");
}

void rollleft(){
  Serial.println("roll left");
  lighthouse(GREEN);
  playSound("mullet.mp3");
}

void rollright(){
  Serial.println("roll right");
  lighthouse(GREEN);
  playSound("spell.mp3");
}

void detectGesture() {
  if (!gestureInProgress) {
    if (abs(gyroX) > GESTURE_THRESHOLD || abs(gyroY) > GESTURE_THRESHOLD) {
      gestureInProgress = true;
      gestureStartTime = millis();
    }
  } else {
    if (millis() - gestureStartTime > GESTURE_DURATION) {
      if (abs(gyroX) > abs(gyroY)) {
        if (gyroX > 0) {
          down();
        } else {
          up();
        }
      } else {
        if (gyroY > 0) {
          rollleft();
        } else {
          rollright();
        }
      }
      gestureInProgress = false;
    }
  }
}

bool isCircularMotion() {
  // Implement circular motion detection logic here
  // This could involve analyzing the pattern of gyroscope data
  // over time to determine if it resembles a circular motion
  
  // For simplicity, this example just checks if the magnitude
  // of angular velocity remains above the threshold for the duration
  float magnitude = sqrt(gyroX*gyroX + gyroY*gyroY);
  Serial.println(magnitude);
  return (magnitude > CIRCULAR_THRESHOLD);
}

void detectCircularMotion() {
  float magnitude = sqrt(gyroX*gyroX + gyroY*gyroY);
  
  if (!gestureInProgress) {
    if (magnitude > CIRCULAR_THRESHOLD) {
      gestureInProgress = true;
      gestureStartTime = millis();
    }
  } else {
    if (millis() - gestureStartTime > GESTURE_DURATION) {
      if (isCircularMotion()) {
        analogWrite(BLUE,   255);
        delay(1000);
        analogWrite(BLUE,   0);
        Serial.println("Circular motion detected!");
      }
      gestureInProgress = false;
    }
  }
}

void readMpu(){
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.acceleration.x;
  gyroY = g.acceleration.y;
  gyroZ = g.acceleration.z;
  // Emergencysleep
  if (temp.temperature > 50 && temp.temperature < 80){
    Serial.println("Temperature exceeded 50°");
    alarm(RED,2000,4);
    startDeepSleep();
  }
  if (temp.temperature > 80){
    Serial.println("Temperature exceeded 80°");
    alarm(RED,1000,1);
    alarm(GREEN,1200,1);
    alarm(BLUE,1400,1);
    alarm(RED,1600,1);
    alarm(GREEN,1800,1);
    alarm(BLUE,2000,1);
    alarm(RED,2200,1);
    alarm(GREEN,2400,1);
    alarm(BLUE,2600,1);
    ESP.deepSleep(0);
  }
}

/************************************ Initialize ******************************/ 

void initlight() {
  pinMode(RED,   OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE,  OUTPUT);
  Serial.print("RED has PWM: ");
  Serial.println(digitalPinHasPWM(RED));
  Serial.print("GREEN has PWM: ");
  Serial.println(digitalPinHasPWM(GREEN));
  Serial.print("BLUE has PWM: ");
  Serial.println(digitalPinHasPWM(BLUE));
  /* DEBUG
  //RED
  digitalWrite(RED,false);
  digitalWrite(RED,true);
  delay(100);
  digitalWrite(RED,false);
  //GREEN
  digitalWrite(GREEN,false);
  digitalWrite(GREEN,true);
  delay(100);
  digitalWrite(GREEN,false);
  //BLUE
  digitalWrite(BLUE,false);
  digitalWrite(BLUE,true);
  delay(100);
  digitalWrite(BLUE,false);
  */
}
void printMetaData(MetaDataType type, const char* str, int len){
  Serial.print("==> ");
  Serial.print(toStr(type));
  Serial.print(": ");
  Serial.println(str);
}
void initsound(const int8_t rightSpeaker) {
  pinMode (rightSpeaker,OUTPUT) ;
  Serial.print("rightSpeaker has PWM: ");
  Serial.println(digitalPinHasPWM(rightSpeaker));
  //DEBUG
  // tone(rightSpeaker, 1046) ;
  // delay (100); 
  // noTone(rightSpeaker);
  Serial.printf("initsound\n");
  AudioLogger::instance().begin(Serial, AudioLogger::Info);

  // setup player
  player.setMetadataCallback(printMetaData);
  
}

void initmpu(const int8_t data, const int8_t clock) {
  Wire.begin(data,clock);
  delay(100);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(1000);
      lighthouse(RED);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause until serial console opens
  delay(100);
  // Initialize LittleFS
  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  initsound(TONE);
  initlight();
  //initmpu(SDA,SCL);
  initmpu(MYSDA,MYSCL);  
}

/************************************  Mainloop ******************************/ 

void loop() {
  loopcount=loopcount+1;
  if(gestureInProgress){
    untouched=false;
  }
  if (loopcount == 3000) {
    if(untouched) {
        alarm(BLUE,400,3);
        WiFiManager wifiManager;
        wifiManager.resetSettings();
        wifiManager.setTimeout(180); // seconds until configuration portal gets turned off
        wifiManager.autoConnect("Zauberstab");
    }
  }
  if (loopcount < 3000){
    Serial.print(".");
  }


  
  readMpu();
  detectGesture();
  detectCircularMotion();
  soundloop();
  delay(1);
}
