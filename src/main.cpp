#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>       // https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>        // https://github.com/adafruit/Adafruit_Sensor
#include <WiFiManager.h>            // https://github.com/tzapu/WiFiManager
#include "AudioTools.h"
#include "AudioLibs/AudioSourceLittleFS.h"
#include "AudioCodecs/CodecMP3Helix.h"
#include "semver/semver.hpp"

using namespace semver::literals;

// Aktuelle Version Ihrer Firmware
const char* CURRENT_VERSION = "1.0.0";
//const char* REPO_URL = "https://api.github.com/repos/ploenk42/garrick/releases/latest";
const char* REPO_URL = "https://motoko.local:8443/latest";

// Your self-signed certificate in PEM format
const char* rootCACertificate = R"(
-----BEGIN CERTIFICATE-----
MIIDmTCCAoGgAwIBAgIUKeTDccZntHxV8ISSR8hbHTf86ugwDQYJKoZIhvcNAQEL
BQAwXDELMAkGA1UEBhMCQVUxEzARBgNVBAgMClNvbWUtU3RhdGUxITAfBgNVBAoM
GEludGVybmV0IFdpZGdpdHMgUHR5IEx0ZDEVMBMGA1UEAwwMbW90b2tvLmxvY2Fs
MB4XDTI0MDkxNDIxMzAxNVoXDTI1MDkxNDIxMzAxNVowXDELMAkGA1UEBhMCQVUx
EzARBgNVBAgMClNvbWUtU3RhdGUxITAfBgNVBAoMGEludGVybmV0IFdpZGdpdHMg
UHR5IEx0ZDEVMBMGA1UEAwwMbW90b2tvLmxvY2FsMIIBIjANBgkqhkiG9w0BAQEF
AAOCAQ8AMIIBCgKCAQEAkYV9UpFmz891Y+woT7+/aVmH4qoeOWX2Bkwx+OGBdrho
nNpBD6H/bgKfB+P/gSrooa5Um+A6f5yfpmVua6rG576SIlS6hNsBVwBvFaIoLyOO
r4LyGiMNqfNWY6wmBit9qLws4wg9C7b6pn8aVSXHK9BnF/sKBTDhj/Go9doRpD0x
fTOIKDyTM7LB4lJ6YlE1LmIDAMVj9+ROpW96U+d5BLlsLCUwSpFfmCLfcy4yGKJG
YEJpcsvSfxocEXl/B/r5fCdWStlzlWKNjO2BXZ5tTdwHW4lCWMb1VGlIhVgkRRhz
01F2AIQ5VeDvFKpxXngza9J3VS1cXm4Q0S/pS6meeQIDAQABo1MwUTAdBgNVHQ4E
FgQUsJa+9L2353ihbphuiM713K+mKrYwHwYDVR0jBBgwFoAUsJa+9L2353ihbphu
iM713K+mKrYwDwYDVR0TAQH/BAUwAwEB/zANBgkqhkiG9w0BAQsFAAOCAQEADFA2
MW+Mde8orNmqJ0tjz4iOaL/FP9lyNyZ6aP2QOBr8ZZwDQ9o4QIpnMTBVp67jFh8A
oE4DnnnN1V5iXdIvLn619JcKbjElMcp/aPEh6vFQjaRwXOAfsqO3mWirxOBkI6Ub
Bto96gU+Z9SsIToAA9kvLJErSdxZKgQhcUJP0AR+pmbcU+RrvsDsTbU7jZQqxU37
XOxw7K97lH/Mqrkl3ecCEocn6J13k+aVbr174upjvp7vdEqyqrwBpnLh2tLpHs3h
AQ8zAlIsBt8f5r83iJH69RNH1BFJvBXvEa1uuqY2ojdjpTKPEWR/Z8ESoRNowq7e
BxMioL9s0L4LCWo8qA==
-----END CERTIFICATE-----
)";

// Audio
const char *startFilePath="/";
const char* ext="mp3";
AudioSourceLittleFS source(startFilePath, ext);
AnalogAudioStream analog;
MP3DecoderHelix decoder;
AudioPlayer player(source, analog, decoder);

// PINS
// const int RED   = 12;
// const int GREEN = 13;
// const int BLUE  = 15;
const int RED   = 4;
const int GREEN = 4;
const int BLUE  = 4;
const int TONE  = 3;
const int MYSDA = 2; // ESP8266EX 12f
const int MYSCL = 14;// ESP8266EX 12f

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

/************************************ System ******************************/
void startDeepSleep(){
	Serial.println("Going to deep sleep...");
	ESP.deepSleep(5 * 10000);
  yield();
}

bool performUpdate(const char* url, const char* type) {
  Serial.println("performUpdate...");
  WiFiClientSecure *client = new WiFiClientSecure;

  if(!client) {
    Serial.println("ERROR: Unable to create client");
    return false;
  }

  client->setCACert(rootCACertificate);
  HTTPClient https;
  Serial.printf("begin https connection to %s\n",url);
  if (!https.begin(*client, url)) {
    Serial.println("ERROR: Unable to begin https connection");
    delete client;
    return false;
  }

  Serial.printf("get content from %s ...",url);
  int httpCode = https.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("ERROR: https unable to get content... code: %d\n", httpCode);
    https.end();
    delete client;
    return false;
  }
  Serial.println(httpCode);

  Serial.print("get content size...");
  int contentLength = https.getSize();
  Serial.println(contentLength);
  if (contentLength <= 0) {
    Serial.printf("ERROR: content size 0");
    https.end();
    delete client;
    return false;
  }

  // Initialize update depending on type "firmware" or "filesystem"
  if (type == "firmware"){
    if (!Update.begin(contentLength)){
      Serial.printf("Not enough space for %s\n",type);
      https.end();
      delete client;
      return false;
    };
  }
  if (type == "filesystem"){
    if (!Update.begin(contentLength, U_SPIFFS)){
      Serial.printf("Not enough space for %s\n",type);
      https.end();
      delete client;
      return false;
    }
  }
  
  // Flash
  Serial.printf("Downloading %s ...\n",type);
  Serial.printf("Updating %s ...\n",type);
  WiFiClient * stream = https.getStreamPtr();
  size_t written = Update.writeStream(*stream);
  if (written != contentLength) {
    Serial.printf("ERROR: couldn't write all bytes to %s \n",type);
    Serial.printf(" size %d ",contentLength);
    Serial.printf(" written %d \n",written);
    https.end();
    delete client;
    return false;
  }
  if (Update.end()) {
    Serial.printf("%s update successful\n",type);
    if (type == "filesystem") {
      Serial.println("Starting new filesystem!");
      LittleFS.begin();
    }
  }
  https.end();
  delete client;
  return true;
}

void checkForUpdates() {
  Serial.println("checkForUpdates...");
  WiFiClientSecure *client = new WiFiClientSecure;
  if(client) {
    client->setCACert(rootCACertificate);
    HTTPClient https;
    Serial.println("https begin...");
    if (https.begin(*client, REPO_URL)) {
      Serial.println("https GET...");
      int httpCode = https.GET();
      if (httpCode > 0) {
        Serial.printf("https GET... code: %d\n", httpCode);
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
          String payload = https.getString();
          JsonDocument doc;
          DeserializationError error = deserializeJson(doc, payload);
          if (error) {
            Serial.println("JSON parsing failed!");
            return;
          }
          const char* latestVersion = doc["tag_name"];
          // Extrahieren der URLs
          const char* firmwareUrl = doc["assets"][0]["browser_download_url"];
          const char* filesystemUrl = doc["assets"][1]["browser_download_url"];
          

          semver::version current = semver::version::parse(CURRENT_VERSION);
          semver::version latest = semver::version::parse(latestVersion);
          
          if (latest > current) {
            Serial.println("Neue Version verfügbar!");
            Serial.printf("Aktuelle Version: %s\n",CURRENT_VERSION);
            Serial.printf("Neueste Version: %s\n",latestVersion);
            Serial.printf("filesystemUrl: %s\n",filesystemUrl);
            Serial.printf("firmwareUrl: %s\n",firmwareUrl);
            
            // Filesystem-Update
            if (performUpdate(filesystemUrl, "filesystem")){
              // Firmware-Update
              performUpdate(firmwareUrl, "firmware");
            }


            // Neustart nach erfolgreichem Update
            ESP.restart();

          } else {
            Serial.println("Keine neue Version verfügbar.");
          }
        } else {
          Serial.println("Fehler beim Abrufen der Release-Informationen");
        }
      }
      https.end();
    } else {
      Serial.printf("https Unable to connect\n");
    }
    delete client;
  } else {
    Serial.println("Unable to create client");
  }
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

void playAlarm(const int ledPin, const int freq, const int count){
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

void playSound(const char* fileName) {
  source.setPath(fileName);
  player.begin();
}

/************************************ Gestures ******************************/ 
void up(){
  Serial.println("up");
  lighthouse(RED);
  playSound("/charge.mp3");
}

void down(){
  Serial.println("down");
  lighthouse(RED);
  playSound("/magic.mp3");
}

void rollleft(){
  Serial.println("roll left");
  lighthouse(GREEN);
  playSound("/mullet.mp3");
}

void rollright(){
  Serial.println("roll right");
  lighthouse(GREEN);
  playSound("/spell.mp3");
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
    //SIGNAL
    playAlarm(RED,2000,4);
    startDeepSleep();
  }
  if (temp.temperature > 80){
    Serial.println("Temperature exceeded 80°");
    //SIGNAL
    playAlarm(RED,1000,1);
    playAlarm(GREEN,1200,1);
    playAlarm(BLUE,1400,1);
    playAlarm(RED,1600,1);
    playAlarm(GREEN,1800,1);
    playAlarm(BLUE,2000,1);
    playAlarm(RED,2200,1);
    playAlarm(GREEN,2400,1);
    playAlarm(BLUE,2600,1);
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
      playAlarm(RED,1000,2);
      playAlarm(BLUE,1000,2);
      return;
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
  initmpu(SDA,SCL);  
}

/************************************  Mainloop ******************************/ 

void loop() {
  loopcount=loopcount+1;
  if(gestureInProgress){
    untouched=false;
  }
  if (loopcount == 1) {
    if(untouched) {
        //SIGNAL
        playAlarm(BLUE,400,3);
        WiFiManager wifiManager;
        wifiManager.autoConnect("garrick");
        checkForUpdates();
    }
  }
  if (loopcount < 3000){
    Serial.print(".");
  }
  try {
    readMpu();
  } catch(String error) {
    Serial.print("Can't read from MPU6050 error: ");
    Serial.println(error);
    //SIGNAL
    playAlarm(RED,1000,2);
    playAlarm(BLUE,1000,2);
  }
  detectGesture();
  detectCircularMotion();
  soundloop();
  delay(1);
}
