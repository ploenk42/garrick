#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <FS.h>
#include <LittleFS.h>
#include <Adafruit_MPU6050.h>       // https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>        // https://github.com/adafruit/Adafruit_Sensor
#include <WiFiManager.h>            // https://github.com/tzapu/WiFiManager
#include "AudioFileSourceLittleFS.h" // https://github.com/earlephilhower/ESP8266Audio
#include "AudioGeneratorWAV.h"
#include "AudioOutputI2SNoDAC.h"
#include "spellimpact.h"

// A single, global CertStore which can be used by all
// connections.  Needs to stay live the entire time any of
// the WiFiClientBearSSLs are present.
#include <CertStoreBearSSL.h>
BearSSL::CertStore certStore;

const char* version = "0.1.0";

// PINS
const int RED   = 12;
const int GREEN = 13;
const int BLUE  = 15;
const int TONE  = 3;
const int MYSDA = 2;
const int MYSCL = 14;


AudioGeneratorWAV *wav;
AudioFileSourceLittleFS *file;
AudioOutputI2SNoDAC *out;
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
bool mchammer = true;

// Autoupdate URIs
// const char* firmwareInfoURI = "https://api.github.com/repos/ploenk42/espwand/releases/latest";
// #define firmwareBinURI "https://raw.githubusercontent.com/ploenk42/espwand/firmware.bin"
const char* firmwareInfoURI = "https://motoko.local:8443/latest";
#define firmwareBinURI "https://motoko.local:8443/firmware.bin"

int loopcount = 0;

void startDeepSleep(){
	Serial.println("Going to deep sleep...");
	ESP.deepSleep(5 * 1000); yield();
}

/************************************ LightFX ******************************/ 
void bluetick()
{
  int state = digitalRead(BLUE);  // get the current state
  digitalWrite(BLUE, !state);     // set pin to the opposite state
}
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

void doremi(const int tonePin,const int speed){
    tone(tonePin, 523) ; //DO note 523 Hz
    delay (speed); 
    tone(tonePin, 587) ; //RE note ...
    delay (speed); 
    tone(tonePin, 659) ; //MI note ...
    delay (speed); 
    tone(tonePin, 783) ; //FA note ...
    delay (speed); 
    tone(tonePin, 880) ; //SOL note ...
    delay (speed); 
    tone(tonePin, 987) ; //LA note ...
    delay (speed); 
    tone(tonePin, 1046) ; // SI note ...
    delay (speed); 
    noTone(tonePin);
}

void playsound(const int sound){
  if (wav->isRunning()) {
    wav->stop();
  }
  file->close();
  delete file;
  delete wav;
  Serial.print("play sound nr. ");
  Serial.println(sound);
  switch (sound) {
  case 1:
    file = new AudioFileSourceLittleFS("/spell.wav");
    wav = new AudioGeneratorWAV();
    wav->begin(file, out);
    break;
  case 2:
    file = new AudioFileSourceLittleFS("/mallett.wav");
    wav = new AudioGeneratorWAV();
    wav->begin(file, out);
    break;
  case 3:
    file = new AudioFileSourceLittleFS("/magic.wav");
    wav = new AudioGeneratorWAV();
    wav->begin(file, out);
    break;
  case 4:
    file = new AudioFileSourceLittleFS("/swing.wav");
    wav = new AudioGeneratorWAV();
    wav->begin(file, out);
    break;
  default:
    file = new AudioFileSourceLittleFS("/magic.wav");
    wav = new AudioGeneratorWAV();
    wav->begin(file, out);
    break; // Wird nicht benötigt, wenn Statement(s) vorhanden sind
  }
}

void soundloop(){
  if (wav->isRunning()) {
    if (!wav->loop()) wav->stop();
  }
}

/************************************ Gestures ******************************/ 
void up(){
  Serial.println("up");
  lighthouse(RED);
  playsound(1);
}

void down(){
  Serial.println("down");
  lighthouse(RED);
  playsound(2);
}

void rollleft(){
  Serial.println("roll left");
  lighthouse(GREEN);
  playsound(3);
}

void rollright(){
  Serial.println("roll right");
  lighthouse(GREEN);
  playsound(4);
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


/************************************ Firmware ******************************/ 
void update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
}

void update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}

void update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}

void updateFirmware() {
  Serial.println("updating firmware");

  ESPhttpUpdate.setLedPin(BLUE, HIGH);
  // Add optional callback notifiers
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);

  BearSSL::WiFiClientSecure client;
  client.setInsecure();
  client.setBufferSizes(1024, 1024);
  client.setCertStore(&certStore);

  t_httpUpdate_return ret = ESPhttpUpdate.update(client, firmwareBinURI);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }
}

void getNewFirmware() {
  WiFiClientSecure *client = new WiFiClientSecure;  
  HTTPClient http;
  client->setInsecure();
  http.setUserAgent("ESP8266");
  http.addHeader("Host", "motoko.local");

  Serial.print("[HTTP] begin...\n");    
  http.begin(*client, firmwareInfoURI); //HTTP
  
  Serial.print("[HTTP] GET...\n");
  // start connection and send HTTP header
  int httpCode = http.GET();
    delay(500);
  // httpCode will be negative on error
  if (httpCode > 0) {
    // HTTP header has been send and Server response header has been handled
    Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    if (httpCode == HTTP_CODE_OK) {
      String firmwareInfoPayload = http.getString();
      JsonDocument firmwareInfoJson; 
      DeserializationError err = deserializeJson(firmwareInfoJson, firmwareInfoPayload);
      if (err) {
        Serial.print(F("deserializeJson() failed with code "));
        Serial.println(err.f_str());
      }
      
      String newVersion = firmwareInfoJson["tag_name"].as<String>();
      Serial.println(newVersion);
      if(!newVersion.equals(version)) {
        updateFirmware();
        Serial.println("New Version Updated!");
      }
      else{
        Serial.print("Device already on latest firmware version");
      }
    }
  } else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
}

// Set time via NTP, as required for x.509 validation
void setClock() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");  // UTC

  Serial.print(F("Waiting for NTP time sync: "));
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    yield();
    delay(500);
    Serial.print(F("."));
    now = time(nullptr);
  }

  Serial.println(F(""));
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.print(F("Current time: "));
  Serial.print(asctime(&timeinfo));
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

void initsound(const int8_t rightSpeaker) {
  pinMode (rightSpeaker,OUTPUT) ;
  Serial.print("rightSpeaker has PWM: ");
  Serial.println(digitalPinHasPWM(rightSpeaker));
  //DEBUG
  // tone(rightSpeaker, 1046) ;
  // delay (100); 
  // noTone(rightSpeaker);
  Serial.printf("initsound\n");
  out = new AudioOutputI2SNoDAC();
  file = new AudioFileSourceLittleFS("/magic-charge-mana-2-186628.mp3.wav");
  wav = new AudioGeneratorWAV();
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
  WiFiManager wifiManager;
  wifiManager.setTimeout(180); // seconds until configuration portal gets turned off
  wifiManager.autoConnect("Zauberstab");
  initsound(TONE);
  initlight();
  //initmpu(SDA,SCL);
  initmpu(MYSDA,MYSCL);  
  LittleFS.begin();
  int numCerts = certStore.initCertStore(LittleFS, PSTR("/certs.idx"), PSTR("/certs.ar"));
  Serial.print(F("Number of CA certs read: "));
  Serial.println(numCerts);
  if (numCerts == 0) {
    Serial.println(F("No certs found. Did you run certs-from-mozilla.py and upload the LittleFS directory before running?"));
    return;  // Can't connect to anything w/o certs!
  }

}

/************************************  Mainloop ******************************/ 

void loop() {
  loopcount=loopcount+1;
  if(gestureInProgress){
    mchammer=false;
  }
  if (loopcount == 3000) {
    if(mchammer) {
      getNewFirmware();
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
