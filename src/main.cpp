#include <Arduino.h>
#include <Math.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>       // https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>        // https://github.com/adafruit/Adafruit_Sensor
#include <WiFiManager.h>            // https://github.com/tzapu/WiFiManager
#include <XT_DAC_Audio.h>
#include <Spell1.h>
#include <Spell2.h>
#include <Spell3.h>                 // Lumos
#include <Spell4.h>                 // Lumos2
#include <Spell6.h>
#include <Spell7.h>
#include <Spell8.h>
#include <Spell10.h>
#include <Spell11.h>
#include <Spell12.h>
 
XT_DAC_Audio_Class DacAudio(25,0);    // Create the main player class object. 
                                      // Use GPIO 25, one of the 2 DAC pins and timer 0
XT_Wav_Class wav(Spell6_wav);     // create an object of type XT_Wav_Class that is used by 
                                      // the dac audio class (below), passing wav data as parameter.

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
const char* ext="WAV";

// PINS
const int GREEN = 13;
const int RED   = 15;
const int BLUE  = 2;
//const int MYSDA = 2; // ESP8266EX 12f
//const int MYSCL = 14;// ESP8266EX 12f

Adafruit_MPU6050 mpu;

// MPU data
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
// Gestures, adjust thresholds as needed
const float GESTURE_THRESHOLD = 4.0;
const float CIRCULAR_THRESHOLD = 2.0;
const int   GESTURE_DURATION = 100; // milliseconds
unsigned long gestureStartTime = 0;
bool gestureInProgress = false;
bool untouched = true;
int loopcount = 0;

/************************************ System ******************************/
void startDeepSleep(){
	// DEBUG: Serial.println("Going to deep sleep...");
  mpu.enableSleep(true);
	ESP.deepSleep(5 * 1000000);
  yield();
}

bool isVersionGreater(const char* v1, const char* v2) {
  int a, b, c, d, e, f;
  sscanf(v1, "%d.%d.%d", &a, &b, &c);
  sscanf(v2, "%d.%d.%d", &d, &e, &f);
  return (a > d) || (a == d && b > e) || (a == d && b == e && c > f);
}
bool performUpdate(const char* url, const char* type) {
  // DEBUG: Serial.println("performUpdate...");
  WiFiClientSecure *client = new WiFiClientSecure;

  if(!client) {
    // DEBUG: Serial.println("ERROR: Unable to create client");
    return false;
  }

  client->setCACert(rootCACertificate);
  HTTPClient https;
  // DEBUG: Serial.printf("begin https connection to %s\n",url);
  if (!https.begin(*client, url)) {
    // DEBUG: Serial.println("ERROR: Unable to begin https connection");
    delete client;
    return false;
  }

  // DEBUG: Serial.printf("get content from %s ...",url);
  int httpCode = https.GET();
  if (httpCode != HTTP_CODE_OK) {
    // DEBUG: Serial.printf("ERROR: https unable to get content... code: %d\n", httpCode);
    https.end();
    delete client;
    return false;
  }
  // DEBUG: Serial.println(httpCode);

  // DEBUG: Serial.print("get content size...");
  int contentLength = https.getSize();
  // DEBUG: Serial.println(contentLength);
  if (contentLength <= 0) {
    // DEBUG: Serial.printf("ERROR: content size 0");
    https.end();
    delete client;
    return false;
  }

  // Initialize update depending on type "firmware" or "filesystem"
  if (type == "firmware"){
    if (!Update.begin(contentLength)){
      // DEBUG: Serial.printf("Not enough space for %s\n",type);
      https.end();
      delete client;
      return false;
    };
  }
  if (type == "filesystem"){
    if (!Update.begin(contentLength, U_SPIFFS)){
      // DEBUG: Serial.printf("Not enough space for %s\n",type);
      https.end();
      delete client;
      return false;
    }
  }
  
  // Flash
  // DEBUG: Serial.printf("Downloading %s ...\n",type);
  // DEBUG: Serial.printf("Updating %s ...\n",type);
  WiFiClient * stream = https.getStreamPtr();
  size_t written = Update.writeStream(*stream);
  if (written != contentLength) {
    // DEBUG: Serial.printf("ERROR: couldn't write all bytes to %s \n",type);
    // DEBUG: Serial.printf(" size %d ",contentLength);
    // DEBUG: Serial.printf(" written %d \n",written);
    https.end();
    delete client;
    return false;
  }
  if (Update.end()) {
    // DEBUG: Serial.printf("%s update successful\n",type);
    if (type == "filesystem") {
      // DEBUG: Serial.println("Starting new filesystem!");
      LittleFS.begin();
    }
  }
  https.end();
  delete client;
  return true;
}

void checkForUpdates() {
  // DEBUG: Serial.println("checkForUpdates...");
  WiFiClientSecure *client = new WiFiClientSecure;
  if(client) {
    client->setCACert(rootCACertificate);
    HTTPClient https;
    // DEBUG: Serial.println("https begin...");
    if (https.begin(*client, REPO_URL)) {
      // DEBUG: Serial.println("https GET...");
      int httpCode = https.GET();
      if (httpCode > 0) {
        // DEBUG: Serial.printf("https GET... code: %d\n", httpCode);
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
          String payload = https.getString();
          JsonDocument doc;
          DeserializationError error = deserializeJson(doc, payload);
          if (error) {
            // DEBUG: Serial.println("JSON parsing failed!");
            return;
          }
          const char* latestVersion = doc["tag_name"];
          // Extrahieren der URLs
          const char* firmwareUrl = doc["assets"][0]["browser_download_url"];
          const char* filesystemUrl = doc["assets"][1]["browser_download_url"];
          
          if (isVersionGreater(latestVersion, CURRENT_VERSION)) {
            // DEBUG: Serial.println("Neue Version verfügbar!");
            // DEBUG: Serial.printf("Aktuelle Version: %s\n",CURRENT_VERSION);
            // DEBUG: Serial.printf("Neueste Version: %s\n",latestVersion);
            // DEBUG: Serial.printf("filesystemUrl: %s\n",filesystemUrl);
            // DEBUG: Serial.printf("firmwareUrl: %s\n",firmwareUrl);
            
            // Filesystem-Update
            if (performUpdate(filesystemUrl, "filesystem")){
              // Firmware-Update
              performUpdate(firmwareUrl, "firmware");
            }

            // Neustart nach erfolgreichem Update
            ESP.restart();

          } else {
            // DEBUG: Serial.println("Keine neue Version verfügbar.");
          }
        } else {
          // DEBUG: Serial.println("Fehler beim Abrufen der Release-Informationen");
        }
      }
      https.end();
    } else {
      // DEBUG: Serial.printf("https Unable to connect\n");
    }
    delete client;
  } else {
    // DEBUG: Serial.println("Unable to create client");
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
  // DEBUG: Serial.println("ALARM!");
  for (int i=0; i < count; i++){
    digitalWrite(ledPin,1);
    //tone(TONE, freq) ;
    delay (500); 
    //noTone(TONE);
    digitalWrite(ledPin,0);
  }
}
/************************************ SoundFX ******************************/ 


void playSound(const char* filename) {
  if(!wav.Playing)          // if completed playing, play again
    DacAudio.Play(&wav);  
  /*
  File file = LittleFS.open(filename, "r");
  if (!file) {
    // DEBUG: Serial.println("Failed to open file for reading");
    return;
  }

  // Skip WAV header
  file.seek(44);
  
  while (file.available()) {
    uint8_t sample = file.read();
    dac_output_voltage(DAC_CHANNEL_1, sample);
    delayMicroseconds(125);  // Adjust this for different sample rates
  }

  file.close();
  */
}

/************************************ Gestures ******************************/ 
void up(){
  // DEBUG: Serial.println("up");
  lighthouse(RED);
  playSound("/charge.wav");
}

void down(){
  // DEBUG: Serial.println("down");
  lighthouse(RED);
  playSound("/magic.wav");
}

void rollleft(){
  // DEBUG: Serial.println("roll left");
  lighthouse(GREEN);
  playSound("/mullet.wav");
}

void rollright(){
  // DEBUG: Serial.println("roll right");
  lighthouse(GREEN);
  playSound("/spell.wav");
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
  // DEBUG: Serial.println(magnitude);
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
        // DEBUG: Serial.println("Circular motion detected!");
      }
      gestureInProgress = false;
    }
  }
}

int leftFiveDigits(double num) {
  return (int)num*100;
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
    // DEBUG: Serial.println("Temperature exceeded 50°");
    //SIGNAL
    playAlarm(RED,2000,4);
    startDeepSleep();
  }
  if (temp.temperature > 80){
    // DEBUG: Serial.println("Temperature exceeded 80°");
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
  // DEBUG: Serial.print("RED has PWM: ");
  // DEBUG: Serial.println(digitalPinHasPWM(RED));
  // DEBUG: Serial.print("GREEN has PWM: ");
  // DEBUG: Serial.println(digitalPinHasPWM(GREEN));
  // DEBUG: Serial.print("BLUE has PWM: ");
  // DEBUG: Serial.println(digitalPinHasPWM(BLUE));
  //RED
  digitalWrite(RED,false);
  digitalWrite(RED,true);
  delay(1000);
  digitalWrite(RED,false);
  //GREEN
  digitalWrite(GREEN,false);
  digitalWrite(GREEN,true);
  delay(1000);
  digitalWrite(GREEN,false);
  //BLUE
  digitalWrite(BLUE,false);
  digitalWrite(BLUE,true);
  delay(1000);
  digitalWrite(BLUE,false);
}

void initmpu(const int8_t data, const int8_t clock) {
  Wire.begin(data,clock);
  delay(100);
  if (!mpu.begin()) {
    // DEBUG: Serial.println("Failed to find MPU6050 chip");
      playAlarm(RED,1000,2);
      playAlarm(BLUE,1000,2);
      return;
  }
  // DEBUG: Serial.println("MPU6050 Found!");

  // Set the accelerometer range (2, 4, 8, or 16 G)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // Set the bandwidth to the lowest setting
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  // Set the wake frequency (options are 1.25 Hz, 5 Hz, 20 Hz, 40 Hz)
  mpu.setCycleRate(MPU6050_CYCLE_5_HZ);

  // DEBUG: Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    // DEBUG: Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    // DEBUG: Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    // DEBUG: Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    // DEBUG: Serial.println("+-16G");
    break;
  }
  // DEBUG: Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    // DEBUG: Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    // DEBUG: Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    // DEBUG: Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    // DEBUG: Serial.println("+- 2000 deg/s");
    break;
  }

  // DEBUG: Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    // DEBUG: Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    // DEBUG: Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    // DEBUG: Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    // DEBUG: Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    // DEBUG: Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    // DEBUG: Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    // DEBUG: Serial.println("5 Hz");
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
    // DEBUG: Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  //dac_output_enable(DAC_CHANNEL_1);
  initlight();
  initmpu(SDA,SCL);  
}

/************************************  Mainloop ******************************/ 

void loop() {
  DacAudio.FillBuffer();              //Fill the sound buffer with data
  loopcount=loopcount+1;
  if(gestureInProgress){
    untouched=false;
  }
  if (loopcount == 3000) {
    if(untouched) {
        //SIGNAL
        playAlarm(BLUE,400,3);
        WiFiManager wifiManager;
        wifiManager.autoConnect("garrick");
        checkForUpdates();
        wifiManager.disconnect();
        WiFi.mode(WIFI_OFF);    // Switch WiFi off to save power
    }
  }
  try {
    mpu.enableSleep(false);
    readMpu();
  } catch(String error) {
    // DEBUG: Serial.printf("Can't read from MPU6050 error: %s\n",error);
    //SIGNAL
    playAlarm(RED,1000,2);
    playAlarm(BLUE,1000,2);
  }
  detectGesture();
  //detectCircularMotion();
  
  // DEBUG: Serial.print(round(accX));
  // DEBUG: Serial.print("\t");
  // DEBUG: Serial.print(round(accY));
  // DEBUG: Serial.print("\t");
  // DEBUG: Serial.print(round(accZ));
  // DEBUG: Serial.print("\t");
  // DEBUG: Serial.print(round(gyroX));
  // DEBUG: Serial.print("\t");
  // DEBUG: Serial.print(round(gyroY));
  // DEBUG: Serial.print("\t");
  // DEBUG: Serial.print(round(gyroZ));
  // DEBUG: Serial.print("\n");
}
