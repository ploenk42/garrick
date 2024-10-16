#include <Arduino.h>
#include <Math.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>       // https://github.com/adafruit/Adafruit_MPU6050
#include <Adafruit_Sensor.h>        // https://github.com/adafruit/Adafruit_Sensor
#include <WiFiManager.h>            // https://github.com/tzapu/WiFiManager
#include <XT_DAC_Audio.h>
//SPELLS
#include <spell1.wav.h>
#include <spell2.wav.h>

/* PINS
32,33,23,
input only 34,35,36,39
wifi 0,2,4*,12,13,14,15,25,26,27
touch 0,2,4*,12,13,14,15,27 32,33
DAC (25),26
SDA 21,22
16,17,18
nonos 0,2,5,12,15
*/

const int DAC = 25;

XT_DAC_Audio_Class DacAudio(DAC,0);    // Use GPIO 25, one of the 2 DAC pins and timer 0
XT_Wav_Class spell1(spell1_wav);
XT_Wav_Class spell2(spell2_wav);

Adafruit_MPU6050 mpu;

unsigned long startTime;

// --- Audio
const char *startFilePath="/";
const char* ext="WAV";

// --- PINS
const int RED = 16;
const int GREEN   = 17;
const int BLUE  = 18;
//const int MYSDA = 2; // ESP8266EX 12f
//const int MYSCL = 14;// ESP8266EX 12f

// --- MPU data
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
// --- Gestures, adjust thresholds as needed
const float GESTURE_THRESHOLD = 4.0;
const float CIRCULAR_THRESHOLD = 2.0;
const int   GESTURE_DURATION = 100; // milliseconds
unsigned long gestureStartTime = 0;
bool gestureInProgress = false;
bool untouched = true;
bool mpu_fail = false;
bool sequenceActive = false;
const int MAX_SIZE = 3;
int stack[MAX_SIZE];
int top = -1;



// OTA update
const unsigned long triggerDelay = 10000; // 10 seconds in milliseconds
bool triggered = false;
const char* CURRENT_VERSION = "1.0.2";
const char* REPO_URL = "https://api.github.com/repos/ploenk42/garrick/releases/latest";
//const char* REPO_URL = "https://motoko.local:8443/latest";

// (self-signed) root certificate in PEM format
const char* rootCACertificate = R"(
-----BEGIN CERTIFICATE-----
MIICjzCCAhWgAwIBAgIQXIuZxVqUxdJxVt7NiYDMJjAKBggqhkjOPQQDAzCBiDEL
MAkGA1UEBhMCVVMxEzARBgNVBAgTCk5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNl
eSBDaXR5MR4wHAYDVQQKExVUaGUgVVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMT
JVVTRVJUcnVzdCBFQ0MgQ2VydGlmaWNhdGlvbiBBdXRob3JpdHkwHhcNMTAwMjAx
MDAwMDAwWhcNMzgwMTE4MjM1OTU5WjCBiDELMAkGA1UEBhMCVVMxEzARBgNVBAgT
Ck5ldyBKZXJzZXkxFDASBgNVBAcTC0plcnNleSBDaXR5MR4wHAYDVQQKExVUaGUg
VVNFUlRSVVNUIE5ldHdvcmsxLjAsBgNVBAMTJVVTRVJUcnVzdCBFQ0MgQ2VydGlm
aWNhdGlvbiBBdXRob3JpdHkwdjAQBgcqhkjOPQIBBgUrgQQAIgNiAAQarFRaqflo
I+d61SRvU8Za2EurxtW20eZzca7dnNYMYf3boIkDuAUU7FfO7l0/4iGzzvfUinng
o4N+LZfQYcTxmdwlkWOrfzCjtHDix6EznPO/LlxTsV+zfTJ/ijTjeXmjQjBAMB0G
A1UdDgQWBBQ64QmG1M8ZwpZ2dEl23OA1xmNjmjAOBgNVHQ8BAf8EBAMCAQYwDwYD
VR0TAQH/BAUwAwEB/zAKBggqhkjOPQQDAwNoADBlAjA2Z6EWCNzklwBBHU6+4WMB
zzuqQhFkoJ2UOQIReVx7Hfpkue4WQrO/isIJxOzksU0CMQDpKmFHjFJKS04YcPbW
RNZu9YO6bVi9JNlWSOrvxKJGgYhqOkbRqZtNyWHa0V1Xahg=
-----END CERTIFICATE-----
)";

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
  digitalWrite(ledPin,0);
}

void lumus(){
  digitalWrite(RED,true);
  digitalWrite(GREEN,true);
  digitalWrite(BLUE,true);
}

void playAlarm(const int ledPin, const int freq, const int count){
  Serial.println("ALARM!");
  for (int i=0; i < count; i++){
    digitalWrite(ledPin,1);
    //tone(TONE, freq) ;
    delay (500); 
    //noTone(TONE);
    digitalWrite(ledPin,0);
  }
}

/************************************ System ******************************/
void startDeepSleep(){
	Serial.println("Going to deep sleep...");
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

  int maxRedirects = 5;
  int redirectCount = 0;

  Serial.printf("get content from %s ...\n",url);
  int httpCode = https.GET();
  while (httpCode == HTTP_CODE_FOUND && redirectCount < maxRedirects) {
    String newUrl = https.getLocation();
    Serial.printf("INFO: redirecting: %s\n", newUrl);
    https.end();
    https.begin(newUrl);
    httpCode = https.GET();
    redirectCount++;
  }
  if (httpCode != HTTP_CODE_OK){
    if (redirectCount >= maxRedirects){
      Serial.printf("ERROR: https unable to get content... too many redirects");
    }else{
      Serial.printf("ERROR: https unable to get content... code: %d\n", httpCode);
    }
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
    lighthouse(GREEN);
    lighthouse(GREEN);
    lighthouse(GREEN);
    if (type == "filesystem") {
      Serial.println("Starting new filesystem!");
      //LittleFS.begin();
      Serial.println("No filesystem!");
      return false;

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
          lighthouse(BLUE);
          lighthouse(BLUE);
          lighthouse(BLUE);
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
          //const char* filesystemUrl = doc["assets"][1]["browser_download_url"];
          
          if (isVersionGreater(latestVersion, CURRENT_VERSION)) {
            Serial.println("Neue Version verfügbar!");
            Serial.printf("Aktuelle Version: %s\n",CURRENT_VERSION);
            Serial.printf("Neueste Version: %s\n",latestVersion);
            //Serial.printf("filesystemUrl: %s\n",filesystemUrl);
            Serial.printf("firmwareUrl: %s\n",firmwareUrl);
            
            // Filesystem-Update
            //if (performUpdate(filesystemUrl, "filesystem")){
            // Firmware-Update
            performUpdate(firmwareUrl, "firmware");
            //}

            // Neustart nach erfolgreichem Update
            ESP.restart();

          } else {
            Serial.println("Keine neue Version verfügbar.");
            lighthouse(RED);
          }
        } else {
          Serial.println("Fehler beim Abrufen der Release-Informationen");
          lighthouse(RED);
        }
      }
      https.end();
    } else {
      Serial.printf("https Unable to connect\n");
      lighthouse(RED);
    }
    delete client;
  } else {
    Serial.println("Unable to create client");
    lighthouse(RED);
  }
}

int leftFiveDigits(double num) {
  return (int)num*100;
}

void push(int value) {
  if (top < MAX_SIZE - 1) {
    stack[++top] = value;
  }else{
    for(int i=0;i<MAX_SIZE;i++){
      stack[i]=stack[i+1];
    }
    stack[MAX_SIZE-1]=value;
  }
}

int pop() {
  if (top >= 0) {
    return stack[top--];
  }
  return -1; // Oder einen anderen Wert, der einen leeren Stack anzeigt
}

/************************************ Gestures ******************************/ 
void detectSequence(){
  Serial.printf("stack: %d%d%d\n", stack[0],stack[1],stack[2]);
  if (stack[0]==1){
    // up swing, point up
    if (stack[1]==2 && stack[2]==1 || stack[1]==3 && stack[2]==2){
      lumus();
      DacAudio.Play(&spell2);
      sequenceActive=true;
    }
  }
  if (sequenceActive && stack[0]==0 && stack[1]==0 && stack[2]==0){
      sequenceActive=false;
      DacAudio.StopAllSounds();
      digitalWrite(RED,false);
      digitalWrite(GREEN,false);
      digitalWrite(BLUE,false);
      DacAudio.Play(&spell1);
  }

}

void pointdown(){
  Serial.println("point down");
  push(0);
  detectSequence();
  gestureInProgress = false;
}

void pointup(){
  Serial.println("point up");
  lighthouse(GREEN);
  push(1);
  detectSequence();
  gestureInProgress = false;
}

void swingup(){
  Serial.println("swing up");
  lighthouse(RED);
  push(2);
  detectSequence();
  gestureInProgress = false;
}

void swingdown(){
  Serial.println("swing down");
  lighthouse(RED);
  push(3);
  detectSequence();
  gestureInProgress = false;
}

void rollleft(){
  Serial.println("roll left");
  lighthouse(BLUE);
  push(4);
  detectSequence();
  gestureInProgress = false;
}

void rollright(){
  Serial.println("roll right");
  lighthouse(BLUE);
  push(5);
  detectSequence();
  gestureInProgress = false;
}

void detectGesture() {
  /*
  Serial.print(round(accX));
  Serial.print("\t");
  Serial.print(round(accY));
  Serial.print("\t");
  Serial.print(round(accZ));
  Serial.print("\t");
  Serial.print(round(gyroX));
  Serial.print("\t");
  Serial.print(round(gyroY));
  Serial.print("\t");
  Serial.print(round(gyroZ));
  Serial.print("\n");
  */
  if (!gestureInProgress) {
    // pointing straight up or straight down or swing above threshold will start gesture
    if (abs(gyroX) > GESTURE_THRESHOLD || abs(gyroY) > GESTURE_THRESHOLD || accY >= 9 || accY <= -9) {
      gestureInProgress = true;
      gestureStartTime = millis();
    }
  } else {
    // if gesture duration passed
    if (millis() - gestureStartTime > GESTURE_DURATION) {
      // still pointing straight up or straight down
      if (accY <= -9){
        if (!sequenceActive){
          pointup();
          return;
        }
      }
      if (accY >= 9){
        pointdown();
        return;
      }
      // still swinging
      if (!sequenceActive){
        if (abs(gyroX) > abs(gyroY)) {
          if (gyroX > 0) {
            swingdown();
            return;
          } else {
            swingup();
            return;
          }
        } else {
          if (gyroY > 0) {
            rollright();
            return;
          } else {
            rollleft();
            return;
          }
        }
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
    playAlarm(GREEN,1200,1);
    playAlarm(BLUE,1400,1);
    playAlarm(GREEN,1800,1);
    playAlarm(BLUE,2000,1);
    playAlarm(RED,2200,1);
    playAlarm(RED,2200,1);
    playAlarm(RED,2200,1);
    playAlarm(RED,2200,1);
    playAlarm(RED,2200,1);
    ESP.deepSleep(0);
  }
}

/************************************ Initialize ******************************/ 

void debuglight() {
  Serial.print("RED has PWM: ");
  Serial.println(digitalPinHasPWM(RED));
  Serial.print("GREEN has PWM: ");
  Serial.println(digitalPinHasPWM(GREEN));
  Serial.print("BLUE has PWM: ");
  Serial.println(digitalPinHasPWM(BLUE));
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
    mpu_fail=true;
    Serial.println("Failed to find MPU6050 chip");
    return;
  }
  Serial.println("MPU6050 Found!");

  // Set the accelerometer range (2, 4, 8, or 16 G)
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // Set the bandwidth to the lowest setting
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  // Set the wake frequency (options are 1.25 Hz, 5 Hz, 20 Hz, 40 Hz)
  mpu.setCycleRate(MPU6050_CYCLE_5_HZ);

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
  startTime = millis();
  Serial.begin(115200);
  //while (!Serial)
  //  delay(10); // will pause until serial console opens
  Serial2.end();
  Serial1.end();
  delay(100);
  initmpu(SDA,SCL);
  pinMode(RED,   OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE,  OUTPUT);
  debuglight();
}

/************************************  Mainloop ******************************/ 

void loop() {
  Serial2.end();
  unsigned long currentTime = millis();
  if (!triggered && (currentTime - startTime >= triggerDelay)) {
    // This code block will execute once, 10 seconds after start
    Serial.println("10 seconds have passed!");
    if(untouched) {
      //SIGNAL
      playAlarm(BLUE,400,3);
      WiFiManager wifiManager;
      wifiManager.autoConnect("garrick");
      checkForUpdates();
      wifiManager.disconnect();
      WiFi.mode(WIFI_OFF);    // Switch WiFi off to save power
    }
    triggered = true; // Prevent this from executing again
  }
  DacAudio.FillBuffer();              //Fill the sound buffer with data
  if(gestureInProgress){
    untouched=false;
  }
  if (!mpu_fail){
    mpu.enableSleep(false);
    readMpu();
    detectGesture();
    //detectCircularMotion();
  }else{
    //mpu_fail Alarm
    playAlarm(RED,1000,2);
    playAlarm(BLUE,1000,2);
  }
  if(!spell1.Playing and !spell2.Playing){
    dacWrite(DAC, 0);          // if completed playing, play again
  }
  delay(1);
}
