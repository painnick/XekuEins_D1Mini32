#include <Arduino.h>

#include "esp_log.h"

#include <ESP32Servo.h>

#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems

// #define USE_SOUND

#ifdef USE_SOUND
#include "DFMiniMp3.h"
#endif

#define MAIN_TAG "Main"

// These are all GPIO pins on the ESP32
// Recommended pins include 2,4,12-19,21-23,25-27,32-33
// for the ESP32-S2 the GPIO pins are 1-21,26,33-42

// 13 outputs PWM signal at boot
// 14 outputs PWM signal at boot

#ifdef USE_SOUND
#define PIN_RX 16 // RX2
#define PIN_TX 17 // TX2
#endif

#define PIN_WAIST 22
#define PIN_RIGHT_ARM 21

#define PIN_BLUE 25 // Fixed
#define PIN_EYE 27 // Fixed
#define PIN_GUN 32

#define CHANNEL_BLUE 12

Servo servoWaist;
Servo servoRightArm;

#ifdef USE_SOUND
class Mp3Notify;
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3;
HardwareSerial mySerial(2); // 16, 17
DfMp3 dfmp3(mySerial);
int volume = MAX_VOLUME; // 0~30
#endif

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);

  servoWaist.attach(PIN_WAIST);
  servoWaist.write(90);
  servoRightArm.attach(PIN_RIGHT_ARM);
  servoRightArm.write(90);

  pinMode(PIN_BLUE, OUTPUT);
  pinMode(PIN_EYE, OUTPUT);
  pinMode(PIN_GUN, OUTPUT);

  ledcSetup(CHANNEL_BLUE, 1000, 8);
  ledcAttachPin(PIN_BLUE, CHANNEL_BLUE);
  ledcWrite(CHANNEL_BLUE, 128);

  digitalWrite(PIN_EYE, HIGH);
  digitalWrite(PIN_GUN, HIGH);

#ifdef USE_SOUND
  dfmp3.begin(9600, 1000);
  dfmp3.reset();

  while(!dfmp3.isOnline()) {
    delay(10);
  }

  dfmp3.setVolume(volume);
#endif
}

bool eyeOn = true;

unsigned long lastBlue = 0;
unsigned long lastEye = 0;
unsigned long lastWaist = 0;
unsigned long lastRightArm = 0;

int waist = 90, minWaist = 90 - 20, maxWaist = 90 + 20;
int rightArm = 90, minRightArm = 90 - 20, maxRightArm = 90 + 20;

void loop() {
#ifdef USE_SOUND
  dfmp3.loop();
#endif

  unsigned long now = millis();

  if (now - lastBlue > 100) {
    ledcWrite(CHANNEL_BLUE, 32 + random() % 96);
    lastBlue = now;
  }

  if (now - lastEye > 2000) {
    digitalWrite(PIN_EYE, eyeOn ? LOW : HIGH);
    eyeOn = !eyeOn;
    lastEye = now;
  }

  if (now - lastWaist > 100) {
    waist = (waist + 1);
    waist = max(waist, minWaist);
    waist = min(waist, maxWaist);
    servoWaist.write(waist);
    lastWaist = now;
  }

  if (now - lastRightArm > 100) {
    rightArm = (rightArm + 1);
    rightArm = max(rightArm, minRightArm);
    rightArm = min(rightArm, maxRightArm);
    servoRightArm.write(rightArm);
    lastRightArm = now;
  }

  delay(10);
}

#ifdef USE_SOUND
//----------------------------------------------------------------------------------
class Mp3Notify
{
public:
  static void PrintlnSourceAction(DfMp3_PlaySources source, const char *action)
  {
    if (source & DfMp3_PlaySources_Sd) {
      ESP_LOGD(MAIN_TAG, "SD Card, %s", action);
    }
    if (source & DfMp3_PlaySources_Usb) {
      ESP_LOGD(MAIN_TAG, "USB Disk, %s", action);
    }
    if (source & DfMp3_PlaySources_Flash) {
      ESP_LOGD(MAIN_TAG, "Flash, %s", action);
    }
  }
  static void OnError(DfMp3 &mp3, uint16_t errorCode)
  {
    // see DfMp3_Error for code meaning
    Serial.println();
    Serial.print("Com Error ");
    switch (errorCode)
    {
    case DfMp3_Error_Busy:
      ESP_LOGW(MAIN_TAG, "Com Error - Busy");
      break;
    case DfMp3_Error_Sleeping:
      ESP_LOGW(MAIN_TAG, "Com Error - Sleeping");
      break;
    case DfMp3_Error_SerialWrongStack:
      ESP_LOGW(MAIN_TAG, "Com Error - Serial Wrong Stack");
      break;

    case DfMp3_Error_RxTimeout:
      ESP_LOGW(MAIN_TAG, "Com Error - Rx Timeout!!!");
      break;
    case DfMp3_Error_PacketSize:
      ESP_LOGW(MAIN_TAG, "Com Error - Wrong Packet Size!!!");
      break;
    case DfMp3_Error_PacketHeader:
      ESP_LOGW(MAIN_TAG, "Com Error - Wrong Packet Header!!!");
      break;
    case DfMp3_Error_PacketChecksum:
      ESP_LOGW(MAIN_TAG, "Com Error - Wrong Packet Checksum!!!");
      break;

    default:
      ESP_LOGW(MAIN_TAG, "Com Error - %d", errorCode);
      break;
    }
  }
  static void OnPlayFinished(DfMp3 &mp3, DfMp3_PlaySources source, uint16_t track)
  {
    ESP_LOGD(MAIN_TAG, "Play finished for #%d", track);
  }
  static void OnPlaySourceOnline(DfMp3 &mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "online");
  }
  static void OnPlaySourceInserted(DfMp3 &mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "inserted");
  }
  static void OnPlaySourceRemoved(DfMp3 &mp3, DfMp3_PlaySources source)
  {
    PrintlnSourceAction(source, "removed");
  }
};
#endif
