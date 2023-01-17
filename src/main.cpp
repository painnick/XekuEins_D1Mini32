#include <Arduino.h>

#include "esp_log.h"

#include <ESP32Servo.h>

#include "soc/soc.h"             // disable brownout problems
#include "soc/rtc_cntl_reg.h"    // disable brownout problems

// #define USE_SOUND

#ifdef USE_SOUND
#include "DFMiniMp3.h"
#endif

#include "scene1.h"

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

#define PIN_WAIST 21
#define PIN_ARM 22

#define PIN_EYE 27 // Fixed
#define PIN_BLUE 25 // Fixed
#define PIN_GUN 32

#define CHANNEL_BLUE 12

Servo servoWaist;
Servo servoArm;

#ifdef USE_SOUND
class Mp3Notify;
typedef DFMiniMp3<HardwareSerial, Mp3Notify> DfMp3;
HardwareSerial mySerial(2); // 16, 17
DfMp3 dfmp3(mySerial);
int volume = MAX_VOLUME; // 0~30
#endif

Scene1 scene1(PIN_EYE, CHANNEL_BLUE, PIN_GUN, &servoWaist, &servoArm);

void setup() {
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);

  servoWaist.attach(PIN_WAIST);
  servoArm.attach(PIN_ARM);

  pinMode(PIN_BLUE, OUTPUT);
  pinMode(PIN_EYE, OUTPUT);
  pinMode(PIN_GUN, OUTPUT);

  ledcSetup(CHANNEL_BLUE, 1000, 8);
  ledcAttachPin(PIN_BLUE, CHANNEL_BLUE);
  // ledcWrite(CHANNEL_BLUE, 128);

#ifdef USE_SOUND
  dfmp3.begin(9600, 1000);
  dfmp3.reset();

  while(!dfmp3.isOnline()) {
    delay(10);
  }

  dfmp3.setVolume(volume);
#endif
}

void loop() {
  scene1.run();
  delay(1000 * 15);
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
