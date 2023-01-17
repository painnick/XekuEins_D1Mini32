#include <Arduino.h>
#include <ESP32Servo.h>

#define US_MIN 544
#define US_MAX 2000

class Scene1 {

public:
    Scene1(uint8_t eye, uint8_t blueCh, uint8_t gun, Servo* waist, Servo* arm) {
        this->eyePin = eye;
        this->blueChannel = blueCh;
        this->gunPin = gun;
        this->waistServo = waist;
        this->armServo = arm;
    }

    void run() {

        digitalWrite(this->eyePin, HIGH);
        delay(500);

        int us_center = map(90, 0, 180, US_MIN, US_MAX);
        int us_waist_max = map(180, 0, 180, US_MIN, US_MAX);
        int us_arm_max = US_MAX;

        this->armServo->writeMicroseconds(us_center);
        this->waistServo->writeMicroseconds(us_center);
        delay(100);

        for(int i = us_center; i < us_waist_max; i += 2) {
            this->waistServo->writeMicroseconds(i);
            delay(5);
        }

        for(int i = us_center; i < us_arm_max; i += 10) {
            this->armServo->writeMicroseconds(i);
            delay(10);
        }

        digitalWrite(this->gunPin, HIGH);
        delay(500);
        digitalWrite(this->gunPin, LOW);
        delay(500);
        digitalWrite(this->gunPin, HIGH);
        delay(500);
        digitalWrite(this->gunPin, LOW);

        for(int i = us_arm_max; i > us_center; i -= 10) {
            this->armServo->writeMicroseconds(i);
            delay(10);
        }

        for(int i = us_waist_max; i > us_center; i -= 2) {
            this->waistServo->writeMicroseconds(i);
            delay(5);
        }

        digitalWrite(this->eyePin, LOW);


        for(int i = 0; i < 100; i ++) {
            ledcWrite(this->blueChannel, 32 + random() % 96);
            delay(10 + (random() % 50));
        }
        ledcWrite(this->blueChannel, 0);

        digitalWrite(this->eyePin, HIGH);
        delay(2000);
        digitalWrite(this->eyePin, LOW);
    }


protected:
    uint8_t eyePin, blueChannel, gunPin;

    Servo* waistServo;
    Servo* armServo;

    int waist = 90, minWaist = 90 - 20, maxWaist = 90 + 60;
    bool dirWaist = true;
    int rightArm = 90, minRightArm = 90 - 10, maxRightArm = 90 + 60;
    bool dirRightArm = true;
};
