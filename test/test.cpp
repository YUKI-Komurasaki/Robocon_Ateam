#include <Arduino.h>
#include <ps4controller.h>
#include "CytronMotorDriver.h"

// モーターインスタンス：PWM, DIR, チャンネル番号
CytronMD motorFL(PWM_DIR, 33, 32, 0);
CytronMD motorFR(PWM_DIR, 35, 34, 1);
CytronMD motorRL(PWM_DIR, 16, 17, 2);
CytronMD motorRR(PWM_DIR, 5, 18, 3);

// 動作確認
void setup() {
  Serial.begin(115200);
  PS4.begin("1a:2b:3c:01:01:02");
  Serial.println("Ready");
}

void loop() {
  if (PS4.isConnected()) {
    int y = PS4.LStickY();

    // 停止
    if (abs(y) < 10) {
      motorFL.setSpeed(0);
      motorFR.setSpeed(0);
      motorRL.setSpeed(0);
      motorRR.setSpeed(0);
      return;
    }

    int speed = map(abs(y), 0, 127, 0, 255);
    speed = (y < 0) ? -speed : speed;

    motorFL.setSpeed(speed);
    motorFR.setSpeed(speed);
    motorRL.setSpeed(speed);
    motorRR.setSpeed(speed);
  }

  delay(20);
}
