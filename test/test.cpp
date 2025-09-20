#include <Arduino.h>
#include <ps4controller.h>
#include "CytronMotorDriver.h"

// モーターインスタンス：PWM, DIR, チャンネル番号
CytronMD motorFL((MODE)PWM_DIR, 33, 32, 0); // 左前
CytronMD motorFR((MODE)PWM_DIR, 35, 34, 1); // 右前
CytronMD motorRL((MODE)PWM_DIR, 16, 17, 2); // 左後
CytronMD motorRR((MODE)PWM_DIR, 5, 18, 3); // 右後

CytronMD motorFD((MODE)PWM_DIR, 23, 22, 4);   // 土台上下
CytronMD motorHL((MODE)PWM_DIR, 21, 19, 5);  // ハンド上下
CytronMD motorHP((MODE)PWM_DIR, 1, 3, 6);  // ハンド前後（台）
CytronMD motorHG((MODE)PWM_DIR, 36, 39, 7);  // グリッパー開閉

// 動作確認
void setup() {
  Serial.begin(115200);
  PS4.begin("1a:2b:3c:01:01:02");
  Serial.println("Ready");
}

void loop() {
  Serial.begin(115200);

  motorHP.setSpeed(20);
  delay(500);

  motorHP.setSpeed(-20);
  delay(500);

  motorHP.setSpeed(0);
}
