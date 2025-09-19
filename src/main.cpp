#include <Arduino.h>
#include <ps4controller.h>
#include <CytronMotorDriver.h>
#include <math.h>



// --- 定数定義 ---
const int BASE_SPEED = 128;      // 通常速度
const int DASH_INCREMENT = 64;   // ダッシュ速度の増分
const int DEADZONE = 10;         // ジョイスティックのドリフト対策

// --- モーター定義 ---
//CytronMD motorFL(PWM_DIR, 33, 32);
//CytronMD motorFR(PWM_DIR, 35, 34);
//CytronMD motorRL(PWM_DIR, 16, 17);
//CytronMD motorRR(PWM_DIR, 5, 18);
CytronMD motorFL((MODE)PWM_DIR, 33, 32, 0);
CytronMD motorFR((MODE)PWM_DIR, 35, 34, 1);
CytronMD motorRL((MODE)PWM_DIR, 16, 17, 2);
CytronMD motorRR((MODE)PWM_DIR, 5, 18, 3);

/*
 * 【重要】以下の4つのモーターは、それぞれ異なる物理ピンに接続し、
 * 実際の配線に合わせてピン番号を変更してください。
 * (下記のピン番号はあくまで重複を避けるための仮設定です)
 */
CytronMD motorFD(PWM_DIR, 2, 15, 4);   // 土台上下
CytronMD motorHL(PWM_DIR, 19, 21, 5);  // ハンド上下
CytronMD motorHP(PWM_DIR, 22, 23, 6);  // ハンド前後（台）
CytronMD motorHG(PWM_DIR, 25, 26, 7);  // グリッパー開閉

// --- 変数 ---
int LeftStickX = 0, LeftStickY = 0;
int LMoveAngle = 0;
int currentSpeed = BASE_SPEED; // 現在の基準速度
bool l3_was_pressed = false;   // L3ボタンのチャタリング防止用フラグ

// --- 初期化 ---
void setup() {
  Serial.begin(115200);
  PS4.begin("1a:2b:3c:01:01:02");   // PS4コントローラーのMACアドレス
  Serial.println("Ready.");
}

// --- モーター制御関数 ---
void controlMotors(int fl, int fr, int rl, int rr) {
  motorFL.setSpeed(fl);
  motorFR.setSpeed(fr);
  motorRL.setSpeed(rl);
  motorRR.setSpeed(rr);
}

void controlPlatform() {
  if (PS4.Triangle() && PS4.Cross()) {
    motorFD.setSpeed(0);
    Serial.println("土台 停止");
  } else if (PS4.Triangle()) {
    motorFD.setSpeed(BASE_SPEED);
    Serial.println("土台 上昇");
  } else if (PS4.Cross()) {
    motorFD.setSpeed(-BASE_SPEED);
    Serial.println("土台 下降");
  } else {
    motorFD.setSpeed(0);
  }
}

void controlHandVertical() {
  if (PS4.Up() && PS4.Down()) {
    motorHL.setSpeed(0);
    Serial.println("ハンド 停止");
  } else if (PS4.Up()) {
    motorHL.setSpeed(BASE_SPEED);
    Serial.println("ハンド 上昇");
  } else if (PS4.Down()) {
    motorHL.setSpeed(-BASE_SPEED);
    Serial.println("ハンド 下降");
  } else {
    motorHL.setSpeed(0);
  }
}

void controlHandForward() {
  if (PS4.Right() && PS4.Left()) {
    motorHP.setSpeed(0);
    Serial.println("ハンド台 停止");
  } else if (PS4.Right()) {
    motorHP.setSpeed(BASE_SPEED);
    Serial.println("ハンド台 前進");
  } else if (PS4.Left()) {
    motorHP.setSpeed(-BASE_SPEED);
    Serial.println("ハンド台 後退");
  } else {
    motorHP.setSpeed(0);
  }
}

void controlGripper() {
  if (PS4.L2() && PS4.R2()) {
    motorHG.setSpeed(0);
    Serial.println("グリッパー 停止");
  } else if (PS4.L2()) {
    motorHG.setSpeed(BASE_SPEED / 10);
    Serial.println("グリッパー 閉じる");
  } else if (PS4.R2()) {
    motorHG.setSpeed(-BASE_SPEED / 10);
    Serial.println("グリッパー 開く");
  } else {
    motorHG.setSpeed(0);
  }
}

// --- メインループ ---
void loop() {
  if (PS4.isConnected()) {

    // --- スティック入力 ---
    LeftStickX = PS4.LStickX();
    LeftStickY = PS4.LStickY();

    // --- デッドゾーン処理 ---
    if (abs(LeftStickX) < DEADZONE) LeftStickX = 0;
    if (abs(LeftStickY) < DEADZONE) LeftStickY = 0;

    // --- 角度計算（-180〜180）---
    LMoveAngle = (180 / PI) * atan2((double)LeftStickX, (double)(-1 * LeftStickY));

    // --- ★修正点: ダッシュモード切替（チャタリング防止）---
    if (PS4.L3() && !l3_was_pressed) {
      currentSpeed = (currentSpeed == BASE_SPEED) ? BASE_SPEED + DASH_INCREMENT : BASE_SPEED;
    }
    l3_was_pressed = PS4.L3();

    // --- 移動処理 ---
    if (LeftStickX != 0 || LeftStickY != 0) {
      if (LMoveAngle < -157 || LMoveAngle > 157) {  // 前進
        controlMotors(-currentSpeed, currentSpeed, -currentSpeed, currentSpeed);
      } else if (LMoveAngle <= -113) {  // 左前
        controlMotors(0, currentSpeed, -currentSpeed, 0);
      } else if (LMoveAngle < -67) {  // 左
        controlMotors(currentSpeed, currentSpeed, -currentSpeed, -currentSpeed);
      } else if (LMoveAngle <= -23) {  // 左後
        controlMotors(currentSpeed, 0, 0, -currentSpeed);
      } else if (LMoveAngle < 23) {  // 後進
        controlMotors(currentSpeed, -currentSpeed, currentSpeed, -currentSpeed);
      } else if (LMoveAngle <= 67) {  // 右後
        controlMotors(0, -currentSpeed, currentSpeed, 0);
      } else if (LMoveAngle < 113) {  // 右
        controlMotors(-currentSpeed, -currentSpeed, currentSpeed, currentSpeed);
      } else if (LMoveAngle <= 157) {  // 右前
        controlMotors(-currentSpeed, 0, 0, currentSpeed);
      }
    } else {
      // --- ★修正点: 回転処理 ---
      if (PS4.L1() && PS4.R1()) {
        controlMotors(0, 0, 0, 0);  // 停止
      } else if (PS4.L1()) {
        // 左回転 (反時計回り)
        controlMotors(-currentSpeed, currentSpeed, -currentSpeed, currentSpeed);
      } else if (PS4.R1()) {
        // 右回転 (時計回り)
        controlMotors(currentSpeed, -currentSpeed, currentSpeed, -currentSpeed);
      } else {
        controlMotors(0, 0, 0, 0);  // 停止
      }
    }

    // --- 補機の制御 ---
    controlPlatform();        // 土台 上下
    controlHandVertical();    // ハンド 上下
    controlHandForward();     // ハンド前後（台）
    controlGripper();         // グリッパー 開閉
  }

  // ★修正点: 応答性向上のためdelay値を調整
  delay(20);
}