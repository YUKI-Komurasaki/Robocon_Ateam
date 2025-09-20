#include <Arduino.h>
#include <ps4controller.h>
#include <math.h>

// --- 定数定義 ---
const int BASE_SPEED = 128;
const int DASH_INCREMENT = 64;
const int DEADZONE = 10;

// --- モーター用ピン定義とPWMチャンネル ---
// 左前
const int FL_PWM = 33, FL_DIR = 32, FL_CH = 0;
// 右前
const int FR_PWM = 12, FR_DIR = 14, FR_CH = 1;
// 左後
const int RL_PWM = 16, RL_DIR = 17, RL_CH = 2;
// 右後
const int RR_PWM = 5,  RR_DIR = 18, RR_CH = 3;

// 土台上下
const int FD_PWM = 26, FD_DIR = 19, FD_CH = 4;
// ハンド上下
const int HL_PWM = 4, HL_DIR = 2, HL_CH = 5;
// ハンド前後
const int HP_PWM = 21,  HP_DIR = 25,  HP_CH = 6;
// グリッパー開閉
const int HG_PWM = 23, HG_DIR = 22, HG_CH = 7;

// --- 変数 ---
int LeftStickX = 0, LeftStickY = 0;
int LMoveAngle = 0;
int currentSpeed = BASE_SPEED;
bool l3_was_pressed = false;

// --- モーター制御関数（ESP32用） ---
void setMotor(int pwmPin, int dirPin, int channel, int speed) {
  bool direction = speed >= 0;
  digitalWrite(dirPin, direction ? HIGH : LOW);
  ledcWrite(channel, abs(speed));
}

// --- まとめて初期化 ---
void setupMotor(int pwmPin, int dirPin, int channel) {
  pinMode(dirPin, OUTPUT);
  ledcSetup(channel, 1000, 8);  // 周波数:1000Hz, 分解能:8bit
  ledcAttachPin(pwmPin, channel);
}

// --- 初期化 ---
void setup() {
  Serial.begin(115200);
  PS4.begin("1a:2b:3c:01:01:02");
  Serial.println("Ready.");

  // 全モーター初期化
  setupMotor(FL_PWM, FL_DIR, FL_CH);
  setupMotor(FR_PWM, FR_DIR, FR_CH);
  setupMotor(RL_PWM, RL_DIR, RL_CH);
  setupMotor(RR_PWM, RR_DIR, RR_CH);

  setupMotor(FD_PWM, FD_DIR, FD_CH);
  setupMotor(HL_PWM, HL_DIR, HL_CH);
  setupMotor(HP_PWM, HP_DIR, HP_CH);
  setupMotor(HG_PWM, HG_DIR, HG_CH);
}

// --- 走行用モーター制御 ---
void controlMotors(int fl, int fr, int rl, int rr) {
  setMotor(FL_PWM, FL_DIR, FL_CH, fl);
  setMotor(FR_PWM, FR_DIR, FR_CH, fr);
  setMotor(RL_PWM, RL_DIR, RL_CH, rl);
  setMotor(RR_PWM, RR_DIR, RR_CH, rr);
}

// --- 土台上下 ---
void controlPlatform() {
  if (PS4.Triangle() && PS4.Cross()) {
    setMotor(FD_PWM, FD_DIR, FD_CH, 0);
  } else if (PS4.Triangle()) {
    setMotor(FD_PWM, FD_DIR, FD_CH, BASE_SPEED - 60);
  } else if (PS4.Cross()) {
    setMotor(FD_PWM, FD_DIR, FD_CH, -BASE_SPEED + 60);
  } else {
    setMotor(FD_PWM, FD_DIR, FD_CH, 0);
  }
}

// --- ハンド上下 ---
void controlHandVertical() {
  if (PS4.Up() && PS4.Down()) {
    setMotor(HL_PWM, HL_DIR, HL_CH, 0);
  } else if (PS4.Up()) {
    setMotor(HL_PWM, HL_DIR, HL_CH, BASE_SPEED - 60);
  } else if (PS4.Down()) {
    setMotor(HL_PWM, HL_DIR, HL_CH, -BASE_SPEED - 60);
  } else {
    setMotor(HL_PWM, HL_DIR, HL_CH, 0);
  }
}

// --- ハンド前後 ---
void controlHandForward() {
  if (PS4.Right() && PS4.Left()) {
    setMotor(HP_PWM, HP_DIR, HP_CH, 0);
  } else if (PS4.Right()) {
    setMotor(HP_PWM, HP_DIR, HP_CH, BASE_SPEED);
  } else if (PS4.Left()) {
    setMotor(HP_PWM, HP_DIR, HP_CH, -BASE_SPEED);
  } else {
    setMotor(HP_PWM, HP_DIR, HP_CH, 0);
  }
}

// --- グリッパー開閉 ---
void controlGripper() {
  if (PS4.L2() && PS4.R2()) {
    setMotor(HG_PWM, HG_DIR, HG_CH, 0);
  } else if (PS4.L2()) {
    setMotor(HG_PWM, HG_DIR, HG_CH, BASE_SPEED / 15);
  } else if (PS4.R2()) {
    setMotor(HG_PWM, HG_DIR, HG_CH, -BASE_SPEED / 15);
  } else {
    setMotor(HG_PWM, HG_DIR, HG_CH, 0);
  }
}

// --- メインループ ---
void loop() {
  if (PS4.isConnected()) {

    LeftStickX = PS4.LStickX();
    LeftStickY = PS4.LStickY();

    if (abs(LeftStickX) < DEADZONE) LeftStickX = 0;
    if (abs(LeftStickY) < DEADZONE) LeftStickY = 0;

    LMoveAngle = (180 / PI) * atan2((double)LeftStickX, (double)(-1 * LeftStickY));

    // ダッシュ切替
    if (PS4.L3() && !l3_was_pressed) {
      currentSpeed = (currentSpeed == BASE_SPEED) ? BASE_SPEED + DASH_INCREMENT : BASE_SPEED;
    }
    l3_was_pressed = PS4.L3();

    // 移動処理
    if (LeftStickX != 0 || LeftStickY != 0) {
      if (LMoveAngle < -157 || LMoveAngle > 157) {
        controlMotors(currentSpeed, -currentSpeed, currentSpeed, -currentSpeed); // 前進
      } else if (LMoveAngle <= -113) {
        controlMotors(0, -currentSpeed, currentSpeed, 0); // 左前
      } else if (LMoveAngle < -67) {
        controlMotors(-currentSpeed, -currentSpeed, currentSpeed, currentSpeed); // 左
      } else if (LMoveAngle <= -23) {
        controlMotors(-currentSpeed, 0, 0, currentSpeed); // 左後
      } else if (LMoveAngle < 23) {
        controlMotors(-currentSpeed, currentSpeed, -currentSpeed, currentSpeed); // 後進
      } else if (LMoveAngle <= 67) {
        controlMotors(0, currentSpeed, -currentSpeed, 0); // 右後
      } else if (LMoveAngle < 113) {
        controlMotors(currentSpeed, currentSpeed, -currentSpeed, -currentSpeed); // 右
      } else if (LMoveAngle <= 157) {
        controlMotors(currentSpeed, 0, 0, -currentSpeed); // 右前
      }
    } else {
      // 回転処理
      if (PS4.L1() && PS4.R1()) {
        controlMotors(0, 0, 0, 0);
      } else if (PS4.L1()) {
        controlMotors(currentSpeed, -currentSpeed, currentSpeed, -currentSpeed); // 左回転
      } else if (PS4.R1()) {
        controlMotors(-currentSpeed, currentSpeed, -currentSpeed, currentSpeed); // 右回転
      } else {
        controlMotors(0, 0, 0, 0);  // 停止
      }
    }

    // 補機制御
    controlPlatform();
    controlHandVertical();
    controlHandForward();
    controlGripper();
  }

  delay(20);
}
