#include <Arduino.h>
#include <ps4controller.h>
#include <math.h>

// --- 定数定義 ---
const int BASE_SPEED_DRIVE = 128;       // 足回り基本速度
const int DASH_INCREMENT = 64;          // ダッシュ時の加算値
const int BASE_SPEED_ACCESSORY = 20;    // 補機基本速度
const int DEADZONE = 15;
const float SMOOTHING = 0.2;            // スムージング係数（0.0～1.0）

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
const int FD_PWM = 26, FD_DIR = 27, FD_CH = 4;
// ハンド上下
const int HL_PWM = 4, HL_DIR = 2, HL_CH = 5;
// ハンド前後
const int HP_PWM = 21, HP_DIR = 25, HP_CH = 6;
// グリッパー開閉
const int HG_PWM = 23, HG_DIR = 22, HG_CH = 7;

// --- モーター回転方向反転フラグ ---
// trueなら速度符号を反転
const bool FL_INVERT = true;   // 左前
const bool FR_INVERT = false;  // 右前
const bool RL_INVERT = true;   // 左後
const bool RR_INVERT = false;  // 右後

// --- 変数 ---
int currentSpeedDrive = BASE_SPEED_DRIVE;               // 足回り速度
const int currentSpeedAccessory = BASE_SPEED_ACCESSORY; // 補機速度（固定）

bool l3_was_pressed = false;

// 足回り速度スムージング
float targetFL = 0, currentFL = 0;
float targetFR = 0, currentFR = 0;
float targetRL = 0, currentRL = 0;
float targetRR = 0, currentRR = 0;

// --- モーター制御関数 ---
void setMotor(int pwmPin, int dirPin, int channel, float speed, bool invert = false) {
  if (invert) speed = -speed;

  bool direction = speed >= 0;
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, direction ? HIGH : LOW);

  int pwm_val = (int)abs(speed);
  if (pwm_val > 255) pwm_val = 255;
  if (pwm_val < 10) pwm_val = 0;  // 10未満は停止

  ledcWrite(channel, pwm_val);
}

void setupMotor(int pwmPin, int dirPin, int channel) {
  pinMode(dirPin, OUTPUT);
  ledcSetup(channel, 1000, 8); // 1000Hz, 8bit
  ledcAttachPin(pwmPin, channel);
}

void stopAllMotors() {
  setMotor(FL_PWM, FL_DIR, FL_CH, 0);
  setMotor(FR_PWM, FR_DIR, FR_CH, 0);
  setMotor(RL_PWM, RL_DIR, RL_CH, 0);
  setMotor(RR_PWM, RR_DIR, RR_CH, 0);
  setMotor(FD_PWM, FD_DIR, FD_CH, 0);
  setMotor(HL_PWM, HL_DIR, HL_CH, 0);
  setMotor(HP_PWM, HP_DIR, HP_CH, 0);
  setMotor(HG_PWM, HG_DIR, HG_CH, 0);
}

// --- スムージング関数 ---
float smoothSpeed(float current, float target) {
  return current + (target - current) * SMOOTHING;
}

// --- 足回り制御（Lスティック + R1/L1で旋回） ---
void controlDrive() {
  int x = -PS4.LStickX();      // ← ここを反転
  int y = -PS4.LStickY();     // 前後移動（上を正）

  // 以下は変更なし
  if (abs(x) < DEADZONE) x = 0;
  if (abs(y) < DEADZONE) y = 0;

  float vx = x / 128.0;
  float vy = y / 128.0;
  float vr = 0;

  if (PS4.R1()) {
    vr = -1.0;
  } else if (PS4.L1()) {
    vr = 1.0;
  }

  float fl = (vy + vx + vr) * currentSpeedDrive;
  float fr = (vy - vx - vr) * currentSpeedDrive;
  float rl = (vy - vx + vr) * currentSpeedDrive;
  float rr = (vy + vx - vr) * currentSpeedDrive;

  targetFL = fl;
  targetFR = fr;
  targetRL = rl;
  targetRR = rr;

  currentFL = smoothSpeed(currentFL, targetFL);
  currentFR = smoothSpeed(currentFR, targetFR);
  currentRL = smoothSpeed(currentRL, targetRL);
  currentRR = smoothSpeed(currentRR, targetRR);

  setMotor(FL_PWM, FL_DIR, FL_CH, currentFL, FL_INVERT);
  setMotor(FR_PWM, FR_DIR, FR_CH, currentFR, FR_INVERT);
  setMotor(RL_PWM, RL_DIR, RL_CH, currentRL, RL_INVERT);
  setMotor(RR_PWM, RR_DIR, RR_CH, currentRR, RR_INVERT);
}

// --- 土台上下制御 ---
void controlPlatform() {
  if (PS4.Triangle() && PS4.Cross()) {
    setMotor(FD_PWM, FD_DIR, FD_CH, 0);
  } else if (PS4.Triangle()) {
    setMotor(FD_PWM, FD_DIR, FD_CH, currentSpeedAccessory);
  } else if (PS4.Cross()) {
    setMotor(FD_PWM, FD_DIR, FD_CH, -currentSpeedAccessory);
  } else {
    setMotor(FD_PWM, FD_DIR, FD_CH, 0);
  }
}

void controlHandVertical() {
  if (PS4.Up() && PS4.Down()) {
    setMotor(HL_PWM, HL_DIR, HL_CH, 0);
  } else if (PS4.Up()) {
    setMotor(HL_PWM, HL_DIR, HL_CH, currentSpeedAccessory - 60);  // 上昇（逆）
  } else if (PS4.Down()) {
    setMotor(HL_PWM, HL_DIR, HL_CH, -currentSpeedAccessory + 60);   // 下降
  } else {
    setMotor(HL_PWM, HL_DIR, HL_CH, 0);
  }
}

void controlHandForward() {
  int ry = -PS4.RStickY();  // 上方向を正とする
  if (abs(ry) < DEADZONE) {
    setMotor(HP_PWM, HP_DIR, HP_CH, 0);
  } else {
    float speed = (ry / 128.0) * currentSpeedAccessory;
    setMotor(HP_PWM, HP_DIR, HP_CH, speed);
  }
}

void controlGripper() {
  if (PS4.L2() && PS4.R2()) {
    setMotor(HG_PWM, HG_DIR, HG_CH, 0);
  } else if (PS4.L2()) {
    setMotor(HG_PWM, HG_DIR, HG_CH, currentSpeedAccessory / 15);   // 開く
  } else if (PS4.R2()) {
    setMotor(HG_PWM, HG_DIR, HG_CH, -currentSpeedAccessory / 15);  // 閉じる
  } else {
    setMotor(HG_PWM, HG_DIR, HG_CH, 0);
  }
}

// --- セットアップ ---
void setup() {
  Serial.begin(115200);
  PS4.begin("1a:2b:3c:01:01:02"); // 自分のPS4コントローラーのMACアドレスに置き換える
  Serial.println("Ready.");

  setupMotor(FL_PWM, FL_DIR, FL_CH);
  setupMotor(FR_PWM, FR_DIR, FR_CH);
  setupMotor(RL_PWM, RL_DIR, RL_CH);
  setupMotor(RR_PWM, RR_DIR, RR_CH);
  setupMotor(FD_PWM, FD_DIR, FD_CH);
  setupMotor(HL_PWM, HL_DIR, HL_CH);
  setupMotor(HP_PWM, HP_DIR, HP_CH);
  setupMotor(HG_PWM, HG_DIR, HG_CH);

  stopAllMotors();
}

// --- メインループ ---
void loop() {
  if (PS4.isConnected()) {
    // ダッシュモード切り替え（L3押下）
    if (PS4.L3() && !l3_was_pressed) {
      currentSpeedDrive = (currentSpeedDrive == BASE_SPEED_DRIVE)
                          ? (BASE_SPEED_DRIVE + DASH_INCREMENT)
                          : BASE_SPEED_DRIVE;
    }
    l3_was_pressed = PS4.L3();

    controlDrive();
    controlPlatform();
    controlHandVertical();
    controlHandForward();
    controlGripper();

    Serial.print("LStickX: "); Serial.print(PS4.LStickX());
    Serial.print(" | LStickY: "); Serial.print(PS4.LStickY());
    Serial.print(" | R1: "); Serial.print(PS4.R1());
    Serial.print(" | L1: "); Serial.print(PS4.L1());
    Serial.println();
  } else {
    stopAllMotors();
  }

  delay(20);
}