/* =========================================================
   コメント付きリファレンス版（ESP32 DevKitC）
   - PS4切断時の自動ブレーキ（自動復帰）
   - ハンド前後をRスティック上下で操作
   - L3で足回り速度2段階切替
   - 加速は速く、減速はゆっくり（転倒防止）
   ========================================================= */

#include <Arduino.h>
#include <ps4controller.h>
#include <math.h>

// =====================
// 基本設定（調整はここを編集）
// =====================
const int DEADZONE = 30;                // スティックデッドゾーン
int currentSpeedAccessory = 100;        // 土台 / ハンド / グリッパー速度

// 足回り速度（L3で切替）
const int DRIVE_SPEED_NORMAL = 128;
const int DRIVE_SPEED_HIGH   = 255;
int currentSpeedDrive = DRIVE_SPEED_NORMAL;
bool isHighSpeedMode = false;

// スムージング（加速/減速別）
const float ACCEL_SMOOTH = 0.25;  // 加速：大きめで素早く追従
const float BRAKE_SMOOTH = 0.10;  // 減速：小さめで滑らかに停止

// =====================
// ピン定義（最終割当）
// =====================
// 足回り
const int FL_PWM = 33, FL_DIR = 32, FL_CH = 0;
const int FR_PWM = 12, FR_DIR = 14, FR_CH = 1;
const int RL_PWM = 16, RL_DIR = 17, RL_CH = 2;
const int RR_PWM = 5,  RR_DIR = 18, RR_CH = 3;

// 土台（Cytron MD13S）
const int FD_PWM = 27, FD_DIR = 13, FD_CH = 4;

// ハンド上下
const int HL_PWM = 4, HL_DIR = 2, HL_CH = 6;
// ハンド前後（Rスティック上下）
const int HP_PWM = 21, HP_DIR = 26, HP_CH = 5;
// グリッパー
const int HG_PWM = 23, HG_DIR = 22, HG_CH = 7;

// 反転フラグ（必要に応じて true に）
const bool FL_INVERT = true;
const bool FR_INVERT = false;
const bool RL_INVERT = true;
const bool RR_INVERT = false;
const bool FD_INVERT = false;
const bool HP_INVERT = false;

// =====================
// モーター初期化 / 制御関数
// =====================
void setupMotor(int pwmPin, int dirPin, int channel) {
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW); // 初期安全：DIR=LOW
  ledcSetup(channel, 1000, 8); // 1kHz, 8bit
  ledcAttachPin(pwmPin, channel);
  ledcWrite(channel, 0); // 初期停止
}

/**
 * setMotor:
 *  - pwmPin: PWM 出力ピン（ledcAttachPinしたピン）
 *  - dirPin: DIR 出力ピン（HIGH=正転, LOW=逆転）
 *  - channel: ledc channel
 *  - speed: -255 .. +255 (負は逆転)
 *  - invert: true なら符号反転
 */
void setMotor(int pwmPin, int dirPin, int channel, float speed, bool invert=false) {
  if (invert) speed = -speed;
  // 範囲制限
  speed = constrain(speed, -255, 255);

  // 方向とPWM出力
  bool direction = (speed >= 0);
  digitalWrite(dirPin, direction ? HIGH : LOW);
  ledcWrite(channel, (int)abs(speed));
}

// =====================
// 全モーター即時停止（安全ブレーキ）
// =====================
void stopAllMotors() {
  int dirPins[] = {FL_DIR, FR_DIR, RL_DIR, RR_DIR, FD_DIR, HL_DIR, HP_DIR, HG_DIR};
  int channels[] = {FL_CH, FR_CH, RL_CH, RR_CH, FD_CH, HL_CH, HP_CH, HG_CH};
  for (int i = 0; i < 8; ++i) {
    ledcWrite(channels[i], 0);        // PWM = 0
    digitalWrite(dirPins[i], LOW);    // DIR = LOW（安全側）
  }
}

// =====================
// タスク群（機能ごとに分割）
// =====================

// ---------- PS4接続監視 ----------
void connectionMonitorTask(void* pvParameters) {
  bool wasConnected = PS4.isConnected();
  for (;;) {
    bool isConnected = PS4.isConnected();
    if (wasConnected && !isConnected) {
      Serial.println("⚠️ PS4 disconnected! Stopping all motors for safety.");
      stopAllMotors();
    } else if (!wasConnected && isConnected) {
      Serial.println("✅ PS4 reconnected. Control resumed.");
    }
    wasConnected = isConnected;
    vTaskDelay(100 / portTICK_PERIOD_MS); // 100 ms 間隔で監視
  }
}

// ---------- 足回り（加速は速く、減速はゆっくり） ----------
void driveTask(void* pvParameters) {
  float fl = 0, fr = 0, rl = 0, rr = 0;                  // 現在値
  float fl_target = 0, fr_target = 0, rl_target = 0, rr_target = 0; // 目標値

  for (;;) {
    if (PS4.isConnected()) {
      int x = -PS4.LStickX();
      int y = -PS4.LStickY();
      if (abs(x) < DEADZONE) x = 0;
      if (abs(y) < DEADZONE) y = 0;

      float vx = x / 128.0;
      float vy = y / 128.0;
      float vr = 0;
      if (PS4.R1()) vr = -1.0;
      else if (PS4.L1()) vr = 1.0;

      // 目標速度（スケールは currentSpeedDrive）
      fl_target = (vy + vx + vr) * currentSpeedDrive;
      fr_target = (vy - vx - vr) * currentSpeedDrive;
      rl_target = (vy - vx + vr) * currentSpeedDrive;
      rr_target = (vy + vx - vr) * currentSpeedDrive;
    } else {
      // 切断時は即目標0へ（監視タスクで安全停止も行う）
      fl_target = fr_target = rl_target = rr_target = 0;
    }

    // 加速／減速で異なるスムージング係数を使う
    fl += (fl_target - fl) * ((abs(fl_target) > abs(fl)) ? ACCEL_SMOOTH : BRAKE_SMOOTH);
    fr += (fr_target - fr) * ((abs(fr_target) > abs(fr)) ? ACCEL_SMOOTH : BRAKE_SMOOTH);
    rl += (rl_target - rl) * ((abs(rl_target) > abs(rl)) ? ACCEL_SMOOTH : BRAKE_SMOOTH);
    rr += (rr_target - rr) * ((abs(rr_target) > abs(rr)) ? ACCEL_SMOOTH : BRAKE_SMOOTH);

    // 出力
    setMotor(FL_PWM, FL_DIR, FL_CH, fl, FL_INVERT);
    setMotor(FR_PWM, FR_DIR, FR_CH, fr, FR_INVERT);
    setMotor(RL_PWM, RL_DIR, RL_CH, rl, RL_INVERT);
    setMotor(RR_PWM, RR_DIR, RR_CH, rr, RR_INVERT);

    vTaskDelay(20 / portTICK_PERIOD_MS); // 20ms 周期（50Hz）
  }
}

// ---------- 土台（△/×） ----------
void platformTask(void* pvParameters) {
  for (;;) {
    float speed = 0;
    if (PS4.isConnected()) {
      if (PS4.Triangle() && PS4.Cross()) speed = 0;
      else if (PS4.Triangle()) speed = currentSpeedAccessory;
      else if (PS4.Cross()) speed = -currentSpeedAccessory;
      else speed = 0;
    }
    setMotor(FD_PWM, FD_DIR, FD_CH, speed, FD_INVERT);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ---------- ハンド上下（↑/↓） ----------
void handVerticalTask(void* pvParameters) {
  for (;;) {
    float speed = 0;
    if (PS4.isConnected()) {
      if (PS4.Up() && PS4.Down()) speed = 0;
      else if (PS4.Up()) speed = -currentSpeedAccessory + 60;
      else if (PS4.Down()) speed = currentSpeedAccessory - 60;
      else speed = 0;
    }
    setMotor(HL_PWM, HL_DIR, HL_CH, speed);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ---------- ハンド前後（Rスティック上下） ----------
void handForwardTask(void* pvParameters) {
  for (;;) {
    float speed = 0;
    if (PS4.isConnected()) {
      int y = -PS4.RStickY();
      if (abs(y) < DEADZONE) y = 0;
      speed = (y / 128.0) * currentSpeedAccessory;
    } else {
      speed = 0;
    }

    if (speed == 0) {
      ledcWrite(HP_CH, 0);
      digitalWrite(HP_DIR, LOW);
    } else {
      setMotor(HP_PWM, HP_DIR, HP_CH, speed, HP_INVERT);
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ---------- グリッパー（L2/R2） ----------
void gripperTask(void* pvParameters) {
  const float GRIPPER_SPEED = currentSpeedAccessory;
  for (;;) {
    float speed = 0;
    if (PS4.isConnected()) {
      if (PS4.L2() && PS4.R2()) speed = 0;
      else if (PS4.L2()) speed = GRIPPER_SPEED;
      else if (PS4.R2()) speed = -GRIPPER_SPEED;
      else speed = 0;
    }
    setMotor(HG_PWM, HG_DIR, HG_CH, speed);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// ---------- L3で足回り速度2段切替 ----------
void speedModeTask(void* pvParameters) {
  bool prevL3 = false;
  for (;;) {
    if (PS4.isConnected()) {
      bool nowL3 = PS4.L3();
      if (nowL3 && !prevL3) {
        isHighSpeedMode = !isHighSpeedMode;
        currentSpeedDrive = isHighSpeedMode ? DRIVE_SPEED_HIGH : DRIVE_SPEED_NORMAL;
        Serial.printf("[MODE] Drive speed: %s\n", isHighSpeedMode ? "HIGH" : "NORMAL");
      }
      prevL3 = nowL3;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// =====================
// setup / loop
// =====================
void setup() {
  Serial.begin(115200);
  PS4.begin("1a:2b:3c:01:01:02"); // PS4 コントローラ MAC（プロジェクトごとに変更可）
  Serial.println("System ready.");

  // モーター初期化（各チャネルの ledcAttachPin と DIR 初期化）
  setupMotor(FL_PWM, FL_DIR, FL_CH);
  setupMotor(FR_PWM, FR_DIR, FR_CH);
  setupMotor(RL_PWM, RL_DIR, RL_CH);
  setupMotor(RR_PWM, RR_DIR, RR_CH);
  setupMotor(FD_PWM, FD_DIR, FD_CH);
  setupMotor(HL_PWM, HL_DIR, HL_CH);
  setupMotor(HP_PWM, HP_DIR, HP_CH);
  setupMotor(HG_PWM, HG_DIR, HG_CH);

  // タスク起動（優先度は必要なら調整）
  xTaskCreate(driveTask, "Drive", 2048, NULL, 1, NULL);
  xTaskCreate(platformTask, "Platform", 2048, NULL, 1, NULL);
  xTaskCreate(handVerticalTask, "HandVert", 2048, NULL, 1, NULL);
  xTaskCreate(handForwardTask, "HandFwd", 2048, NULL, 1, NULL);
  xTaskCreate(gripperTask, "Gripper", 2048, NULL, 1, NULL);
  xTaskCreate(speedModeTask, "SpeedMode", 2048, NULL, 1, NULL);
  xTaskCreate(connectionMonitorTask, "ConnMon", 2048, NULL, 2, NULL); // 優先度高めで安全監視
}

void loop() {
  // ループは空。FreeRTOSタスクに処理を任せる設計
}