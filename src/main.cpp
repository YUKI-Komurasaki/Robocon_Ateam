#include <Arduino.h>
#include <ps4controller.h>
#include <math.h>

// --- 基本設定 ---
const int BASE_SPEED_DRIVE = 128;
int currentSpeedAccessory = 100;
const int DEADZONE = 30;            // スティックのデッドゾーン
const float DECEL_TIME = 1.0f;      // 停止までの減速時間（秒）※ここを変更して調整可能

// --- モーター制御ピン設定 ---
const int FL_PWM = 33, FL_DIR = 32, FL_CH = 0;
const int FR_PWM = 12, FR_DIR = 14, FR_CH = 1;
const int RL_PWM = 16, RL_DIR = 17, RL_CH = 2;
const int RR_PWM = 5,  RR_DIR = 18, RR_CH = 3;

const int FD_PWM = 27, FD_DIR = 13, FD_CH = 4;   // 土台（MD13S）
const int HL_PWM = 4, HL_DIR = 2, HL_CH = 6;     // ハンド上下
const int HP_PWM = 21, HP_DIR = 26, HP_CH = 5;   // ハンド前後
const int HG_PWM = 23, HG_DIR = 22, HG_CH = 7;   // グリッパー

// --- 反転フラグ ---
const bool FL_INVERT = true;
const bool FR_INVERT = false;
const bool RL_INVERT = true;
const bool RR_INVERT = false;
const bool FD_INVERT = false;
const bool HP_INVERT = false;

// --- グローバル状態 ---
int currentSpeedDrive = BASE_SPEED_DRIVE;
float last_fl = 0, last_fr = 0, last_rl = 0, last_rr = 0;
unsigned long lastConnectedTime = 0;
bool isDecelerating = false;

// --- 初期化 ---
void setupMotor(int pwmPin, int dirPin, int channel) {
  pinMode(dirPin, OUTPUT);
  ledcSetup(channel, 1000, 8);
  ledcAttachPin(pwmPin, channel);
  ledcWrite(channel, 0);
  digitalWrite(dirPin, LOW);
}

void setMotor(int pwmPin, int dirPin, int channel, float speed, bool invert = false) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);
  bool direction = (speed >= 0);
  digitalWrite(dirPin, direction ? HIGH : LOW);
  ledcWrite(channel, (int)abs(speed));
}

// --- 足回りタスク ---
void driveTask(void* pvParameters) {
  for (;;) {
    float fl = 0, fr = 0, rl = 0, rr = 0;
    bool connected = PS4.isConnected();

    if (connected) {
      int x = -PS4.LStickX();
      int y = -PS4.LStickY();
      if (abs(x) < DEADZONE) x = 0;
      if (abs(y) < DEADZONE) y = 0;

      float vx = x / 128.0;
      float vy = y / 128.0;
      float vr = 0;

      if (PS4.R1()) vr = -1.0;
      else if (PS4.L1()) vr = 1.0;

      fl = (vy + vx + vr) * currentSpeedDrive;
      fr = (vy - vx - vr) * currentSpeedDrive;
      rl = (vy - vx + vr) * currentSpeedDrive;
      rr = (vy + vx - vr) * currentSpeedDrive;

      lastConnectedTime = millis();
      isDecelerating = false;
    } 
    else {
      // コントローラ切断時 → 減速停止
      if (!isDecelerating) {
        unsigned long start = millis();
        while (millis() - start < DECEL_TIME * 1000) {
          float factor = 1.0 - ((millis() - start) / (DECEL_TIME * 1000.0));
          setMotor(FL_PWM, FL_DIR, FL_CH, last_fl * factor, FL_INVERT);
          setMotor(FR_PWM, FR_DIR, FR_CH, last_fr * factor, FR_INVERT);
          setMotor(RL_PWM, RL_DIR, RL_CH, last_rl * factor, RL_INVERT);
          setMotor(RR_PWM, RR_DIR, RR_CH, last_rr * factor, RR_INVERT);
          vTaskDelay(20 / portTICK_PERIOD_MS);
        }
        isDecelerating = true;
      }
      fl = fr = rl = rr = 0;
    }

    setMotor(FL_PWM, FL_DIR, FL_CH, fl, FL_INVERT);
    setMotor(FR_PWM, FR_DIR, FR_CH, fr, FR_INVERT);
    setMotor(RL_PWM, RL_DIR, RL_CH, rl, RL_INVERT);
    setMotor(RR_PWM, RR_DIR, RR_CH, rr, RR_INVERT);

    last_fl = fl;
    last_fr = fr;
    last_rl = rl;
    last_rr = rr;

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- 土台上下 ---
void platformTask(void* pvParameters) {
  for (;;) {
    float speed = 0;
    if (PS4.isConnected()) {
      if (PS4.Triangle() && PS4.Cross()) speed = 0;
      else if (PS4.Triangle()) speed = currentSpeedAccessory;
      else if (PS4.Cross()) speed = -currentSpeedAccessory;
    }
    setMotor(FD_PWM, FD_DIR, FD_CH, speed, FD_INVERT);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- ハンド上下 ---
void handVerticalTask(void* pvParameters) {
  for (;;) {
    float speed = 0;
    if (PS4.isConnected()) {
      if (PS4.Up() && PS4.Down()) speed = 0;
      else if (PS4.Up()) speed = -currentSpeedAccessory + 60;
      else if (PS4.Down()) speed = currentSpeedAccessory - 60;
    }
    setMotor(HL_PWM, HL_DIR, HL_CH, speed);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- ハンド前後（Rスティック上下）---
void handForwardTask(void* pvParameters) {
  for (;;) {
    float speed = 0;
    if (PS4.isConnected()) {
      int ry = -PS4.RStickY();
      if (abs(ry) < DEADZONE) ry = 0;
      speed = (ry / 128.0) * currentSpeedAccessory;
    }
    setMotor(HP_PWM, HP_DIR, HP_CH, speed, HP_INVERT);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- グリッパー ---
void gripperTask(void* pvParameters) {
  for (;;) {
    float speed = 0;
    if (PS4.isConnected()) {
      if (PS4.L2() && PS4.R2()) speed = 0;
      else if (PS4.L2()) speed = currentSpeedAccessory;
      else if (PS4.R2()) speed = -currentSpeedAccessory;
    }
    setMotor(HG_PWM, HG_DIR, HG_CH, speed);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- L3ボタンで速度切替（2段階）---
void speedSwitchTask(void* pvParameters) {
  bool toggle = false;
  for (;;) {
    if (PS4.isConnected() && PS4.L3()) {
      toggle = !toggle;
      currentSpeedDrive = toggle ? BASE_SPEED_DRIVE / 2 : BASE_SPEED_DRIVE;
      Serial.printf("[Drive Speed] %s\n", toggle ? "Low" : "Normal");
      delay(500);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// --- セットアップ ---
void setup() {
  Serial.begin(115200);
  PS4.begin("1a:2b:3c:01:01:02");
  Serial.println("Ready.");

  setupMotor(FL_PWM, FL_DIR, FL_CH);
  setupMotor(FR_PWM, FR_DIR, FR_CH);
  setupMotor(RL_PWM, RL_DIR, RL_CH);
  setupMotor(RR_PWM, RR_DIR, RR_CH);
  setupMotor(FD_PWM, FD_DIR, FD_CH);
  setupMotor(HL_PWM, HL_DIR, HL_CH);
  setupMotor(HP_PWM, HP_DIR, HP_CH);
  setupMotor(HG_PWM, HG_DIR, HG_CH);

  xTaskCreate(driveTask, "Drive", 4096, NULL, 1, NULL);
  xTaskCreate(platformTask, "Platform", 2048, NULL, 1, NULL);
  xTaskCreate(handVerticalTask, "HandVert", 2048, NULL, 1, NULL);
  xTaskCreate(handForwardTask, "HandFwd", 2048, NULL, 1, NULL);
  xTaskCreate(gripperTask, "Gripper", 2048, NULL, 1, NULL);
  xTaskCreate(speedSwitchTask, "SpeedSwitch", 2048, NULL, 1, NULL);
}

void loop() {}