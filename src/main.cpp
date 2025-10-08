#include <Arduino.h>
#include <ps4controller.h>
#include <math.h>

// --- 定数 ---
const int BASE_SPEED_DRIVE = 128;
int currentSpeedAccessory = 100;
const int DEADZONE = 30;   // 広めのデッドゾーン

// --- モーター用ピン ---
// 足回り
const int FL_PWM = 33, FL_DIR = 32, FL_CH = 0;
const int FR_PWM = 12, FR_DIR = 14, FR_CH = 1;
const int RL_PWM = 16, RL_DIR = 17, RL_CH = 2;
const int RR_PWM = 5,  RR_DIR = 18, RR_CH = 3;

// 土台（Cytron MD13S）
const int FD_PWM = 27;
const int FD_DIR = 13;
const int FD_CH  = 4;

// ハンド上下（Up / Downボタン）
const int HL_PWM = 4, HL_DIR = 2, HL_CH = 6;
// ハンド前後（Rスティック上下）
const int HP_PWM = 21, HP_DIR = 26, HP_CH = 5;
// グリッパー（L2 / R2）
const int HG_PWM = 23, HG_DIR = 22, HG_CH = 7;

// --- モーター反転フラグ ---
const bool FL_INVERT = true;
const bool FR_INVERT = false;
const bool RL_INVERT = true;
const bool RR_INVERT = false;
const bool FD_INVERT = false;  
const bool HP_INVERT = false;

// --- 足回りスピード ---
int currentSpeedDrive = BASE_SPEED_DRIVE;

// --- 速度切替用 ---
const int DRIVE_SPEED_NORMAL = 64;
const int DRIVE_SPEED_HIGH   = 128;
bool isHighSpeedMode = false;

// --- モーター初期化 ---
void setupMotor(int pwmPin, int dirPin, int channel) {
  pinMode(dirPin, OUTPUT);
  ledcSetup(channel, 1000, 8);
  ledcAttachPin(pwmPin, channel);
  ledcWrite(channel, 0);
  digitalWrite(dirPin, LOW); // 初期状態で安全
}

// --- モーター制御 ---
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

      fl = (vy + vx + vr) * currentSpeedDrive;
      fr = (vy - vx - vr) * currentSpeedDrive;
      rl = (vy - vx + vr) * currentSpeedDrive;
      rr = (vy + vx - vr) * currentSpeedDrive;
    } else {
      fl = fr = rl = rr = 0;  // PS4切断時は停止
    }

    setMotor(FL_PWM, FL_DIR, FL_CH, fl, FL_INVERT);
    setMotor(FR_PWM, FR_DIR, FR_CH, fr, FR_INVERT);
    setMotor(RL_PWM, RL_DIR, RL_CH, rl, RL_INVERT);
    setMotor(RR_PWM, RR_DIR, RR_CH, rr, RR_INVERT);

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- 土台上下タスク ---
void platformTask(void* pvParameters) {
  for (;;) {
    float speed = 0;

    if (PS4.isConnected()) {
      if (PS4.Triangle() && PS4.Cross()) speed = 0;
      else if (PS4.Triangle()) speed = currentSpeedAccessory;
      else if (PS4.Cross()) speed = -currentSpeedAccessory;
      else speed = 0;
    } else {
      speed = 0;
    }

    setMotor(FD_PWM, FD_DIR, FD_CH, speed, FD_INVERT);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- ハンド上下タスク ---
void handVerticalTask(void* pvParameters) {
  for (;;) {
    float speed = 0;
    if (PS4.isConnected()) {
      if (PS4.Up() && PS4.Down()) speed = 0;
      else if (PS4.Up()) speed = -currentSpeedAccessory + 60;
      else if (PS4.Down()) speed = currentSpeedAccessory - 60;
    } else {
      speed = 0;
    }
    setMotor(HL_PWM, HL_DIR, HL_CH, speed);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- ハンド前後タスク（Rスティック上下） ---
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

// --- グリッパータスク ---
void gripperTask(void* pvParameters) {
  const float GRIPPER_SPEED = currentSpeedAccessory;
  for (;;) {
    float speed = 0;
    if (PS4.isConnected()) {
      if (PS4.L2() && PS4.R2()) speed = 0;
      else if (PS4.L2()) speed = GRIPPER_SPEED;
      else if (PS4.R2()) speed = -GRIPPER_SPEED;
    } else {
      speed = 0;
    }
    setMotor(HG_PWM, HG_DIR, HG_CH, speed);
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

// --- L3ボタンで速度切替タスク ---
void speedModeTask(void* pvParameters) {
  bool prevL3 = false;

  for (;;) {
    if (PS4.isConnected()) {
      bool nowL3 = PS4.L3();

      if (nowL3 && !prevL3) {
        isHighSpeedMode = !isHighSpeedMode;
        currentSpeedDrive = isHighSpeedMode ? DRIVE_SPEED_HIGH : DRIVE_SPEED_NORMAL;
        Serial.printf("[MODE] Drive speed: %s\n", 
                      isHighSpeedMode ? "HIGH" : "NORMAL");
      }
      prevL3 = nowL3;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// --- セットアップ ---
void setup() {
  Serial.begin(115200);
  PS4.begin("1a:2b:3c:01:01:02"); 
  Serial.println("Ready.");

  // モーター初期化
  setupMotor(FL_PWM, FL_DIR, FL_CH);
  setupMotor(FR_PWM, FR_DIR, FR_CH);
  setupMotor(RL_PWM, RL_DIR, RL_CH);
  setupMotor(RR_PWM, RR_DIR, RR_CH);
  setupMotor(FD_PWM, FD_DIR, FD_CH);
  setupMotor(HL_PWM, HL_DIR, HL_CH);
  setupMotor(HP_PWM, HP_DIR, HP_CH);
  setupMotor(HG_PWM, HG_DIR, HG_CH);

  // タスク起動
  xTaskCreate(driveTask, "Drive", 2048, NULL, 1, NULL);
  xTaskCreate(platformTask, "Platform", 2048, NULL, 1, NULL);
  xTaskCreate(handVerticalTask, "HandVert", 2048, NULL, 1, NULL);
  xTaskCreate(handForwardTask, "HandFwd", 2048, NULL, 1, NULL);
  xTaskCreate(gripperTask, "Gripper", 2048, NULL, 1, NULL);
  xTaskCreate(speedModeTask, "SpeedMode", 2048, NULL, 1, NULL);
}

void loop() {}