#include <Arduino.h>
#include <ps4controller.h>
#include <math.h>

// --- 定数 ---
const int BASE_SPEED_DRIVE = 128;
const int DASH_INCREMENT = 64;
int currentSpeedAccessory = 100;
const int DEADZONE = 30;   // 広めのデッドゾーン

// --- モーター用ピン ---
const int FL_PWM = 33, FL_DIR = 32, FL_CH = 0;
const int FR_PWM = 12, FR_DIR = 14, FR_CH = 1;
const int RL_PWM = 16, RL_DIR = 17, RL_CH = 2;
const int RR_PWM = 5,  RR_DIR = 18, RR_CH = 3;
const int FD_PWM = 26, FD_DIR = 19, FD_CH = 4;
const int HL_PWM = 4, HL_DIR = 2, HL_CH = 5;
const int HP_PWM = 21, HP_DIR = 25, HP_CH = 6;
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

// --- モーター初期化 ---
void setupMotor(int pwmPin, int dirPin, int channel){
  pinMode(dirPin, OUTPUT);
  ledcSetup(channel, 1000, 8);
  ledcAttachPin(pwmPin, channel);
  ledcWrite(channel, 0);
}

// --- モーター制御 ---
void setMotor(int pwmPin, int dirPin, int channel, float speed, bool invert=false){
  if(invert) speed = -speed;

  // PWM制限
  if(speed > 255) speed = 255;
  if(speed < -255) speed = -255;

  bool direction = speed >= 0;
  digitalWrite(dirPin, direction ? HIGH : LOW);
  ledcWrite(channel, (int)abs(speed));
}

// --- 足回りタスク ---
void driveTask(void* pvParameters){
  for(;;){
    float fl=0, fr=0, rl=0, rr=0;
    if(PS4.isConnected()){
      int x = -PS4.LStickX();
      int y = -PS4.LStickY();
      if(abs(x) < DEADZONE) x = 0;
      if(abs(y) < DEADZONE) y = 0;
      float vx = x/128.0, vy = y/128.0, vr = 0;
      if(PS4.R1()) vr = -1.0;
      else if(PS4.L1()) vr = 1.0;
      fl = (vy+vx+vr)*currentSpeedDrive;
      fr = (vy-vx-vr)*currentSpeedDrive;
      rl = (vy-vx+vr)*currentSpeedDrive;
      rr = (vy+vx-vr)*currentSpeedDrive;
    }
    setMotor(FL_PWM,FL_DIR,FL_CH,fl,FL_INVERT);
    setMotor(FR_PWM,FR_DIR,FR_CH,fr,FR_INVERT);
    setMotor(RL_PWM,RL_DIR,RL_CH,rl,RL_INVERT);
    setMotor(RR_PWM,RR_DIR,RR_CH,rr,RR_INVERT);
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// --- 土台上下タスク ---
void platformTask(void* pvParameters){
  for(;;){
    float speed = 0;
    if(PS4.isConnected()){
      if(PS4.Triangle() && PS4.Cross()) speed = 0;
      else if(PS4.Triangle()) speed = currentSpeedAccessory;
      else if(PS4.Cross()) speed = -currentSpeedAccessory;
    }
    setMotor(FD_PWM,FD_DIR,FD_CH,speed,FD_INVERT);
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// --- ハンド上下タスク ---
void handVerticalTask(void* pvParameters){
  for(;;){
    float speed = 0;
    if(PS4.isConnected()){
      if(PS4.Up() && PS4.Down()) speed = 0;
      else if(PS4.Up()) speed = -currentSpeedAccessory+60;
      else if(PS4.Down()) speed = currentSpeedAccessory-60;
    }
    setMotor(HL_PWM,HL_DIR,HL_CH,speed);
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// --- ハンド前後タスク（修正版） ---
void handForwardTask(void* pvParameters){
  for(;;){
    float speed = 0;

    if(PS4.isConnected()){
      int ry = PS4.RStickY();   // 下=+127, 上=-128
      if(abs(ry) > 20){         // デッドゾーン
        // 符号を逆にして「上=正転 / 下=逆転」
        speed = -(float)ry / 127.0f * currentSpeedAccessory;
      }
    }

    if(speed == 0){
      ledcWrite(HP_CH, 0);  
      digitalWrite(HP_DIR, LOW);  // 停止
    } else {
      setMotor(HP_PWM, HP_DIR, HP_CH, speed, HP_INVERT);
    }

    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// --- グリッパータスク ---
void gripperTask(void* pvParameters){
  for(;;){
    float speed = 0;
    if(PS4.isConnected()){
      if(PS4.L2() && PS4.R2()) speed = 0;
      else if(PS4.L2()) speed = currentSpeedAccessory/40;
      else if(PS4.R2()) speed = -currentSpeedAccessory/40;
    }
    setMotor(HG_PWM,HG_DIR,HG_CH,speed);
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// --- セットアップ ---
void setup(){
  Serial.begin(115200);
  PS4.begin("90:15:06:7c:3e:26");
  Serial.println("Ready.");

  setupMotor(FL_PWM,FL_DIR,FL_CH);
  setupMotor(FR_PWM,FR_DIR,FR_CH);
  setupMotor(RL_PWM,RL_DIR,RL_CH);
  setupMotor(RR_PWM,RR_DIR,RR_CH);
  setupMotor(FD_PWM,FD_DIR,FD_CH);
  setupMotor(HL_PWM,HL_DIR,HL_CH);
  setupMotor(HP_PWM,HP_DIR,HP_CH);
  setupMotor(HG_PWM,HG_DIR,HG_CH);

  xTaskCreate(driveTask, "Drive", 2048, NULL, 1, NULL);
  xTaskCreate(platformTask, "Platform", 2048, NULL, 1, NULL);
  xTaskCreate(handVerticalTask, "HandVert", 2048, NULL, 1, NULL);
  xTaskCreate(handForwardTask, "HandFwd", 2048, NULL, 1, NULL);
  xTaskCreate(gripperTask, "Gripper", 2048, NULL, 1, NULL);
}

void loop(){}