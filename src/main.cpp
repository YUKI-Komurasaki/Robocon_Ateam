#include <Arduino.h>
#include <ps4controller.h>
#include <math.h>

// --- 定数定義 ---
const int BASE_SPEED_DRIVE = 128;
const int DASH_INCREMENT = 64;
const int BASE_SPEED_ACCESSORY = 100;
const int DEADZONE = 15;
const float SMOOTHING = 0.2;

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
const int HP_PWM = 21, HP_DIR = 25, HP_CH = 6;
// グリッパー
const int HG_PWM = 23, HG_DIR = 22, HG_CH = 7;

// --- モーター反転フラグ ---
const bool FL_INVERT = true;
const bool FR_INVERT = false;
const bool RL_INVERT = true;
const bool RR_INVERT = false;
const bool FD_INVERT = false; // △で上昇、×で下降

// --- 変数 ---
int currentSpeedDrive = BASE_SPEED_DRIVE;
const int currentSpeedAccessory = BASE_SPEED_ACCESSORY;
bool l3_was_pressed = false;

// 足回りスムージング
float targetFL=0, currentFL=0, targetFR=0, currentFR=0;
float targetRL=0, currentRL=0, targetRR=0, currentRR=0;

// --- モーター制御 ---
void setMotor(int pwmPin, int dirPin, int channel, float speed, bool invert=false){
  if(invert) speed=-speed;
  bool direction = speed>=0;
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, direction ? HIGH : LOW);
  int pwm_val=(int)abs(speed);
  if(pwm_val>255)pwm_val=255;
  if(pwm_val<10)pwm_val=0;
  ledcWrite(channel,pwm_val);
}

void setupMotor(int pwmPin,int dirPin,int channel){
  pinMode(dirPin,OUTPUT);
  ledcSetup(channel,1000,8);
  ledcAttachPin(pwmPin,channel);
}

void stopAllMotors(){
  setMotor(FL_PWM, FL_DIR, FL_CH,0,FL_INVERT);
  setMotor(FR_PWM, FR_DIR, FR_CH,0,FR_INVERT);
  setMotor(RL_PWM, RL_DIR, RL_CH,0,RL_INVERT);
  setMotor(RR_PWM, RR_DIR, RR_CH,0,RR_INVERT);
  setMotor(FD_PWM, FD_DIR, FD_CH,0,FD_INVERT);
  setMotor(HL_PWM, HL_DIR, HL_CH,0);
  setMotor(HP_PWM, HP_DIR, HP_CH,0);
  setMotor(HG_PWM, HG_DIR, HG_CH,0);
}

// --- スムージング ---
float smoothSpeed(float current,float target){
  return current+(target-current)*SMOOTHING;
}

// --- 足回り制御 ---
void controlDrive(){
  int x=-PS4.LStickX();
  int y=-PS4.LStickY();
  if(abs(x)<DEADZONE)x=0;
  if(abs(y)<DEADZONE)y=0;
  float vx=x/128.0, vy=y/128.0, vr=0;
  if(PS4.R1()) vr=-1.0;
  else if(PS4.L1()) vr=1.0;
  float fl=(vy+vx+vr)*currentSpeedDrive;
  float fr=(vy-vx-vr)*currentSpeedDrive;
  float rl=(vy-vx+vr)*currentSpeedDrive;
  float rr=(vy+vx-vr)*currentSpeedDrive;
  targetFL=fl; targetFR=fr; targetRL=rl; targetRR=rr;
  currentFL=smoothSpeed(currentFL,targetFL);
  currentFR=smoothSpeed(currentFR,targetFR);
  currentRL=smoothSpeed(currentRL,targetRL);
  currentRR=smoothSpeed(currentRR,targetRR);
  setMotor(FL_PWM,FL_DIR,FL_CH,currentFL,FL_INVERT);
  setMotor(FR_PWM,FR_DIR,FR_CH,currentFR,FR_INVERT);
  setMotor(RL_PWM,RL_DIR,RL_CH,currentRL,RL_INVERT);
  setMotor(RR_PWM,RR_DIR,RR_CH,currentRR,RR_INVERT);
}

// --- 土台上下制御 ---
void controlPlatform(){
  if(PS4.Triangle()&&PS4.Cross()) setMotor(FD_PWM,FD_DIR,FD_CH,0,FD_INVERT);
  else if(PS4.Triangle()) setMotor(FD_PWM,FD_DIR,FD_CH,currentSpeedAccessory,FD_INVERT);
  else if(PS4.Cross()) setMotor(FD_PWM,FD_DIR,FD_CH,-currentSpeedAccessory,FD_INVERT);
  else setMotor(FD_PWM,FD_DIR,FD_CH,0,FD_INVERT);
}

// --- ハンド上下 ---
void controlHandVertical(){
  if(PS4.Up()&&PS4.Down()) setMotor(HL_PWM,HL_DIR,HL_CH,0);
  else if(PS4.Up()) setMotor(HL_PWM,HL_DIR,HL_CH,currentSpeedAccessory-60);
  else if(PS4.Down()) setMotor(HL_PWM,HL_DIR,HL_CH,-currentSpeedAccessory+60);
  else setMotor(HL_PWM,HL_DIR,HL_CH,0);
}

// --- ハンド前後 ---
void controlHandForward(){
  int ry=-PS4.RStickY();
  if(abs(ry)<DEADZONE) setMotor(HP_PWM,HP_DIR,HP_CH,0);
  else setMotor(HP_PWM,HP_DIR,HP_CH,(ry/128.0)*currentSpeedAccessory);
}

// --- グリッパー（ゆっくり小刻み操作） ---
void controlGripper(){
  if(PS4.L2() && PS4.R2()) setMotor(HG_PWM, HG_DIR, HG_CH, 0);
  else if(PS4.L2()) setMotor(HG_PWM, HG_DIR, HG_CH, currentSpeedAccessory / 40);   // 閉じる
  else if(PS4.R2()) setMotor(HG_PWM, HG_DIR, HG_CH, -currentSpeedAccessory / 40);  // 開く
  else setMotor(HG_PWM, HG_DIR, HG_CH, 0);
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
  stopAllMotors();
}

// --- メインループ ---
void loop(){
  if(PS4.isConnected()){
    if(PS4.L3() && !l3_was_pressed){
      currentSpeedDrive=(currentSpeedDrive==BASE_SPEED_DRIVE) ? (BASE_SPEED_DRIVE+DASH_INCREMENT) : BASE_SPEED_DRIVE;
    }
    l3_was_pressed=PS4.L3();
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
  } else stopAllMotors();
  delay(20);
}