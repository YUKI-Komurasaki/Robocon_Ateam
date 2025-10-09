#include <Arduino.h>
#include <ps4controller.h>
#include <math.h>

const int BASE_SPEED_DRIVE = 128;
int currentSpeedAccessory = 100;
const int DEADZONE = 30;
const float DECEL_DURATION = 0.5; // 秒

// モーター用ピン
const int FL_PWM = 33, FL_DIR = 32, FL_CH = 0;
const int FR_PWM = 12, FR_DIR = 14, FR_CH = 1;
const int RL_PWM = 16, RL_DIR = 17, RL_CH = 2;
const int RR_PWM = 5,  RR_DIR = 18, RR_CH = 3;
const int FD_PWM = 27, FD_DIR = 13, FD_CH = 4;
const int HL_PWM = 4, HL_DIR = 2, HL_CH = 6;
const int HP_PWM = 21, HP_DIR = 26, HP_CH = 5;
const int HG_PWM = 23, HG_DIR = 22, HG_CH = 7;

const bool FL_INVERT = true;
const bool FR_INVERT = false;
const bool RL_INVERT = true;
const bool RR_INVERT = false;
const bool FD_INVERT = false;
const bool HP_INVERT = false;

int currentSpeedDrive = BASE_SPEED_DRIVE;
float fl_val=0, fr_val=0, rl_val=0, rr_val=0;

// モーター初期化
void setupMotor(int pwmPin, int dirPin, int channel){
  pinMode(dirPin, OUTPUT);
  ledcSetup(channel, 1000, 8);
  ledcAttachPin(pwmPin, channel);
  ledcWrite(channel,0);
  digitalWrite(dirPin,LOW);
}

// モーター制御
void setMotor(int pwmPin,int dirPin,int channel,float speed,bool invert=false){
  if(invert) speed=-speed;
  speed=constrain(speed,-255,255);
  bool direction = (speed>=0);
  digitalWrite(dirPin,direction?HIGH:LOW);
  ledcWrite(channel,(int)abs(speed));
}

// 足回りタスク
void driveTask(void* pvParameters){
  for(;;){
    float target_fl=0,target_fr=0,target_rl=0,target_rr=0;

    if(PS4.isConnected()){
      int x=-PS4.LStickX();
      int y=-PS4.LStickY();
      if(abs(x)<DEADZONE)x=0;
      if(abs(y)<DEADZONE)y=0;

      float vx=x/128.0, vy=y/128.0, vr=0;
      if(PS4.R1()) vr=-1.0;
      else if(PS4.L1()) vr=1.0;

      target_fl=(vy+vx+vr)*currentSpeedDrive;
      target_fr=(vy-vx-vr)*currentSpeedDrive;
      target_rl=(vy-vx+vr)*currentSpeedDrive;
      target_rr=(vy+vx-vr)*currentSpeedDrive;

      if(PS4.L3()) currentSpeedDrive=BASE_SPEED_DRIVE/2;
      else currentSpeedDrive=BASE_SPEED_DRIVE;
    }

    float step=255.0/(DECEL_DURATION*50); // 20ms周期
    if(target_fl>fl_val) fl_val=min(fl_val+step,target_fl);
    else fl_val=max(fl_val-step,target_fl);
    if(target_fr>fr_val) fr_val=min(fr_val+step,target_fr);
    else fr_val=max(fr_val-step,target_fr);
    if(target_rl>rl_val) rl_val=min(rl_val+step,target_rl);
    else rl_val=max(rl_val-step,target_rl);
    if(target_rr>rr_val) rr_val=min(rr_val+step,target_rr);
    else rr_val=max(rr_val-step,target_rr);

    setMotor(FL_PWM,FL_DIR,FL_CH,fl_val,FL_INVERT);
    setMotor(FR_PWM,FR_DIR,FR_CH,fr_val,FR_INVERT);
    setMotor(RL_PWM,RL_DIR,RL_CH,rl_val,RL_INVERT);
    setMotor(RR_PWM,RR_DIR,RR_CH,rr_val,RR_INVERT);
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// 土台上下
void platformTask(void* pvParameters){
  for(;;){
    float speed=0;
    if(PS4.isConnected()){
      if(PS4.Triangle() && PS4.Cross()) speed=0;
      else if(PS4.Triangle()) speed=currentSpeedAccessory;
      else if(PS4.Cross()) speed=-currentSpeedAccessory;
    }
    setMotor(FD_PWM,FD_DIR,FD_CH,speed,FD_INVERT);
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// ハンド上下
void handVerticalTask(void* pvParameters){
  for(;;){
    float speed=0;
    if(PS4.isConnected()){
      if(PS4.Up() && PS4.Down()) speed=0;
      else if(PS4.Up()) speed=-currentSpeedAccessory+60;
      else if(PS4.Down()) speed=currentSpeedAccessory-60;
    }
    setMotor(HL_PWM,HL_DIR,HL_CH,speed);
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// ハンド前後：右スティックY
void handForwardTask(void* pvParameters){
  for(;;){
    float speed=0;
    if(PS4.isConnected()){
      int y=-PS4.RStickY();
      if(abs(y)<DEADZONE) y=0;
      speed=(y/128.0)*currentSpeedAccessory;
    }
    if(speed==0){
      ledcWrite(HP_CH,0);
      digitalWrite(HP_DIR,LOW);
    }else{
      setMotor(HP_PWM,HP_DIR,HP_CH,speed,HP_INVERT);
    }
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

// グリッパー
void gripperTask(void* pvParameters){
  for(;;){
    float speed=0;
    if(PS4.isConnected()){
      if(PS4.L2() && PS4.R2()) speed=0;
      else if(PS4.L2()) speed=currentSpeedAccessory;
      else if(PS4.R2()) speed=-currentSpeedAccessory;
    }
    setMotor(HG_PWM,HG_DIR,HG_CH,speed);
    vTaskDelay(20/portTICK_PERIOD_MS);
  }
}

void setup(){
  Serial.begin(115200);
  PS4.begin("1a:2b:3c:01:01:02");
  setupMotor(FL_PWM,FL_DIR,FL_CH);
  setupMotor(FR_PWM,FR_DIR,FR_CH);
  setupMotor(RL_PWM,RL_DIR,RL_CH);
  setupMotor(RR_PWM,RR_DIR,RR_CH);
  setupMotor(FD_PWM,FD_DIR,FD_CH);
  setupMotor(HL_PWM,HL_DIR,HL_CH);
  setupMotor(HP_PWM,HP_DIR,HP_CH);
  setupMotor(HG_PWM,HG_DIR,HG_CH);

  xTaskCreate(driveTask,"Drive",2048,NULL,1,NULL);
  xTaskCreate(platformTask,"Platform",2048,NULL,1,NULL);
  xTaskCreate(handVerticalTask,"HandVert",2048,NULL,1,NULL);
  xTaskCreate(handForwardTask,"HandFwd",2048,NULL,1,NULL);
  xTaskCreate(gripperTask,"Gripper",2048,NULL,1,NULL);
}

void loop(){}