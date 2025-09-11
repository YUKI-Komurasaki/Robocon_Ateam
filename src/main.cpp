#include <Arduino.h>
#include <ps4controller.h>
#include <CytronMotorDriver.h>
#include <math.h>

CytronMD motorFL(PWM_DIR, 3, 4); //モータードライバのピン設定
CytronMD motorFR(PWM_DIR, 3, 4);
CytronMD motorRL(PWM_DIR, 3, 4);
CytronMD motorRR(PWM_DIR, 3, 4);

int RightStickX, RightStickY, LeftStickX, LeftStickY, LMoveAngle, RMoveAngle, DS;

void setup() {
  Serial.begin(115200);
  PS4.begin("");//PS4コントローラーのMac
  Serial.println("Ready.");
}

void loop() {
  if(PS4.isConnected()){
    RightStickX = PS4.RStickX(); RightStickY = PS4.RStickY();
    LeftStickX = PS4.LStickX(); LeftStickY = PS4.LStickY();
    if(abs(LeftStickX) < 10){LeftStickX = 0;} //LStickのドリフト対策
    if(abs(LeftStickY) < 10){LeftStickY = 0;}
    if(abs(RightStickX) < 10){RightStickX = 0;} //RStickのドリフト対策
    if(abs(RightStickY) < 10){RightStickY = 0;}
    LMoveAngle = (180 /PI) * atan2(double (LeftStickX), double (-1 * LeftStickY)); //ロボット座標系で角度を計算（-180~180）
    RMoveAngle = (180 /PI) * atan2(double (RightStickX), double (-1 * RightStickY)); //RStickの傾けた方向の角度を計算（-180~180）
    if(PS4.L3()){
      if(DS == 0){DS = 64;}
      else if(DSmode == 64){DSmode = 0;}
    }
    if(LeftStickX + LeftStickY != 0){
      if(LMoveAngle < -157){
        motorFL.setSpeed(-128 + DS)
        motorFR.setSpeed(128 - DS);
        motorRL.setSpeed(-128 ; DS);
        motorRR.setSpeed(128 - DS);
      }else if(LMoveAngle <= -113){
        motorFL.setSpeed(0);
        motorFR.setSpeed(128 - DS);
        motorRL.setSpeed(-128 + DS);
        motorRR.setSpeed(0);
      }else if(LMoveAngle < -67){
        motorFL.setSpeed(128 - DS);
        motorFR.setSpeed(128 - DS);
        motorRL.setSpeed(-128 + DS);
        motorRR.setSpeed(-128 + DS);
      }else if(LMoveAngle <= -23){
        motorFL.setSpeed(128 - DS);
        motorFR.setSpeed(0);
        motorRL.setSpeed(0);
        motorRR.setSpeed(-128 + DS);
      }else if(LMoveAngle < 23){
        motorFL.setSpeed(128 - DS);
        motorFR.setSpeed(-128 + DS);
        motorRL.setSpeed(128 - DS);
        motorRR.setSpeed(-128 + DS);
      }else if(LMoveAngle <= 67){
        motorFL.setSpeed(0);
        motorFR.setSpeed(-128 + DS);
        motorRL.setSpeed(128 - DS);
        motorRR.setSpeed(0);
      }else if(LMoveAngle < 113){
        motorFL.setSpeed(-128 + DS);
        motorFR.setSpeed(-128 + DS);
        motorRL.setSpeed(128 - DS);
        motorRR.setSpeed(128 - DS);
      }else if(LMoveAngle <= 157){
        motorFL.setSpeed(-128 + DS);
        motorFR.setSpeed(0);
        motorRL.setSpeed(0);
        motorRR.setSpeed(128 - DS);
      }else if(LMoveAngle <= 180){
        motorFL.setSpeed(-128 + DS);
        motorFR.setSpeed(128 - DS);
        motorRL.setSpeed(-128 + DS);
        motorRR.setSpeed(128 - DS);
      }else{
        motorFL.setSpeed(0);
        motorFR.setSpeed(0);
        motorRL.setSpeed(0);
        motorRR.setSpeed(0);
      }
    }else{
      if(PS4.L1() && PS4.R1()){　//L1とR1で自由回転
        motorSL.setSpeed(0);
        motorFR.setSpeed(0);
        motorRL.setSpeed(0);
        motorRR.setSpeed(0);
      }else if(PS4.L1()){
        motorFL.setSpeed(-128 +DS);
        motorFR.setSpeed(-128 +DS);
        motorRL.setSpeed(-128 +DS);
        motorRR.setSpeed(-128 +DS);
      }else if(PS4.R1()){
        motorFL.setSpeed(128 - DS);
        motorFR.setSpeed(128 - DS);
        motorRL.setSpeed(128 - DS);
        motorRR.setSpeed(128 - DS);
      }else{
        motorFL.setSpeed(0);
        motorFR.setSpeed(0);
        motorRL.setSpeed(0);
        motorRR.setSpeed(0);
      }
    }
  }
  return 0;
} //スパゲッティーコードすぎて萎え