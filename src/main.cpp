#include <Arduino.h>
#include <ps4controller.h>
#include <CytronMotorDriver.h>
#include <math.h>

CytronMD motorFL(PWM_DIR, 33, 32); //モータードライバのピン設定
CytronMD motorFR(PWM_DIR, 35, 34);
CytronMD motorRL(PWM_DIR, 16, 17);
CytronMD motorRR(PWM_DIR, 5, 18);
CytronMD motorFD(PWM_DIR,3,4);
CytronMD motorHL(PWM_DIR,3,4);
CytronMD motorHP(PWM_DIR,3,4);
CytronMD motorHG(PWM_DIR,3,4);

int LeftStickX, LeftStickY, LMoveAngle, DS;

void setup() {
  Serial.begin(115200);
  PS4.begin("");//PS4コントローラーのMac
  Serial.println("Ready.");
}

void loop() {
  Ps4.update(); // Ps4.update() を呼び出すことで、コントローラーの状態を更新します

  if(PS4.isConnected()){

    LeftStickX = PS4.LStickX(); LeftStickY = PS4.LStickY();
    if(abs(LeftStickX) < 10){LeftStickX = 0;} //LStickのドリフト対策
    if(abs(LeftStickY) < 10){LeftStickY = 0;}
    LMoveAngle = (180 /PI) * atan2(double (LeftStickX), double (-1 * LeftStickY)); //ロボット座標系で角度を計算（-180~180）
    if(PS4.L3()){ //L3ボタンが押されたとき（ダッシュモード）
      if(DS == 0){DS = 64;}
      else{DSmode = 0;}
    }

    if(LeftStickX + LeftStickY != 0){ //閾値を付け8方向に平行移動
      if(LMoveAngle < -157){ //前進
        motorFL.setSpeed(-128 + DS)
        motorFR.setSpeed(128 - DS);
        motorRL.setSpeed(-128 ; DS);
        motorRR.setSpeed(128 - DS);
      }else if(LMoveAngle <= -113){ //左前進
        motorFL.setSpeed(0);
        motorFR.setSpeed(128 - DS);
        motorRL.setSpeed(-128 + DS);
        motorRR.setSpeed(0);
      }else if(LMoveAngle < -67){ //左進
        motorFL.setSpeed(128 - DS);
        motorFR.setSpeed(128 - DS);
        motorRL.setSpeed(-128 + DS);
        motorRR.setSpeed(-128 + DS);
      }else if(LMoveAngle <= -23){ //左後進
        motorFL.setSpeed(128 - DS);
        motorFR.setSpeed(0);
        motorRL.setSpeed(0);
        motorRR.setSpeed(-128 + DS);
      }else if(LMoveAngle < 23){ //後進
        motorFL.setSpeed(128 - DS);
        motorFR.setSpeed(-128 + DS);
        motorRL.setSpeed(128 - DS);
        motorRR.setSpeed(-128 + DS);
      }else if(LMoveAngle <= 67){ //右後進
        motorFL.setSpeed(0);
        motorFR.setSpeed(-128 + DS);
        motorRL.setSpeed(128 - DS);
        motorRR.setSpeed(0);
      }else if(LMoveAngle < 113){ //右進
        motorFL.setSpeed(-128 + DS);
        motorFR.setSpeed(-128 + DS);
        motorRL.setSpeed(128 - DS);
        motorRR.setSpeed(128 - DS);
      }else if(LMoveAngle <= 157){ //右前進
        motorFL.setSpeed(-128 + DS);
        motorFR.setSpeed(0);
        motorRL.setSpeed(0);
        motorRR.setSpeed(128 - DS);
      }else if(LMoveAngle <= 180){ //前進
        motorFL.setSpeed(-128 + DS);
        motorFR.setSpeed(128 - DS);
        motorRL.setSpeed(-128 + DS);
        motorRR.setSpeed(128 - DS);
      }else{ //バグ対策
        motorFL.setSpeed(0);
        motorFR.setSpeed(0);
        motorRL.setSpeed(0);
        motorRR.setSpeed(0);
      }
    }else{
      //L1とR1で自由回転
      if(PS4.L1() && PS4.R1()){ //L1とR1が同時押しされたとき (停止)
        motorSL.setSpeed(0);
        motorFR.setSpeed(0);
        motorRL.setSpeed(0);
        motorRR.setSpeed(0);
      }else if(PS4.L1()){ //L1が押されたとき (左回転)
        motorFL.setSpeed(-128 +DS);
        motorFR.setSpeed(-128 +DS);
        motorRL.setSpeed(-128 +DS);
        motorRR.setSpeed(-128 +DS);
      }else if(PS4.R1()){ //R1が押されたとき (右回転)
        motorFL.setSpeed(128 - DS);
        motorFR.setSpeed(128 - DS);
        motorRL.setSpeed(128 - DS);
        motorRR.setSpeed(128 - DS);
      }else{ // いずれのボタンも押されていないとき
        motorFL.setSpeed(0);
        motorFR.setSpeed(0);
        motorRL.setSpeed(0);
        motorRR.setSpeed(0);
      }
    }

    if(PS4.Triangle() && Ps4.cross()) { //△と×が同時押しされたとき (土台停止)
      motorFD.setSpeed(0);
      Serial.println("土台停止");
    }else if (PS4.triangle()){ //△ボタンが押されたとき (土台上昇)
      int turn_speed=100;
      motorFD.setSpeed(turn_speed);
      Serial.println("土台上昇");
    }else if (PS4.cross()) { //×ボタンが押されたとき (土台下降)
      int turn_speed=100;
      motorFD.setSpeed(-turn_speed);
      Serial.println("土台下降");
    }else{ // いずれのボタンも押されていないとき
      motorFD.setSpeed(0);
    }

    if(PS4.Up() && PS4.Down()) { //↑と↓が同時押しされたとき (ハンド停止)
      motorHL.setSpeed(0);
      Serial.println("ハンド停止");
    }else if (PS4.Up()){ //↑ボタンが押されたとき (ハンド上昇)
      int turn_speed=100;
      motorHL.setSpeed(turn_speed);
      Serial.println("ハンド上昇");
    }else if (PS4.Down()) { //↓ボタンが押されたとき (ハンド下降)
      int turn_speed=100;
      motorHL.setSpeed(-turn_speed);
      Serial.println("ハンド下降");
    }else{ // いずれのボタンも押されていないとき
      motorHL.setSpeed(0);
    }

    if(PS4.Right() && PS4.Left()) { //→と←が同時押しされたとき (土台停止)
      motorHP.setSpeed(0);
      Serial.println("土台停止");
    }else if (PS4.Right()){ //→ボタンが押されたとき (土台前進)
      int turn_speed=100;
      motorHP.setSpeed(turn_speed);
      Serial.println("土台前進");
    }else if (PS4.Left()){ //←ボタンが押されたとき (土台後退)
      int turn_speed=100;
      motorHP.setSpeed(-turn_speed);
      Serial.println("土台後退");
    } else{ // いずれのボタンも押されていないとき
      motorHP.setSpeed(0);
    }

    if(PS4.L2() && PS4.R2()) { //L2とR2が同時押しされたとき (ハンド停止)
      motorHG.setSpeed(0);
      Serial.println("ハンド停止");
    }else if (PS4.L2()){ //L2ボタンが押されたとき (ハンド閉)
      int turn_speed=10;
      motorHG.setSpeed(turn_speed);
      Serial.println("ハンド閉");
    }else if (PS4.R2()) { //R2ボタンが押されたとき (ハンド開)
      int turn_speed=10;
      motorHG.setSpeed(-turn_speed);
      Serial.println("ハンド開");
    }else{ // いずれのボタンも押されていないとき
      motorHG.setSpeed(0);
    }
  }
  delay(50);
  return 0;
} //スパゲッティーコードすぎて萎え