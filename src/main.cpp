#include <Arduino.h>
#include <ps4controller.h>
#include <CytronMotorDriver.h>
#include <math.h>

CytronMD motorFL(PWM_DIR, 3, 4); //モータードライバのピン設定
CytronMD motorFR(PWM_DIR, 3, 4);
CytronMD motorRL(PWM_DIR, 3, 4);
CytronMD motorRR(PWM_DIR, 3, 4);
CytronMD D1(PWM_DIR,3,4);
CytronMD D2(PWM_DIR,3,4);
CytronMD D3(PWM_DIR,3,4);
CytronMD D4(PWM_DIR,3,4);
CytronMD D5(PWM_DIR,3,4);

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
    if(PS4.L3()){
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

    // ○ボタンが押されたとき (右回転)
    if (PS4.Circle()){
      Serial.println("Circle button pressed - Forward");
      int turn_speed=100;
      D1.setSpeed(turn_speed);
      D2.setSpeed(turn_speed);
      Serial.println("右回転");
    }
    // □ボタンが押されたとき (左回転)
    if (PS4.Square()) {
      Serial.println("Square button pressed - Reverse");
      int turn_speed=100;
      D1.setSpeed(-turn_speed);
      D2.setSpeed(-turn_speed);
      Serial.println("左回転");
    }
    // ---  十字ボタンの↑と↓で制御 （ハンドの上昇機構）---
    if (PS4.Up()){
      Serial.println("up button pressed - Forward");
      int turn_speed=100;
      D3.setSpeed(turn_speed);
      Serial.println("右回転");
    }
    if (PS4.Down()) {
      Serial.println("down button pressed - Reverse");
      int turn_speed=100;
      D3.setSpeed(-turn_speed);
      Serial.println("左回転");
    }
    // --- 十字ボタンの→と←で制御 （前後移動）---
    if (PS4.Right()){
      Serial.println("Right Button pressed - Forward");
      int turn_speed=100;
      D4.setSpeed(turn_speed);
      Serial.println("右回転");
    }
    if (PS4.Left()){
      Serial.println("Left Button pressed - Reverse");
      int turn_speed=100;
      D4.setSpeed(-turn_speed);
      Serial.println("左回転");
    } 
    //ハンド：L1で右回転、R1で左回転
    if (PS4.L1()){
      Serial.println("L1 Button pressed - Forward");
      int turn_speed=10;
      D5.setSpeed(turn_speed);
      Serial.println("右回転");
    } 
    if (PS4.R1()) {
      Serial.println("R1 Button pressed - Reverse");
      int turn_speed=10;
      D5.setSpeed(-turn_speed);
      Serial.println("左回転");
    }
    // いずれかのボタンが離されたとき
    else if (Ps4.event.button_up.circle || Ps4.event.button_up.square) {
      Serial.println("Button released - Stop");
            
      // モーターを停止
      analogWrite(MOTOR1_PWM, 0);
      analogWrite(MOTOR2_PWM, 0);
    }
  
  }
  delay(50);
  return 0;
} //スパゲッティーコードすぎて萎え