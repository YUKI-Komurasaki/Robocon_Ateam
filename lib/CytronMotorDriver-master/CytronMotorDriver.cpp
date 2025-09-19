#include "CytronMotorDriver.h"

CytronMD::CytronMD(MODE mode, uint8_t pwmPin, uint8_t dirPin, uint8_t pwmChannel)
  : _mode(mode), _pwmPin(pwmPin), _dirPin(dirPin), _pwmChannel(pwmChannel)
{
  pinMode(_dirPin, OUTPUT);
  ledcSetup(_pwmChannel, 1000, 8); // 周波数1000Hz, 8bit PWM
  ledcAttachPin(_pwmPin, _pwmChannel);
}

void CytronMD::setSpeed(int16_t speed)
{
  if (_mode == PWM_DIR) {
    bool dir = speed >= 0;
    digitalWrite(_dirPin, dir ? HIGH : LOW);

    int pwm = abs(speed);
    if (pwm > 255) pwm = 255;

    ledcWrite(_pwmChannel, pwm);
  }
}
