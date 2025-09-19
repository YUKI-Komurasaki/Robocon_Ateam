#ifndef CYTRON_MOTOR_DRIVER_H
#define CYTRON_MOTOR_DRIVER_H

#include <Arduino.h>

enum MODE {
  PWM_DIR,
  PWM_PWM  // 今回は使いませんが定義だけ残しておきます
};

class CytronMD {
  public:
    CytronMD(MODE mode, uint8_t pwmPin, uint8_t dirPin, uint8_t pwmChannel);
    void setSpeed(int16_t speed);

  private:
    MODE _mode;
    uint8_t _pwmPin, _dirPin, _pwmChannel;
};

#endif
