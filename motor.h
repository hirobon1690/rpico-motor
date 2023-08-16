#pragma once
#include <hardware/spi.h>
#include <math.h>
#include <stdio.h>
#include <queue>
#include "encoderBase.h"
#include "gpio.h"
#include "pico/stdlib.h"
#include "pid.h"
#include "pwm.h"

class Motor {
   public:
    Motor(Gpio& dir0, Pwm& Pwm0, Encoder& enc, Gpio& cs);
    void init();
    void setVel(float val);
    void setPos(float target);
    void duty(float val);
    float read();
    void setVelGain(float Kp, float Ki, float Kd);
    void setPosGain(float Kp, float Ki, float Kd);
    void timer_cb();
    void timer_cb_pos();
    float getCurrentSpeed();
    void disablePosPid();
    void resetPos();
    float currentDuty, currentPos;

   private:
    Gpio& dir0;
    Gpio& cs;
    Pwm& pwm0;
    Encoder& enc;
    repeating_timer_t timer;
    Pid velpid, pospid;
    int prevEnc, prevPos;
    float speeds[10];
    bool isPosPidEnabled;
    const int MAX_SPEED = 540;
};