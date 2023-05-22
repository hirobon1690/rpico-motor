#pragma once
#include <stdio.h>
#include "encoder.h"
#include "pico/stdlib.h"
#include "pwm.h"



class Motor {
   public:
    Motor(Pwm& pwm0, Pwm& pwm1, Encoder& enc);
    void init();
    void write(float val);
    void duty(float val);
    float read();
    void setGain(float Kp, float Ki, float Kd);
    void setGainPos(float Kp, float Ki, float Kd);
    void timer_cb();
    void timer_cb_pos();
    void setTargetPos(float target);
    float getCurrentSpeed();

   private:
    Pwm& pwm0;
    Pwm& pwm1;
    Encoder& enc;
    repeating_timer_t timer;
    float target, targetPos;
    float prevError, prevEnc, prevErrorPos;
    float P, I, D;
    float Kp, Ki, Kd, Kp_pos, Ki_pos, Kd_pos;
    float currentDuty;
    float prev, prevPos;
};