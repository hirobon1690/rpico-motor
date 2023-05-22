#include "motor.h"

Motor::Motor(Pwm& pwm0, Pwm& pwm1, Encoder& enc)
    : pwm0(pwm0), pwm1(pwm1), enc(enc) {
}

void Motor::init() {
    pwm0.init();
    pwm1.init();
    enc.init();
}

void Motor::write(float val) {
    target = val;
}

void Motor::setTargetPos(float target) {
    this->targetPos = target;
}

void Motor::setGainPos(float Kp, float Ki, float Kd) {
    this->Kp_pos = Kp;
    this->Ki_pos = Ki;
    this->Kd_pos = Kd;
}

void Motor::duty(float val) {
    if (val > 0) {
        pwm0.write(val);
        pwm1.write(0);
        currentDuty = pwm0.read();
    } else {
        pwm0.write(0);
        pwm1.write(-val);
        currentDuty = -pwm1.read();
    }
    
}

float Motor::getCurrentSpeed() {
    return ((enc.get() - prevEnc) / 0.01) / 15000;
}

float Motor::read() {
    return currentDuty;
}

void Motor::timer_cb() {
    int encoder = enc.get();
    float currentSpeed = ((encoder - prevEnc) / 0.01) / 7500;
    // printf("%f\n", currentSpeed);
    prevEnc = encoder;
    P = target - currentSpeed;
    I += P;
    D = (P - prevError) / 0.01;
    duty(Kp * P + Ki * I + Kd * D + prev);
    prev = Kp * P + Ki * I + Kd * D + prev;
    prevError = P;
}

void Motor::timer_cb_pos() {
    static float I;
    float current = (float)enc.get()/6300.0*360;
    printf("%f\n", current);
    float P = targetPos - current;
    I += P;
    float D = (P - prevErrorPos) / 0.01;
    write(Kp_pos * P + Ki_pos * I + Kd_pos * D);
    prevErrorPos = P;
}


void Motor::setGain(float Kp, float Ki, float Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}