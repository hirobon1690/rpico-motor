#include "motor.h"
#define PI 3.14159265358979323846

Motor::Motor(Gpio& dir0, Pwm& pwm0, Encoder& enc, Gpio& cs)
    : dir0(dir0), pwm0(pwm0), enc(enc), cs(cs) {
    velpid.setGain(1, 0.0, 0.0);
    velpid.setDt(0.01);
    pospid.setGain(0.5, 0.0, 0.0);
    pospid.setDt(0.1);
    // pospid.setGain(2.5, 0.08, 0.12);
}

void Motor::init(bool isReverse) {
    dir0.init();
    pwm0.init();
    enc.init();
    cs.init();
    uint8_t tx_data[2] = {0x08, 0x80};
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    cs.write(0);
    spi_write_read_blocking(spi0, tx_data, nullptr, 2);
    cs.write(1);
    uint8_t config_data[2] = {0x0a, 0b01111111};
    cs.write(0);
    spi_write_read_blocking(spi0, config_data, nullptr, 2);
    cs.write(1);
    prevEnc = enc.get();
    this->isReverse = isReverse;
}

void Motor::setVel(float val) {
    if (val == 0) {
        duty(0);
    } else if (val > MAX_SPEED) {
        val = MAX_SPEED;
    } else if (val < -MAX_SPEED) {
        val = -MAX_SPEED;
    }
    velpid.setGoal(val * 2);
}

void Motor::setPos(float target) {
    isPosPidEnabled = true;
    pospid.setGoal(target);
}

void Motor::setVelGain(float Kp, float Ki, float Kd) {
    velpid.setGain(Kp, Ki, Kd);
}
void Motor::setPosGain(float Kp, float Ki, float Kd) {
    pospid.setGain(Kp, Ki, Kd);
}

void Motor::duty(float val) {
    if (val >= 0) {
        pwm0.write(val);
        dir0.write(!isReverse);
        currentDuty = pwm0.read();
    } else {
        pwm0.write(-val);
        dir0.write(isReverse);
        currentDuty = -pwm0.read();
    }
}

// deg/s
float Motor::getCurrentSpeed() {
    int currentEnc = enc.get();
    float speed = ((((float)currentEnc - (float)prevEnc) * 360.0) / 4096.0) / 0.01;
    prevEnc = currentEnc;
    for (int i = 0; i < 3; i++) {
        speeds[i + 1] = speeds[i];
    }
    speeds[0] = speed;
    float sum = 0;
    for (int i = 0; i < 1; i++) {
        sum += speeds[i];
    }
    return sum / 1;
}

float Motor::read() {
    return currentDuty;
}

void Motor::timer_cb() {
    float speed = getCurrentSpeed();
    velpid.update(speed);
    // printf("%f\n", speed);
    float a = velpid.calc();
    // printf("%f, %f\n", currentDuty, speed);
    duty(currentDuty + a / 10000);
}

void Motor::timer_cb_pos() {
    float pos = enc.get() * 180.0 / 4096.0;
    if (!isPosPidEnabled) {
        return;
    }
    pospid.update(pos);
    float a = pospid.calc();
    printf("%f, %f\n", pos, a);
    // if (std::abs(a) > 1) {
        setVel(a);
    // } else {
        // setVel(0);
        // duty(0);
    // }
}

void Motor::disablePosPid() {
    isPosPidEnabled = false;
}

void Motor::resetPos() {
    // setPos(0);
    disablePosPid();
    enc.set(0);
    currentPos = 0;
    prevPos = 0;
    prevEnc = 0;
}

void Motor::setMaxSpeed(float maxSpeed) {
    MAX_SPEED = maxSpeed;
}