#include "servo.hpp"

bool ServoMotor::timersAllocated = false;

ServoMotor::ServoMotor(uint8_t pin, uint8_t minAngle, uint8_t maxAngle, int minUs, int maxUs)
    : mPin(pin), mMinUs(minUs), mMaxUs(maxUs), mMinAngle(minAngle), mMaxAngle(maxAngle) {}

void ServoMotor::begin() {
    // ESP32Servo draws its 50 Hz timing from the LEDC timer pool. Hand it all
    // four timers once - the drive motors are on MCPWM, so nothing else needs
    // them. Guarded so multiple ServoMotor instances don't re-allocate.
    if (!timersAllocated) {
        ESP32PWM::allocateTimer(0);
        ESP32PWM::allocateTimer(1);
        ESP32PWM::allocateTimer(2);
        ESP32PWM::allocateTimer(3);
        timersAllocated = true;
    }
    servo.setPeriodHertz(50); // standard hobby-servo frame rate
    servo.attach(mPin, mMinUs, mMaxUs);
}

void ServoMotor::write(int angle) {
    mLastAngle = constrain(angle, 0, 180);
    servo.write(mLastAngle);
}

void ServoMotor::open() {
    write(mMaxAngle);
}

void ServoMotor::close() {
    write(mMinAngle);
}

void ServoMotor::writeMicroseconds(int us) {
    servo.writeMicroseconds(us);
}

void ServoMotor::detach() {
    servo.detach();
}
