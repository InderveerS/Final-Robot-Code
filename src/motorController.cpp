#include "motorController.hpp"


MotorController::MotorController(uint8_t mPin1, uint8_t mPin2, bool mInverted, 
    uint8_t mEncPin1, uint8_t mEncPin2, pcnt_unit_t mUnit, float kp, float ki, float kd, float dt, float alpha) : 
    motor(mPin1, mPin2, mInverted), pid(kp, ki, kd, dt, mOutMin, mOutMax, alpha), 
    encoder(mEncPin1, mEncPin2, mUnit, mCountsPerRev, mWheelCircumferenceM) 
{
    this->mPin1 = mPin1;
    this->mPin2 = mPin2;
    this->mInverted = mInverted;

    this->mEncPin1 = mEncPin1;
    this->mEncPin2 = mEncPin2;
    this->mUnit = mUnit;

    this->mKp = kp;
    this->mKi = ki;
    this->mKd = kd;
    this->mDt = dt;
    this->mAlpha = alpha;

    encoder.begin(); // Initialize the encoder
}

void MotorController::setTargetVelocity(float target) {
    float velocity = encoder.getVelocity(mDt);
    float error = target - velocity;
    float output = pid.update(error);
    motor.setPWMPercent(static_cast<int8_t>(output)); // TODO: look into adding feedforward portion when velocity mapping is done
}
