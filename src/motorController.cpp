#include "motorController.hpp"


MotorController::MotorController(uint8_t mPin1, uint8_t mPin2, bool mInverted, mcpwm_unit_t mcpwmUnit, mcpwm_timer_t mcpwmTimer,
    uint8_t mEncPin1, uint8_t mEncPin2, pcnt_unit_t mUnit, float kp, float ki, float kd, float dt, float alpha) : 
    
    motor(mPin1, mPin2, mInverted, mcpwmUnit, mcpwmTimer), pid(kp, ki, kd, dt, OUT_MIN, OUT_MAX, alpha, INTEGRAL_THRESH), 
    encoder(mEncPin1, mEncPin2, mUnit, COUNTS_PER_REV, WHEEL_CIRCUMFERENCE_M) 
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
    // float maxDelta = MAX_ACCEL * mDt;
    // if (target > mRampedTarget + maxDelta)      mRampedTarget += maxDelta;
    // else if (target < mRampedTarget - maxDelta) mRampedTarget -= maxDelta;
    // else                                        mRampedTarget = target;

    mRampedTarget = target;

    float velocity = encoder.getVelocity();
    mLastVelocity = velocity; // Update the last measured velocity
    float error = mRampedTarget - velocity;
    float output = pid.update(error);

    // Affine feedforward with deadband, applied in the direction of travel.
    float feedforward = 0.0f;
    if (mRampedTarget > FF_VEL_THRESHOLD)       feedforward = FF_DEADBAND + FF_SLOPE * mRampedTarget;
    else if (mRampedTarget < -FF_VEL_THRESHOLD) feedforward = -FF_DEADBAND + FF_SLOPE * mRampedTarget;

    float final_pwm = constrain(feedforward + output, OUT_MIN, OUT_MAX);
    motor.setPWMPercent(static_cast<int8_t>(roundf(final_pwm)));
}
