#include "motorController.hpp"


MotorController::MotorController(uint8_t mPin1, uint8_t mPin2, bool mInverted, mcpwm_unit_t mcpwmUnit, mcpwm_timer_t mcpwmTimer,
    uint8_t mEncPin1, uint8_t mEncPin2, pcnt_unit_t mUnit, float kp, float ki, float kd, float dt, float alpha) :
    motor(mPin1, mPin2, mInverted, mcpwmUnit, mcpwmTimer),
    pid(kp, ki, kd, dt, OUT_MIN, OUT_MAX, alpha, INTEGRAL_THRESH),
    encoder(mEncPin1, mEncPin2, mUnit, COUNTS_PER_REV, WHEEL_CIRCUMFERENCE_M)
{
    encoder.begin();
}

void MotorController::setTargetVelocity(float target) {
    mRampedTarget = target;

    float velocity = encoder.getVelocity();
    mLastVelocity = velocity;
    float error = mRampedTarget - velocity;
    float output = pid.update(error);
    mLastOutput = output;

    // Static-friction kick: while this wheel is commanded to move but reads
    // stopped, swap in the breakaway deadband. Latched with hysteresis so the
    // 16-point duty step can't chatter around the threshold. Only the deadband
    // changes - the slope still describes the moving region, and the PID is
    // untouched, so once the wheel breaks free this releases on its own.
    float speed = fabsf(velocity);
    if (mKickActive) {
        if (speed > FF_STALL_EXIT_VEL) mKickActive = false;
    } else if (speed < FF_STALL_VEL) {
        mKickActive = true;
    }
    const float deadband = mKickActive ? FF_STATIC_DEADBAND : FF_DEADBAND;

    // Affine feedforward with deadband, applied in the direction of travel.
    float feedforward = 0.0f;
    if (mRampedTarget > FF_VEL_THRESHOLD)       feedforward = deadband + FF_SLOPE * mRampedTarget;
    else if (mRampedTarget < -FF_VEL_THRESHOLD) feedforward = -deadband + FF_SLOPE * mRampedTarget;

    float final_pwm = constrain(feedforward + output, OUT_MIN, OUT_MAX);
    mLastPwm = static_cast<int8_t>(roundf(final_pwm));
    motor.setPWMPercent(mLastPwm);
}
