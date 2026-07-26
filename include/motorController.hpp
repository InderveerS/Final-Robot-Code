#pragma once

#include <Arduino.h>
#include "config.hpp"
#include "motor.hpp"
#include "pid.hpp"
#include "encoder.hpp"

class MotorController {
    public:

        static constexpr float COUNTS_PER_REV = cfg::COUNTS_PER_REV;
        static constexpr float WHEEL_CIRCUMFERENCE_M = cfg::WHEEL_CIRCUMFERENCE_M;
        static constexpr float OUT_MIN = cfg::MOTOR_OUT_MIN;
        static constexpr float OUT_MAX = cfg::MOTOR_OUT_MAX;
        static constexpr float INTEGRAL_THRESH = cfg::MOTOR_INTEGRAL_THRESH;
        static constexpr float MAX_ACCEL = cfg::MOTOR_MAX_ACCEL;
        static constexpr float WHEELBASE_M = cfg::WHEELBASE_M;

        static constexpr float FF_DEADBAND = cfg::FF_DEADBAND;
        static constexpr float FF_SLOPE = cfg::FF_SLOPE;
        static constexpr float FF_VEL_THRESHOLD = cfg::FF_VEL_THRESHOLD;

        MotorController(uint8_t mPin1, uint8_t mPin2, bool mInverted, mcpwm_unit_t mcpwmUnit, mcpwm_timer_t mcpwmTimer,
            uint8_t mEncPin1, uint8_t mEncPin2, pcnt_unit_t mUnit, float kp, float ki, float kd, float dt, float alpha);

        void setTargetVelocity(float target);

        float getLastMeasuredVelocity() const { return mLastVelocity; }

        void resetPID() {
            pid.reset();
        }

        Motor motor;
        Encoder encoder;

    private:
        float mLastVelocity = 0.0f;
        float mRampedTarget = 0.0f;

        PID pid;
};