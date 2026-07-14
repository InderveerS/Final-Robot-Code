#pragma once

#include <Arduino.h>
#include "motor.hpp"
#include "pid.hpp"
#include "encoder.hpp"

class MotorController {
    public:

        static constexpr float COUNTS_PER_REV = 2272.0f; 
        static constexpr float WHEEL_CIRCUMFERENCE_M = 0.2582f;
        static constexpr float OUT_MIN = -100.0f;
        static constexpr float OUT_MAX = 100.0f;
        static constexpr float MAX_VELOCITY = 0.669f;
        static constexpr float INTEGRAL_THRESH = 0.1f; // Threshold for integral activation
        static constexpr float MAX_ACCEL = 0.6f; // Maximum acceleration in m/s^2

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
        uint8_t mPin1;
        uint8_t mPin2;
        bool mInverted;

        uint8_t mEncPin1;
        uint8_t mEncPin2;
        pcnt_unit_t mUnit;

        float mKp;
        float mKi;
        float mKd;
        float mDt;
        float mAlpha;

        float mLastVelocity = 0.0f;
        float mRampedTarget = 0.0f;

        PID pid;

};    