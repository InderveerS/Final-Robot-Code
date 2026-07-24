#pragma once

#include <Arduino.h>
#include "motor.hpp"
#include "pid.hpp"
#include "encoder.hpp"

class MotorController {
    public:

        static constexpr float COUNTS_PER_REV = 1973.0f; 
        static constexpr float WHEEL_CIRCUMFERENCE_M = 0.29225f;
        static constexpr float OUT_MIN = -100.0f;
        static constexpr float OUT_MAX = 100.0f;
        static constexpr float INTEGRAL_THRESH = 0.1f; // Threshold for integral activation
        static constexpr float MAX_ACCEL = 0.6f; // Maximum acceleration in m/s^2
        static constexpr float WHEELBASE_M = 0.254f; // distance between wheel centers

        // Feedforward: duty = sign(v)*FF_DEADBAND + FF_SLOPE*v (affine fit of the
        // duty->velocity sweep's MOVING region - motion onset is ~37% but the
        // running curve extrapolates back to ~21; static > kinetic friction).
        // The curve saturates above ~0.52 m/s (near velMax), so top-end sweep
        // points are excluded from the fit. Retuned after the chassis rebuild.
        // Below FF_VEL_THRESHOLD we command 0 so the robot can actually stop.
        static constexpr float FF_DEADBAND = 21.0f;      // duty %
        static constexpr float FF_SLOPE = 84.0f;         // duty % per m/s
        static constexpr float FF_VEL_THRESHOLD = 0.02f; // m/s

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