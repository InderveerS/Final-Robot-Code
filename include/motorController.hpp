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
        static constexpr float FF_STATIC_DEADBAND = cfg::FF_STATIC_DEADBAND;
        static constexpr float FF_STALL_VEL = cfg::FF_STALL_VEL;
        static constexpr float FF_STALL_EXIT_VEL = cfg::FF_STALL_EXIT_VEL;

        MotorController(uint8_t mPin1, uint8_t mPin2, bool mInverted, mcpwm_unit_t mcpwmUnit, mcpwm_timer_t mcpwmTimer,
            uint8_t mEncPin1, uint8_t mEncPin2, pcnt_unit_t mUnit, float kp, float ki, float kd, float dt, float alpha);

        void setTargetVelocity(float target);

        // Telemetry accessors (last values from the most recent setTargetVelocity).
        float getLastMeasuredVelocity() const { return mLastVelocity; }
        float getTargetVelocity() const { return mRampedTarget; }
        float getLastOutput() const { return mLastOutput; }  // velocity PID output
        int getLastPwm() const { return mLastPwm; }           // final duty commanded
        bool isKicking() const { return mKickActive; }        // static-friction kick engaged?

        void resetPID() {
            pid.reset();
            mKickActive = false; // a new primitive re-evaluates from its own speed
        }

        Motor motor;
        Encoder encoder;

    private:
        float mLastVelocity = 0.0f;
        float mRampedTarget = 0.0f;
        float mLastOutput = 0.0f;
        int8_t mLastPwm = 0;
        // Latched so the kick has hysteresis: set below FF_STALL_VEL, cleared
        // above FF_STALL_EXIT_VEL. Starts false - at rest with no command there
        // is nothing to break away from.
        bool mKickActive = false;

        PID pid;
};