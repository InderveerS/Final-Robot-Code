#pragma once

#include <Arduino.h>
#include "motor.hpp"
#include "pid.hpp"
#include "encoder.hpp"

class MotorController {
    public:
        MotorController(uint8_t mPin1, uint8_t mPin2, bool mInverted, uint8_t mEncPin1, uint8_t mEncPin2, 
            pcnt_unit_t mUnit, float kp, float ki, float kd, float dt, float alpha);

        void setTargetVelocity(float target);

    private:
        uint8_t mPin1;
        uint8_t mPin2;
        bool mInverted;

        uint8_t mEncPin1;
        uint8_t mEncPin2;
        const float mCountsPerRev = 2048.0; // TODO: update this to match the actual encoder counts per revolution
        const float mWheelCircumferenceM = 0.08255; // Double check this
        pcnt_unit_t mUnit;

        float mKp;
        float mKi;
        float mKd;
        float mDt;
        const float mOutMin = -100.0;
        const float mOutMax = 100.0;
        float mAlpha;

        Motor motor;
        PID pid;
        Encoder encoder;
};    