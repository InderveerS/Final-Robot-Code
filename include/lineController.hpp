#pragma once
#include <Arduino.h>
#include "config.hpp"
#include "motorController.hpp"
#include "irArray.hpp"
#include "stopReason.hpp"

class LineController {
    public:
        LineController(MotorController& leftMotor, MotorController& rightMotor, IRArray& irArray, float kp, float ki, float kd, float dt, float alpha);
        void updateLinePID(); // outer loop, can call differently than motor loop
        void updateMotorPID();

        // Blocking: line-follows until the event fires OR maxDistance (average
        // encoder travel, m) is reached, then returns why it stopped.
        //   event      : predicate polled each cycle; nullptr disables it
        //   maxDistance : <= 0 disables the distance backstop
        //   stopAtEnd  : true stops the motors on exit, false coasts (keeps
        //                velocity into the next primitive)
        StopReason followUntil(bool (*event)(), float maxDistance, uint16_t delayMs,
                               bool stopAtEnd = true, uint16_t timeoutMs = 10000);

    private:
        MotorController& LeftMotor;
        MotorController& RightMotor;
        IRArray& irArray;

        static constexpr float target = cfg::LINE_TARGET;
        static constexpr float maxCorrection = cfg::LINE_MAX_CORRECTION;
        static constexpr float baseVel = cfg::LINE_BASE_VEL;
        static constexpr float VEL_CHANGE_CONST = cfg::LINE_VEL_CHANGE_CONST;

        float realVel = 0.0f;

        PID linePID;

        volatile float omega = 0.0f; // angular velocity in rad/s
};