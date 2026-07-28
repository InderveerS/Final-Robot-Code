#pragma once
#include <Arduino.h>
#include "config.hpp"
#include "motorController.hpp"
#include "irArray.hpp"
#include "stopReason.hpp"
#include "eventDebounce.hpp"

class LineController {
    public:
        LineController(MotorController& leftMotor, MotorController& rightMotor, IRArray& irArray, float kp, float ki, float kd, float dt, float alpha);
        void updateLinePID(); // outer loop, can call differently than motor loop
        void updateMotorPID();

        // Blocking: line-follows until the event fires OR maxDistance (average
        // encoder travel, m) is reached, then returns why it stopped.
        //   event         : predicate polled each cycle; nullptr disables it
        //   maxDistance   : <= 0 disables the distance backstop
        //   confirmCycles : consecutive true samples needed before the event
        //                   counts. 1 = fire on the first true (original
        //                   behaviour). Raise it to reject IR/switch noise.
        //   baseVel       : m/s straight-line speed for THIS call, the
        //                   equivalent of moveUntil's velocity / turnUntil's
        //                   omegaDps. Tight corrections still scale it down by
        //                   VEL_CHANGE_CONST. Applies only to this call.
        //   stopAtEnd     : true stops the motors on exit, false coasts (keeps
        //                   velocity into the next primitive)
        StopReason followUntil(bool (*event)(), float maxDistance,
                                uint16_t confirmCycles = 1,
                                float baseVel = cfg::LINE_BASE_VEL,
                                bool stopAtEnd = true,
                                uint16_t timeoutMs = 10000, uint16_t delayMs = cfg::CONTROL_PERIOD_MS);

        float getLinePosition() const { return mLastLinePos; } // last readLine() value
        float getOmega() const { return omega; }               // last commanded angular vel

    private:
        MotorController& LeftMotor;
        MotorController& RightMotor;
        IRArray& irArray;

        static constexpr float target = cfg::LINE_TARGET;
        static constexpr float maxCorrection = cfg::LINE_MAX_CORRECTION;
        static constexpr float DEFAULT_BASE_VEL = cfg::LINE_BASE_VEL;
        static constexpr float VEL_CHANGE_CONST = cfg::LINE_VEL_CHANGE_CONST;

        // Live straight-line speed, set by followUntil() on every call so it can
        // never carry over from a previous one. updateMotorPID() reads it. If
        // you drive updateMotorPID() directly (the diagnostics line/motor PID
        // tasks do), this stays at DEFAULT_BASE_VEL.
        float mBaseVel = DEFAULT_BASE_VEL;

        float realVel = 0.0f;
        float mLastLinePos = 0.0f;

        PID linePID;

        volatile float omega = 0.0f; // angular velocity in rad/s
};