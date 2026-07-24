#pragma once
#include <Arduino.h>
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

        static constexpr float target = 0.0f;
        static constexpr float maxCorrection = 5.263394f;
        static constexpr float wheelbase = 0.254f;
        static constexpr float baseVel = 0.42f;
        static constexpr float VEL_CHANGE_CONST = 1.2f; // Adjust this constant to control how much the base velocity changes with angular velocity

        float realVel = 0.0f;

        PID linePID;

        float kp;
        float ki;
        float kd;
        float dt;
        float alpha;

        volatile float omega = 0.0f; // angular velocity in rad/s
};    