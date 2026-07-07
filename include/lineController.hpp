#pragma once 
#include <Arduino.h>
#include "motorController.hpp"
#include "irArray.hpp"

class LineController {
    public:
        LineController(MotorController& leftMotor, MotorController& rightMotor, IRArray& irArray, float kp, float ki, float kd, float dt, float alpha);
        void updateLinePID(); // outer loop, can call differently than motor loop
        void updateMotorPID();

    private:
        MotorController& LeftMotor;
        MotorController& RightMotor;
        IRArray& irArray;

        PID linePID;

        float kp;
        float ki;
        float kd;
        float dt;
        float alpha;

        const float target = 0.0f; // target line position (centered)
        const float maxCorrection = 3.0f; // maximum correction in rad/s
        const float wheelbase = 0.5f;
        const float baseVel = 0.5f; // base velocity in m/s

        volatile float omega = 0.0f; // angular velocity in rad/s
};    