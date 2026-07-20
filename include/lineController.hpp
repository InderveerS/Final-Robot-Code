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

        static constexpr float target = 0.0f;
        static constexpr float maxCorrection = 5.263394f;
        static constexpr float wheelbase = 0.254f;
        static constexpr float baseVel = 0.37f;
        static constexpr float VEL_CHANGE_CONST = 1.0f; // Adjust this constant to control how much the base velocity changes with angular velocity

        float realVel = 0.0f;

        PID linePID;

        float kp;
        float ki;
        float kd;
        float dt;
        float alpha;

        volatile float omega = 0.0f; // angular velocity in rad/s
};    