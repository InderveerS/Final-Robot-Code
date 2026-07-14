#pragma once
#include <Arduino.h>
#include "motorController.hpp"

class DistanceController {
    public:
        DistanceController(MotorController& leftMotor, MotorController& rightMotor, float kp, float ki, float kd, float dt, float alpha);
        void updateDistancePID(float targetDistance); // target distance in meters
        void startDistanceCalculation(){
            LeftMotor.encoder.startDistance();
            RightMotor.encoder.startDistance();
        }

    private:

        float kp;
        float ki;
        float kd;
        float dt;
        float alpha;

        const float velMax = 0.5; // maximum velocity in m/s
        const float velMin = -0.5; // minimum velocity in m/s

        MotorController& LeftMotor;
        MotorController& RightMotor;

        PID distancePID;

};