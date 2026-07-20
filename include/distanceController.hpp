#pragma once
#include <Arduino.h>
#include "motorController.hpp"
#include "imu.hpp"

class DistanceController {
    public:
        DistanceController(MotorController& leftMotor, MotorController& rightMotor, Imu& imu,
            float kp, float ki, float kd, float hKp, float hKi, float hKd, float dt, float alpha);

        // Marks the encoder reference and captures the current IMU heading as
        // the hold reference - the robot drives targetDistance meters holding
        // whatever direction it faced when this was called.
        void startMove(float targetDistance);
        void update(); // call at 100 Hz while driving
        bool isSettled() const { return mSettledCount >= SETTLE_CYCLES; }
        float getDistance() const { return mLastDistance; }

    private:
        static constexpr float velMax = 0.5f;            // m/s
        static constexpr float velMin = -0.5f;           // m/s
        // Minimum commanded speed outside the settle band: keeps the wheels
        // above static-friction breakaway so moves can't stall short of the
        // target (same idea as TurnController::minOmega, 45 deg/s ~ 0.10 m/s).
        // If it hunts across the band, lower this; if it still stalls, raise.
        static constexpr float minVel = 0.10f;           // m/s
        static constexpr float maxHeadingOmega = 85.0f;  // deg/s (~1.5 rad/s), keeps correction gentle
        static constexpr float settleTolerance = 0.01f;  // m
        static constexpr int SETTLE_CYCLES = 10;         // consecutive in-tolerance updates

        MotorController& LeftMotor;
        MotorController& RightMotor;
        Imu& imu;

        PID distancePID;
        PID headingPID;

        float mTargetDistance = 0.0f;
        float mTargetHeading = 0.0f;
        float mLastDistance = 0.0f;
        int mSettledCount = 0;
};
