#pragma once
#include <Arduino.h>
#include "motorController.hpp"
#include "imu.hpp"

class TurnController {
    public:
        TurnController(MotorController& leftMotor, MotorController& rightMotor, Imu& imu, float kp, float ki, float kd, float dt, float alpha);

        void turnTo(float absHeadingDeg); // shortest path to an absolute heading
        void turnBy(float deltaDeg);      // relative turn, can exceed 180 deg
        void update();                    // call at 100 Hz while turning
        bool isSettled() const { return mSettledCount >= SETTLE_CYCLES; }

    private:
        static constexpr float maxOmega = 170.0f;        // deg/s (~3 rad/s)
        // Minimum commanded rate outside the settle band: keeps wheel speeds
        // above static-friction breakaway so small turns can't stall short.
        // If it hunts across the band, lower this; if it still stalls, raise.
        static constexpr float minOmega = 45.0f;         // deg/s
        static constexpr float settleTolerance = 0.5f;   // deg
        static constexpr int SETTLE_CYCLES = 10;         // consecutive in-tolerance updates

        MotorController& LeftMotor;
        MotorController& RightMotor;
        Imu& imu;

        PID anglePID;

        float mTargetHeading = 0.0f;
        int mSettledCount = 0;
};
