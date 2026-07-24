#pragma once
#include <Arduino.h>
#include "motorController.hpp"
#include "imu.hpp"
#include "stopReason.hpp"

class TurnController {
    public:
        TurnController(MotorController& leftMotor, MotorController& rightMotor, Imu& imu, float kp, float ki, float kd, float dt, float alpha);

        void turnTo(float absHeadingDeg); // shortest path to an absolute heading
        void turnBy(float deltaDeg);      // relative turn, can exceed 180 deg
        void update();                    // call at 100 Hz while turning
        bool isSettled() const { return mSettledCount >= SETTLE_CYCLES; }
        // Blocking: turns deltaDeg then returns. Gives up after timeoutMs so a
        // turn that can never settle can't hang the mission.
        void turn(float deltaDeg, uint16_t delayMs, uint16_t timeoutMs = 5000);

        // Blocking: spins in place at a CONSTANT rate (independent of the
        // turn-PID clamp) until the event fires OR maxAngle (deg turned from
        // the start heading) is reached. Returns why.
        //   omegaDps   : deg/s, signed (positive = CCW); the search rate
        //   event      : predicate polled each cycle; nullptr disables it
        //   maxAngle   : <= 0 disables the angle backstop
        //   stopAtEnd  : true stops the motors on exit, false coasts
        StopReason turnUntil(float omegaDps, bool (*event)(), float maxAngle,
                             uint16_t delayMs, bool stopAtEnd = true, uint16_t timeoutMs = 10000);

    private:
        static constexpr float maxOmega = 170.0f;        // deg/s (~3 rad/s)
        // Minimum commanded rate outside the settle band: keeps wheel speeds
        // above static-friction breakaway so small turns can't stall short.
        // If it hunts across the band, lower this; if it still stalls, raise.
        static constexpr float minOmega = 68.0f;         // deg/s
        static constexpr float settleTolerance = 0.5f;   // deg
        static constexpr int SETTLE_CYCLES = 10;         // consecutive in-tolerance updates

        MotorController& LeftMotor;
        MotorController& RightMotor;
        Imu& imu;

        PID anglePID;

        float mTargetHeading = 0.0f;
        int mSettledCount = 0;
};
