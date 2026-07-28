#pragma once
#include <Arduino.h>
#include "config.hpp"
#include "motorController.hpp"
#include "imu.hpp"
#include "stopReason.hpp"
#include "eventDebounce.hpp"

class TurnController {
    public:
        TurnController(MotorController& leftMotor, MotorController& rightMotor, Imu& imu, float kp, float ki, float kd, float dt, float alpha);

        void turnTo(float absHeadingDeg); // shortest path to an absolute heading
        void turnBy(float deltaDeg);      // relative turn, can exceed 180 deg
        void update();                    // call at 100 Hz while turning
        bool isSettled() const { return mSettledCount >= SETTLE_CYCLES; }
        float getTargetHeading() const { return mTargetHeading; }
        // Blocking: turns to the absolute heading absHeadingDeg (shortest path)
        // then returns. Gives up after timeoutMs so a turn that can never settle
        // can't hang the mission.
        void turn(float absHeadingDeg, uint16_t timeoutMs = 5000, uint16_t delayMs = cfg::CONTROL_PERIOD_MS);

        // Blocking: spins in place at a CONSTANT rate (independent of the
        // turn-PID clamp) until the event fires OR maxAngle (deg turned from
        // the start heading) is reached. Returns why.
        //   omegaDps      : deg/s, signed (positive = CCW); the search rate
        //   event         : predicate polled each cycle; nullptr disables it
        //   maxAngle      : <= 0 disables the angle backstop
        //   confirmCycles : consecutive true samples needed before the event
        //                   counts. 1 = fire on the first true (original
        //                   behaviour). Raise it to reject IR/switch noise.
        //   stopAtEnd     : true stops the motors on exit, false coasts
        StopReason turnUntil(float omegaDps, bool (*event)(), float maxAngle,
                             uint16_t confirmCycles = 1,
                             bool stopAtEnd = true, uint16_t timeoutMs = 10000, uint16_t delayMs = cfg::CONTROL_PERIOD_MS);

    private:
        static constexpr float maxOmega = cfg::TURN_MAX_OMEGA;
        // Minimum commanded rate outside the settle band: keeps wheel speeds
        // above static-friction breakaway so small turns can't stall short.
        static constexpr float minOmega = cfg::TURN_MIN_OMEGA;
        static constexpr float settleTolerance = cfg::TURN_SETTLE_TOLERANCE;
        static constexpr int SETTLE_CYCLES = cfg::TURN_SETTLE_CYCLES;

        MotorController& LeftMotor;
        MotorController& RightMotor;
        Imu& imu;

        PID anglePID;

        float mTargetHeading = 0.0f;
        int mSettledCount = 0;
};
