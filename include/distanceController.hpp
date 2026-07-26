#pragma once
#include <Arduino.h>
#include "config.hpp"
#include "motorController.hpp"
#include "imu.hpp"
#include "stopReason.hpp"

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
        // Blocking: drives targetDistance then returns. Gives up after
        // timeoutMs so a move that can never settle can't hang the mission.
        void move(float targetDistance, uint16_t delayMs, uint16_t timeoutMs = 8000);

        // Blocking: drives straight at a CONSTANT velocity (independent of the
        // distance-PID clamps) with heading hold, until the event fires OR
        // maxDistance (average encoder travel, m) is reached. Returns why.
        //   velocity   : m/s, signed (negative = reverse); the search speed
        //   event      : predicate polled each cycle; nullptr disables it
        //   maxDistance : <= 0 disables the distance backstop
        //   stopAtEnd  : true stops the motors on exit, false coasts
        StopReason moveUntil(float velocity, bool (*event)(), float maxDistance,
                             uint16_t delayMs, bool stopAtEnd = true, uint16_t timeoutMs = 10000);

    private:
        static constexpr float velMax = cfg::DIST_VEL_MAX;
        static constexpr float velMin = cfg::DIST_VEL_MIN;
        // Minimum commanded speed outside the settle band: keeps the wheels
        // above static-friction breakaway so moves can't stall short of target.
        static constexpr float minVel = cfg::DIST_MIN_VEL;
        static constexpr float maxHeadingOmega = cfg::DIST_MAX_HEADING_OMEGA;
        static constexpr float settleTolerance = cfg::DIST_SETTLE_TOLERANCE;
        static constexpr int SETTLE_CYCLES = cfg::DIST_SETTLE_CYCLES;

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
