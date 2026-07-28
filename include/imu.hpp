#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "config.hpp"

class Imu {
    public:
        Imu();

        // Robot must be STATIONARY during begin(): it captures the gyro's
        // rest bias (~1 s of averaging) that update() subtracts forever after.
        bool begin();

        // Two I2C reads per call - call at 100 Hz, and only ever from a single
        // task (Wire is not thread-safe). Hybrid heading: below the handoff
        // rate the fused Euler delta is used (drift-free at rest - the chip
        // continuously re-learns gyro bias), above it the raw gyro rate is
        // integrated (the fusion under-counts fast rotations, the rate
        // register does not). Only one-sample deltas are ever used, so the
        // angle the fusion loses during fast turns never enters the sum.
        void update();

        // Continuous heading in DEGREES, CCW-positive, zero at whatever
        // direction the robot faced at startup.
        // Cached value - no I2C, safe to call from anywhere.
        float getHeading() const { return mHeading; }

        // Tilt from the fused Euler output (DEGREES), absolute (not zeroed at
        // start). Gravity-referenced, so unlike heading these are drift-free
        // and unaffected by rotation speed - reliable even during a run. Which
        // one tracks a ramp depends on how the chip is mounted; check on
        // hardware which moves when the robot tips forward, and its sign.
        float getRoll() const { return mRoll; }   // BNO055 Euler y
        float getPitch() const { return mPitch; } // BNO055 Euler z

        // Diagnostics: the internals behind the hybrid heading, so a log can
        // tell "the turn controller missed" apart from "getHeading() lied".
        // All cached from the last update() - no I2C, safe from any task.
        // getRawEuler() is the chip's own ABSOLUTE fused heading, not zeroed at
        // boot like getHeading(), so compare their per-sample DELTAS, not their
        // values: if heading moves further than the Euler over a turn, the gyro
        // path (IMU_GYRO_SCALE) is over-counting, and vice versa.
        float getRawEuler() const { return mLastEuler; } // fused Euler heading (deg, CCW+)
        float getRate() const { return mLastRate; }      // bias-corrected gyro-z (deg/s)
        bool isUsingGyro() const { return mUsingGyro; }  // last update integrated raw gyro?

        static float wrapTo180(float angle); // wrap into [-180, 180] degrees

    private:
        Adafruit_BNO055 bno;

        float mHeading = 0.0f;   // accumulated continuous heading (deg)
        float mRoll = 0.0f;      // fused Euler roll (deg), gravity-referenced
        float mPitch = 0.0f;     // fused Euler pitch (deg), gravity-referenced
        float mGyroBias = 0.0f;  // gyro-z rest bias (deg/s), captured in begin()
        float mLastRate = 0.0f;  // previous rate sample (deg/s), for trapezoid
        float mLastEuler = 0.0f; // previous fused Euler heading (deg, CCW+)
        uint32_t mLastMicros = 0;
        uint32_t mLastFastMicros = 0; // last time |rate| exceeded the handoff
        bool mUsingGyro = false;      // which path the last update() took
        bool mFirstRead = true;
};
