#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define IMU_SDA_PIN 40
#define IMU_SCL_PIN 21
#define IMU_I2C_CLOCK_HZ 400000UL // drop to 100000UL if reads get flaky

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

        static float wrapTo180(float angle); // wrap into [-180, 180] degrees

    private:
        Adafruit_BNO055 bno;

        float mHeading = 0.0f;   // accumulated continuous heading (deg)
        float mGyroBias = 0.0f;  // gyro-z rest bias (deg/s), captured in begin()
        float mLastRate = 0.0f;  // previous rate sample (deg/s), for trapezoid
        float mLastEuler = 0.0f; // previous fused Euler heading (deg, CCW+)
        uint32_t mLastMicros = 0;
        uint32_t mLastFastMicros = 0; // last time |rate| exceeded the handoff
        bool mFirstRead = true;
};
