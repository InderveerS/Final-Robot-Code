#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "config.hpp"

class Imu {
    public:
        Imu();

        // Configures I2C and puts the chip in IMUPLUS. Does NOT measure the gyro
        // bias - call captureBias() for that, once the robot has settled.
        bool begin();

        // Measures the gyro's rest bias (~1 s of averaging); the robot MUST be
        // stationary. Separate from begin() on purpose - see imu.cpp. Call it
        // after the boot settle so the chip's own gyro calibration has converged
        // first, otherwise the estimate carries ~0.03 deg/s of error into every
        // raw-gyro integration. Safe to call again whenever the robot is still.
        void captureBias();

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

        // Landmark reset: re-reference the estimate to a known absolute field
        // heading, discarding whatever drift has accumulated. Intended for use
        // right after a line-follow has physically aligned the robot with a line
        // of known bearing. Lands on the NEAREST equivalent angle, so the
        // continuous/unwrapped property above survives - see imu.cpp.
        void setHeading(float absHeadingDeg);

        // Tilt from the fused Euler output (DEGREES), absolute (not zeroed at
        // start). Gravity-referenced, so unlike heading these are drift-free
        // and unaffected by rotation speed - reliable even during a run. Which
        // one tracks a ramp depends on how the chip is mounted; check on
        // hardware which moves when the robot tips forward, and its sign.
        float getRoll() const { return mRoll; }   // BNO055 Euler y
        float getPitch() const { return mPitch; } // BNO055 Euler z

        // Roll/pitch peak-to-peak since the previous call, then rearms. update()
        // runs at 100 Hz and the log samples ~22 Hz, so instantaneous roll/pitch
        // alias away chassis rocking entirely; these brackets do not. Call from
        // ONE reader only (the logging task) - each call consumes the window.
        void takeAttitudePeaks(float& rollPP, float& pitchPP);

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
        // Rest bias in raw sensor deg/s, set once by captureBias() and then held
        // constant - update() does NOT track it (see config.hpp). Logged so the
        // boot capture can be checked run to run.
        float getGyroBias() const { return mGyroBias; }

        // I2C read failures. Adafruit's getVector() memsets its buffer to zero,
        // calls readLen(), and DISCARDS readLen's success flag - so a failed
        // transaction returns (0,0,0), indistinguishable from real data. A
        // dropped gyro sample silently loses ~1.6 deg at a 160 deg/s turn rate
        // (the trapezoid halves two consecutive samples), which is why this is
        // worth counting. Exact float compares are correct here: the values are
        // int16/16.0, so a true zero is exactly zero.
        //
        // Euler is the trustworthy signal - roll, pitch and heading all reading
        // exactly 0.00 at once does not happen on a real surface once the robot
        // has moved. Gyro all-zero is AMBIGUOUS at rest (a still chip really can
        // read 0 on all three axes), so it is counted separately and should not
        // be used as the fault indicator.
        uint32_t getEulerFaults() const { return mEulerFaults; }
        uint32_t getGyroFaults() const { return mGyroFaults; }

        // Scheduling health. update() integrates over a MEASURED dt, so a late
        // sample is handled correctly - but only if the rate did not change much
        // across the gap. At 160 deg/s a 100 ms stall integrates one trapezoid
        // across a whole acceleration and can lose several degrees in one go.
        // Wire.cpp logs its I2C errors through Serial from inside this task, so
        // a full TX buffer stalls the sampler exactly when the bus is unhappy.
        // Expect maxDt ~10-12 ms; anything above ~20 ms is a real stall.
        uint32_t getMaxDtUs() const { return mMaxDtUs; }
        uint32_t getUpdates() const { return mUpdates; }

        // Physically impossible readings, counted AND rejected. This is the
        // failure the (0,0,0) counters CANNOT see: a bit flip in the I2C data
        // phase ACKs normally and returns a wrong-but-plausible number. One bad
        // gyro sample injects (value/100) deg into the heading in a single
        // cycle, which is the only mechanism left that fits a rare 45 deg miss
        // on a test where everything else reads clean.
        uint32_t getRateGlitches() const { return mRateGlitches; }
        uint32_t getEulerGlitches() const { return mEulerGlitches; }
        // Worst rejected values, so a glitch can be told from a threshold set
        // too tight - a real corruption is usually wildly out, not marginal.
        float getWorstRate() const { return mWorstRate; }
        float getWorstEulerStep() const { return mWorstEulerStep; }

        // Call once the robot starts driving. At boot on a level floor Euler
        // genuinely IS (0,0,0) - heading starts at zero and the chip is flat -
        // so the stationary opening would otherwise count as pure fault.
        void resetFaultCounts() {
            mEulerFaults = 0; mGyroFaults = 0; mMaxDtUs = 0; mUpdates = 0;
            mRateGlitches = 0; mEulerGlitches = 0;
            mWorstRate = 0.0f; mWorstEulerStep = 0.0f;
        }

        static float wrapTo180(float angle); // wrap into [-180, 180] degrees

    private:
        Adafruit_BNO055 bno;

        float mHeading = 0.0f;   // accumulated continuous heading (deg)
        float mRoll = 0.0f;      // fused Euler roll (deg), gravity-referenced
        float mPitch = 0.0f;     // fused Euler pitch (deg), gravity-referenced
        // Running attitude brackets, drained by takeAttitudePeaks(). Seeded
        // inverted so the first update() sets both ends.
        float mRollMin = 1e9f,  mRollMax = -1e9f;
        float mPitchMin = 1e9f, mPitchMax = -1e9f;
        float mGyroBias = 0.0f;  // gyro-z rest bias (deg/s), captured in begin()
        float mLastRate = 0.0f;  // previous rate sample (deg/s), for trapezoid
        float mLastEuler = 0.0f; // previous fused Euler heading (deg, CCW+)
        uint32_t mLastMicros = 0;
        uint32_t mLastFastMicros = 0; // last time |rate| exceeded the handoff
        bool mUsingGyro = false;      // which path the last update() took
        bool mFirstRead = true;
        // Bumped in update() on the IMU task, read from the mission task.
        // Aligned 32-bit access is atomic on Xtensa, so no lock is needed.
        volatile uint32_t mEulerFaults = 0;
        volatile uint32_t mGyroFaults = 0;
        volatile uint32_t mMaxDtUs = 0;
        volatile uint32_t mUpdates = 0;
        volatile uint32_t mRateGlitches = 0;
        volatile uint32_t mEulerGlitches = 0;
        volatile float mWorstRate = 0.0f;
        volatile float mWorstEulerStep = 0.0f;
};
