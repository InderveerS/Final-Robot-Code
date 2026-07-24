#include "imu.hpp"

// Gyro z follows the right-hand rule: with the chip flat and z up, CCW
// rotation reads positive - matching the robot convention directly.
// Bench check after any remount: hand-rotate CCW, getHeading() must
// increase. If it decreases (e.g. chip mounted upside down), set to -1.
#define GYRO_SIGN 1.0f

// BNO055 Euler heading is compass-style (clockwise-positive), so it needs the
// opposite sign from the gyro. If GYRO_SIGN ever flips, flip this too.
#define EULER_SIGN -1.0f

// Gyro sensitivity trim correction. This unit over-reads rotation by ~2.7%
// (two full-turn tests read 369 and 370 deg for a true 360). Calibrate by
// spinning N continuous fast full turns: GYRO_SCALE = (N*360) / reported.
// Only affects the gyro path - the fused Euler path doesn't have this error.
#define GYRO_SCALE 0.974f

#define GYRO_BIAS_SAMPLES 100 // x 10 ms = ~1 s of rest-bias averaging

// Above this measured rate (deg/s) the fused Euler output under-counts, so
// heading switches to raw gyro integration. Below it, fused Euler deltas are
// used - they are drift-free at rest (the chip re-learns gyro bias on its
// own). Verified on this robot: fusion loss appears somewhere below 100 deg/s.
#define FUSION_HANDOFF_DPS 40.0f

// After a fast rotation ends, the fused Euler output slews its accumulated
// lag ("catches up") for a few hundred ms. Those degrees were already counted
// by the gyro path, so trusting Euler deltas immediately double-counts them
// (measured: +9 deg on a stop-and-go 360). Keep integrating the gyro for this
// long after the last fast sample so the catch-up slew is discarded.
#define EULER_RESUME_MS 500

Imu::Imu() : bno(55, BNO055_ADDRESS_A, &Wire) {}

bool Imu::begin() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(IMU_I2C_CLOCK_HZ);

    // Still IMUPLUS mode: we no longer use the fused Euler output (it loses
    // angle during fast rotation), but running the fusion keeps the chip's
    // own continuous gyro calibration active. The magnetometer stays unused -
    // motor currents corrupt it.
    if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
        return false;
    }
    delay(50);
    bno.setExtCrystalUse(true); // mode-cycles the chip internally, keep after begin()
    delay(100);

    // Capture the gyro-z rest bias (robot must be stationary). Subtracting
    // this in update() removes most of the integration drift.
    double sum = 0.0;
    for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
        sum += bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z(); // deg/s
        delay(10);
    }
    mGyroBias = (float)(sum / GYRO_BIAS_SAMPLES);

    return true;
}

void Imu::update() {
    uint32_t nowMicros = micros();
    // Rate register: deg/s, no fusion pipeline, valid to +/-2000 deg/s.
    float rate = GYRO_SIGN * GYRO_SCALE * ((float)bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z() - mGyroBias);
    // Fused Euler vector (one read): x = heading, y = roll, z = pitch.
    auto eulerVec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float euler = EULER_SIGN * (float)eulerVec.x(); // heading, CCW-positive
    mRoll = (float)eulerVec.y();                    // gravity-referenced, cache raw
    mPitch = (float)eulerVec.z();

    if (mFirstRead) {
        mLastMicros = nowMicros;
        mLastFastMicros = nowMicros - 1000000; // 1 s ago: start on the Euler path
        mLastRate = rate;
        mLastEuler = euler;
        mFirstRead = false;
        return; // mHeading stays 0: boot orientation is the reference
    }

    if (fabsf(rate) > FUSION_HANDOFF_DPS) {
        mLastFastMicros = nowMicros;
    }

    // Gyro path while rotating fast AND for EULER_RESUME_MS afterwards, so the
    // fusion's post-turn catch-up slew never enters the sum.
    if (nowMicros - mLastFastMicros < (uint32_t)EULER_RESUME_MS * 1000) {
        // Fast rotation: fused output under-counts, integrate the raw rate.
        // dt is measured, not assumed, so scheduling jitter doesn't corrupt
        // the angle (unsigned subtraction is correct across the micros() wrap).
        float dt = (nowMicros - mLastMicros) * 1e-6f;
        mHeading += 0.5f * (rate + mLastRate) * dt; // trapezoid
    } else {
        // Slow/stationary: fused Euler delta - zero drift at rest because the
        // chip keeps re-estimating its own gyro bias. wrapTo180 keeps the
        // one-sample delta correct across the 0/360 boundary.
        mHeading += wrapTo180(euler - mLastEuler);
    }

    mLastMicros = nowMicros;
    mLastRate = rate;
    mLastEuler = euler;
}

float Imu::wrapTo180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}
