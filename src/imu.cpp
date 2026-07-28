#include "imu.hpp"

// Tuning constants live in config.hpp (cfg::IMU_*). Notes:
//  - IMU_GYRO_SIGN: gyro-z is right-hand-rule CCW-positive when the chip is
//    flat, z up. Bench check after any remount: CCW must increase getHeading().
//  - IMU_EULER_SIGN: BNO055 Euler heading is CW-positive, opposite the gyro.
//  - IMU_GYRO_SCALE: sensitivity trim; the fused Euler path has no such error.
//  - IMU_FUSION_HANDOFF_DPS: above this the fused Euler under-counts, so we
//    integrate the raw gyro; below it we use drift-free fused Euler deltas.
//  - IMU_EULER_RESUME_MS: after a fast turn the fused output slews its lag for
//    a few hundred ms; those degrees were already counted on the gyro path, so
//    we stay on the gyro that long to avoid double-counting.

Imu::Imu() : bno(55, BNO055_ADDRESS_A, &Wire) {}

bool Imu::begin() {
    Wire.begin(cfg::IMU_SDA_PIN, cfg::IMU_SCL_PIN);
    Wire.setClock(cfg::IMU_I2C_CLOCK_HZ);

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
    for (int i = 0; i < cfg::IMU_GYRO_BIAS_SAMPLES; i++) {
        sum += bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z(); // deg/s
        delay(10);
    }
    mGyroBias = (float)(sum / cfg::IMU_GYRO_BIAS_SAMPLES);

    return true;
}

void Imu::update() {
    uint32_t nowMicros = micros();
    // Rate register: deg/s, no fusion pipeline, valid to +/-2000 deg/s.
    float rate = cfg::IMU_GYRO_SIGN * cfg::IMU_GYRO_SCALE * ((float)bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z() - mGyroBias);
    // Fused Euler vector (one read): x = heading, y = roll, z = pitch.
    auto eulerVec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float euler = cfg::IMU_EULER_SIGN * (float)eulerVec.x(); // heading, CCW-positive
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

    if (fabsf(rate) > cfg::IMU_FUSION_HANDOFF_DPS) {
        mLastFastMicros = nowMicros;
    }

    // Gyro path while rotating fast AND for IMU_EULER_RESUME_MS afterwards, so
    // the fusion's post-turn catch-up slew never enters the sum. The choice is
    // recorded (isUsingGyro) so a log can attribute heading error to a path.
    mUsingGyro = (nowMicros - mLastFastMicros < (uint32_t)cfg::IMU_EULER_RESUME_MS * 1000);
    if (mUsingGyro) {
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
