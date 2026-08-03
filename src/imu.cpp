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

    // The ~27 "i2cWriteReadNonStop returned Error -1" lines at every boot are
    // NORMAL and need no fix. Adafruit's begin() soft-resets the chip
    // (SYS_TRIGGER 0x20) and then polls the chip ID every 10 ms until it comes
    // back; each poll during the reset fails and the ESP32 Wire driver prints.
    // A delay here only moves them later. begin() cannot return until the ID
    // reads back, so setExtCrystalUse() below is always on an awake chip.
    // Errors AFTER this window are real - leave the logging on so they show.

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

    return true;
}

// Averages the gyro-z rest bias. The robot MUST be stationary throughout.
//
// Deliberately NOT part of begin(): begin() finishes ~100 ms after
// setExtCrystalUse() mode-cycles the chip, long before the chip's own gyro
// calibration has converged, so a bias measured there is taken on an unsettled
// sensor. Doing it there cost ~0.03 deg/s of residual - about 2 deg of heading
// over an 80 s run (LOG175: captured -0.042, settled value ~-0.01).
//
// Call this after the boot settle, or any other time the robot is known to be
// still. Until it is called mGyroBias is 0, which is degraded but not broken.
void Imu::captureBias() {
    double sum = 0.0;
    for (int i = 0; i < cfg::IMU_GYRO_BIAS_SAMPLES; i++) {
        sum += bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE).z(); // deg/s
        delay(10);
    }
    mGyroBias = (float)(sum / cfg::IMU_GYRO_BIAS_SAMPLES);
}

void Imu::update() {
    uint32_t nowMicros = micros();

    // Scheduling health, measured before anything else can distort it.
    mUpdates = mUpdates + 1;
    if (!mFirstRead) {
        uint32_t dt = nowMicros - mLastMicros; // unsigned, correct across wrap
        if (dt > mMaxDtUs) mMaxDtUs = dt;
    }

    // Rate register: deg/s, no fusion pipeline, valid to +/-2000 deg/s.
    auto gyroVec = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // getVector() zeroes its buffer and throws away readLen()'s success flag, so
    // a failed I2C read is returned as (0,0,0) with no way to tell it from data.
    // Exact compares are right: the values are int16/16.0, so zero is exact.
    // Ambiguous at rest (a still chip really can read 0 on all three), which is
    // why this is counted apart from the Euler one.
    if (gyroVec.x() == 0.0 && gyroVec.y() == 0.0 && gyroVec.z() == 0.0) {
        mGyroFaults = mGyroFaults + 1;
    }
    float rawZ = (float)gyroVec.z();
    float rate = cfg::IMU_GYRO_SIGN * cfg::IMU_GYRO_SCALE * (rawZ - mGyroBias);

    // Impossible rate: the turn PID is clamped to TURN_MAX_OMEGA, so nothing
    // the robot does gets near this. A value out here is corruption that passed
    // I2C framing, not motion. Hold the previous sample rather than integrate
    // it - one cycle of staleness is 1.6 deg at worst, the glitch is far more.
    if (fabsf(rate) > cfg::IMU_MAX_RATE_DPS) {
        if (fabsf(rate) > fabsf(mWorstRate)) mWorstRate = rate;
        mRateGlitches = mRateGlitches + 1;
        rate = mLastRate;
    }
    // float temp = rate;
    if(fabsf(rate) < 0.2f) {
        rate = 0.0f;
    }
    // Fused Euler vector (one readf): x = heading, y = roll, z = pitch.
    auto eulerVec = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // The trustworthy fault signal: heading, roll and pitch all exactly 0.00 at
    // once is not a real attitude once the robot has moved off boot orientation.
    // Counted only - the zeros still go through below, so this measures the
    // problem without changing the behaviour being measured.
    if (eulerVec.x() == 0.0 && eulerVec.y() == 0.0 && eulerVec.z() == 0.0) {
        mEulerFaults = mEulerFaults + 1;
    }
    float euler = cfg::IMU_EULER_SIGN * (float)eulerVec.x(); // heading, CCW-positive
    mRoll = (float)eulerVec.y();                    // gravity-referenced, cache raw
    mPitch = (float)eulerVec.z();

    // Attitude peaks since the last takeAttitudePeaks(). update() runs at 100 Hz
    // but the log samples ~22 Hz, so a rocking chassis (one wheel off the ground
    // on the ramp) is aliased away entirely by the instantaneous values. These
    // brackets survive the undersampling.
    if (mRoll  < mRollMin)  mRollMin  = mRoll;
    if (mRoll  > mRollMax)  mRollMax  = mRoll;
    if (mPitch < mPitchMin) mPitchMin = mPitch;
    if (mPitch > mPitchMax) mPitchMax = mPitch;

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

    // Same plausibility test on the fused path. A rejected step must NOT update
    // mLastEuler, so the next good sample measures across the whole gap instead
    // of anchoring to the corrupt value.
    const float dEuler = wrapTo180(euler - mLastEuler);
    const bool eulerOk = fabsf(dEuler) <= cfg::IMU_MAX_EULER_STEP_DEG;
    if (!eulerOk) {
        if (fabsf(dEuler) > fabsf(mWorstEulerStep)) mWorstEulerStep = dEuler;
        mEulerGlitches = mEulerGlitches + 1;
    }

    // Gyro path while rotating fast AND for IMU_EULER_RESUME_MS afterwards, so
    // the fusion's post-turn catch-up slew never enters the sum. The choice is
    // recorded (isUsingGyro) so a log can attribute heading error to a path.
    mUsingGyro = cfg::IMU_GYRO_ONLY ||
                 (nowMicros - mLastFastMicros < (uint32_t)cfg::IMU_EULER_RESUME_MS * 1000);
    if (mUsingGyro) {
        // Fast rotation: fused output under-counts, integrate the raw rate.
        // dt is measured, not assumed, so scheduling jitter doesn't corrupt
        // the angle (unsigned subtraction is correct across the micros() wrap).
        float dt = (nowMicros - mLastMicros) * 1e-6f;
        mHeading += 0.5f * (rate + mLastRate) * dt; // trapezoid
    } else {
        // Slow/stationary: fused Euler delta - zero drift at rest because the
        // chip keeps re-estimating its own gyro bias. wrapTo180 keeps the
        // one-sample delta correct across the 0/360 boundary. Scaled like the
        // gyro branch: without a trim here the two paths disagree on how far
        // the robot turned, which is what turnCountTestB was measuring.
        if (eulerOk) mHeading += cfg::IMU_EULER_SCALE * dEuler;
    }

    mLastMicros = nowMicros;
    mLastRate = rate;
    // Deliberately NOT updated on a rejected step: anchoring to a corrupt value
    // would turn one bad sample into two bad deltas.
    if (eulerOk) mLastEuler = euler;
    // No bias tracking here - see the note in config.hpp for what was tried and
    // why it was removed. mGyroBias is set once by captureBias() and held.
}

// Returns roll/pitch peak-to-peak since the previous call, then rearms. Coning
// error - the spurious yaw an integrator accumulates when a body rotates about
// two axes at once - scales with the SQUARE of that amplitude, so this is the
// quantity that matters when asking whether the ramp is corrupting heading.
// Reading the instantaneous roll/pitch at log rate cannot answer that.
void Imu::takeAttitudePeaks(float& rollPP, float& pitchPP) {
    // Guard the first call, before update() has bracketed anything.
    rollPP  = (mRollMax  >= mRollMin)  ? (mRollMax  - mRollMin)  : 0.0f;
    pitchPP = (mPitchMax >= mPitchMin) ? (mPitchMax - mPitchMin) : 0.0f;
    mRollMin = mPitchMin = 1e9f;
    mRollMax = mPitchMax = -1e9f;
}

// Re-references the heading estimate to a known absolute field heading. Use it
// where the robot's physical orientation is externally constrained - the end of
// a line-follow being the usable case on this field, since the line is fixed to
// the field while the habitat moves.
//
// Adds the wrapped difference rather than assigning, so the result is the NEAREST
// equivalent of absHeadingDeg to the current estimate. That preserves the
// continuous/unwrapped heading that turnTo() depends on: a bare assignment after
// several rotations would leave heading near 0 while the robot still "knew" it had
// turned 3600, and the next turnTo() would unwind all of it. This form can never
// move heading by more than 180 deg.
//
// Only worth calling if absHeadingDeg is known in the BOOT frame to better than
// the drift being removed. If that number was itself read off a drifting
// estimate, this re-references to a moving target and buys nothing.
//
// Threading: mHeading is otherwise written only by update(), on the IMU task.
// Calling this from the mission task can drop a single 10 ms increment - a few
// hundredths of a degree - but cannot produce a torn value. Not worth a lock
// against the degrees this exists to remove.
void Imu::setHeading(float absHeadingDeg) {
    mHeading += wrapTo180(absHeadingDeg - mHeading);
}

float Imu::wrapTo180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}
