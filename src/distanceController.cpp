#include "distanceController.hpp"

DistanceController::DistanceController(MotorController& leftMotor, MotorController& rightMotor, Imu& imu,
    float kp, float ki, float kd, float hKp, float hKi, float hKd, float dt, float alpha)
    : LeftMotor(leftMotor), RightMotor(rightMotor), imu(imu),
      distancePID(kp, ki, kd, dt, velMin, velMax, alpha, 0.0f),
      headingPID(hKp, hKi, hKd, dt, -maxHeadingOmega, maxHeadingOmega, alpha, 0.0f)
{
}

void DistanceController::startMove(float targetDistance) {
    LeftMotor.encoder.startDistance();
    RightMotor.encoder.startDistance();
    mTargetDistance = targetDistance;
    mTargetHeading = imu.getHeading();
    distancePID.reset();
    headingPID.reset();
    mSettledCount = 0;
}

void DistanceController::update() {
    float leftDistance = LeftMotor.encoder.getDistance();
    float rightDistance = RightMotor.encoder.getDistance();
    float averageDistance = (leftDistance + rightDistance) / 2.0f;
    mLastDistance = averageDistance;

    float distError = mTargetDistance - averageDistance;
    float v = distancePID.update(distError);

    // Static-friction floor: P alone commands speeds below breakaway near the
    // target, stalling the move short. Outside the settle band enforce a
    // minimum speed; inside it, coast to a stop.
    if (fabsf(distError) <= settleTolerance) {
        v = 0.0f;
    } else if (fabsf(v) < minVel) {
        float dir = (v != 0.0f) ? v : distError;
        v = (dir < 0.0f) ? -minVel : minVel;
    }

    // Heading hold: differential correction on top of the forward command.
    // The correction is differential, so the encoder average above stays a
    // valid measure of forward travel.
    float headingError = mTargetHeading - imu.getHeading(); // deg
    float omega = headingPID.update(headingError);          // deg/s

    // omega*wheelbase/2 is only valid in rad/s, hence the one conversion here
    float halfV = omega * DEG_TO_RAD * MotorController::WHEELBASE_M / 2.0f;
    LeftMotor.setTargetVelocity(v - halfV);
    RightMotor.setTargetVelocity(v + halfV);

    if (fabsf(distError) < settleTolerance) {
        if (mSettledCount < SETTLE_CYCLES) mSettledCount++;
    } else {
        mSettledCount = 0;
    }
}

void DistanceController::move(float targetDistance, uint16_t delayMs, uint16_t timeoutMs) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(delayMs);
    const TickType_t deadline = xLastWakeTime + pdMS_TO_TICKS(timeoutMs);

    startMove(targetDistance);
    // Block until settled or the deadline passes. The signed tick difference
    // handles the (rare) tick-counter wraparound correctly.
    while (!isSettled() && (int32_t)(deadline - xTaskGetTickCount()) > 0) {
        update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    // Stop so a finished or timed-out move doesn't keep commanding the motors.
    LeftMotor.setTargetVelocity(0.0f);
    RightMotor.setTargetVelocity(0.0f);
}

StopReason DistanceController::moveUntil(float velocity, bool (*event)(), float maxDistance,
                                        uint16_t delayMs, bool stopAtEnd, uint16_t timeoutMs) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(delayMs);
    const TickType_t deadline = xLastWakeTime + pdMS_TO_TICKS(timeoutMs);

    LeftMotor.encoder.startDistance();
    RightMotor.encoder.startDistance();
    mTargetHeading = imu.getHeading();
    headingPID.reset();

    StopReason reason = StopReason::Timeout;
    for (;;) {
        // Constant commanded speed (NOT the distance PID), heading hold keeps
        // the path straight. Must re-command each cycle to keep the velocity
        // loop closed.
        float headingError = mTargetHeading - imu.getHeading();
        float omega = headingPID.update(headingError);
        float halfV = omega * DEG_TO_RAD * MotorController::WHEELBASE_M / 2.0f;
        LeftMotor.setTargetVelocity(velocity - halfV);
        RightMotor.setTargetVelocity(velocity + halfV);

        if (event != nullptr && event()) { reason = StopReason::Event; break; }

        float avgDist = (LeftMotor.encoder.getDistance() + RightMotor.encoder.getDistance()) / 2.0f;
        if (maxDistance > 0.0f && fabsf(avgDist) >= maxDistance) { reason = StopReason::Limit; break; }

        if ((int32_t)(deadline - xTaskGetTickCount()) <= 0) { reason = StopReason::Timeout; break; }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    if (stopAtEnd) {
        LeftMotor.setTargetVelocity(0.0f);
        RightMotor.setTargetVelocity(0.0f);
    }
    return reason;
}