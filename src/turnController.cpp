#include "turnController.hpp"
#include "telemetry.hpp"

TurnController::TurnController(MotorController& leftMotor, MotorController& rightMotor, Imu& imu, float kp, float ki, float kd, float dt, float alpha) :
    LeftMotor(leftMotor), RightMotor(rightMotor), imu(imu), anglePID(kp, ki, kd, dt, -maxOmega, maxOmega, alpha, 0.0f)
{
}

void TurnController::turnTo(float absHeadingDeg) {
    float heading = imu.getHeading();
    // heading is unwrapped/continuous, so aim for the nearest equivalent angle
    mTargetHeading = heading + Imu::wrapTo180(absHeadingDeg - heading);
    anglePID.reset();
    mSettledCount = 0;
}

void TurnController::turnBy(float deltaDeg) {
    mTargetHeading = imu.getHeading() + deltaDeg;
    anglePID.reset();
    mSettledCount = 0;
}

void TurnController::update() {
    float error = mTargetHeading - imu.getHeading(); // deg
    float omega = anglePID.update(error);            // deg/s

    // Static-friction floor: P alone commands wheel speeds below breakaway
    // for small errors, so small turns stall short of the target. Outside
    // the settle band enforce a minimum rate; inside it, coast to a stop.
    if (fabsf(error) <= settleTolerance) {
        omega = 0.0f;
    } else if (fabsf(omega) < minOmega) {
        float dir = (omega != 0.0f) ? omega : error;
        omega = (dir < 0.0f) ? -minOmega : minOmega;
    }

    // Point turn: zero forward velocity, pure differential.
    // omega*wheelbase/2 is only valid in rad/s, hence the one conversion here.
    float halfV = omega * DEG_TO_RAD * MotorController::WHEELBASE_M / 2.0f;
    LeftMotor.setTargetVelocity(-halfV);
    RightMotor.setTargetVelocity(halfV);

    if (fabsf(error) < settleTolerance) {
        if (mSettledCount < SETTLE_CYCLES) mSettledCount++;
    } else {
        mSettledCount = 0;
    }
}

void TurnController::turn(float absHeadingDeg, uint16_t timeoutMs, uint16_t delayMs) {
    telemetry::setActivity("TURN", absHeadingDeg, false, nullptr);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(delayMs);
    const TickType_t deadline = xLastWakeTime + pdMS_TO_TICKS(timeoutMs);

    turnTo(absHeadingDeg);
    // Block until settled or the deadline passes. The signed tick difference
    // handles the (rare) tick-counter wraparound correctly.
    while (!isSettled() && (int32_t)(deadline - xTaskGetTickCount()) > 0) {
        update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    // Stop so a finished or timed-out turn doesn't keep commanding the motors.
    LeftMotor.setTargetVelocity(0.0f);
    RightMotor.setTargetVelocity(0.0f);
}

StopReason TurnController::turnUntil(float omegaDps, bool (*event)(), float maxAngle,
                                     uint16_t confirmCycles,
                                     bool stopAtEnd, uint16_t timeoutMs, uint16_t delayMs) {
    telemetry::setActivity("TURN_UNTIL", maxAngle, event != nullptr, event);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(delayMs);
    const TickType_t deadline = xLastWakeTime + pdMS_TO_TICKS(timeoutMs);
    EventDebounce trigger(confirmCycles);

    float startHeading = imu.getHeading();
    // Constant angular rate (NOT the turn PID): fixed differential from omegaDps.
    float halfV = omegaDps * DEG_TO_RAD * MotorController::WHEELBASE_M / 2.0f;

    StopReason reason = StopReason::Timeout;
    for (;;) {
        // Re-command each cycle to keep the velocity loop closed.
        LeftMotor.setTargetVelocity(-halfV);
        RightMotor.setTargetVelocity(halfV);

        if (trigger.poll(event)) { reason = StopReason::Event; break; }

        if (maxAngle > 0.0f && fabsf(imu.getHeading() - startHeading) >= maxAngle) {
            reason = StopReason::Limit; break;
        }

        if ((int32_t)(deadline - xTaskGetTickCount()) <= 0) { reason = StopReason::Timeout; break; }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    if (stopAtEnd) {
        LeftMotor.setTargetVelocity(0.0f);
        RightMotor.setTargetVelocity(0.0f);
    }
    return reason;
}
