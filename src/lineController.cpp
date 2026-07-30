#include "lineController.hpp"
#include "telemetry.hpp"

LineController::LineController(MotorController& leftMotor, MotorController& rightMotor, IRArray& irArray, float kp, float ki, float kd, float dt, float alpha) :
    LeftMotor(leftMotor), RightMotor(rightMotor), irArray(irArray), linePID(kp, ki, kd, dt, -maxCorrection, maxCorrection, alpha, 0.0f)
{
}

void LineController::updateLinePID() {
    float linePosition = irArray.readLine();
    mLastLinePos = linePosition;
    float error = target - linePosition;
    omega = linePID.update(error);
}

void LineController::updateMotorPID() {
    // Scale base velocity down as the correction grows, so tight turns slow down.
    realVel = mBaseVel * (1.0f - VEL_CHANGE_CONST * (fabs(omega) / maxCorrection));

    float leftVel = realVel - (omega * MotorController::WHEELBASE_M / 2.0f);
    float rightVel = realVel + (omega * MotorController::WHEELBASE_M / 2.0f);

    LeftMotor.setTargetVelocity(leftVel);
    RightMotor.setTargetVelocity(rightVel);
}

StopReason LineController::followUntil(bool (*event)(), float maxDistance,
                                        uint16_t confirmCycles, float baseVel, bool stopAtEnd,
                                        uint16_t timeoutMs, uint16_t delayMs) {
    telemetry::setActivity("FOLLOW", maxDistance, event != nullptr, event);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(delayMs);
    const TickType_t deadline = xLastWakeTime + pdMS_TO_TICKS(timeoutMs);
    EventDebounce trigger(confirmCycles);
    mBaseVel = baseVel; // set every call, so a previous call's speed can't leak in

    // Mark the distance reference (average encoder travel from here).
    LeftMotor.encoder.startDistance();
    RightMotor.encoder.startDistance();

    StopReason reason = StopReason::Timeout;
    for (;;) {
        updateLinePID();   // outer: sensor -> omega
        updateMotorPID();  // inner: omega -> wheel velocities

        if (trigger.poll(event)) {
            reason = StopReason::Event;
            break;
        }

        float avgDist = (LeftMotor.encoder.getDistance() + RightMotor.encoder.getDistance()) / 2.0f;
        if (maxDistance > 0.0f && avgDist >= maxDistance) {
            reason = StopReason::Limit;
            break;
        }

        if ((int32_t)(deadline - xTaskGetTickCount()) <= 0) {
            reason = StopReason::Timeout;
            break;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    if (stopAtEnd) {
        LeftMotor.resetPID();
        RightMotor.resetPID();

        LeftMotor.setTargetVelocity(0.0f);
        RightMotor.setTargetVelocity(0.0f);
    }
    return reason;
}