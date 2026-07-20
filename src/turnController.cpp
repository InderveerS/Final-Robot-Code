#include "turnController.hpp"

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
