#include "lineController.hpp"

LineController::LineController(MotorController& leftMotor, MotorController& rightMotor, IRArray& irArray, float kp, float ki, float kd, float dt, float alpha) : 
    LeftMotor(leftMotor), RightMotor(rightMotor), irArray(irArray), linePID(kp, ki, kd, dt, -maxCorrection, maxCorrection, alpha, 0.0f) 
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
    this->alpha = alpha;
}

void LineController :: updateLinePID() {
    float linePosition = irArray.readLine();
    float error = target - linePosition;
    omega = linePID.update(error);
}

void LineController :: updateMotorPID() {

    realVel = baseVel * (1.0f - VEL_CHANGE_CONST *(fabs(omega) / maxCorrection)); // Adjust base velocity based on angular velocity

    float leftVel = realVel - (omega * wheelbase / 2.0f);
    float rightVel = realVel + (omega * wheelbase / 2.0f);

    LeftMotor.setTargetVelocity(leftVel);
    RightMotor.setTargetVelocity(rightVel);
}