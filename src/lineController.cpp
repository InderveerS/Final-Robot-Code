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
    float leftVel = baseVel - (omega * wheelbase / 2.0f);
    float rightVel = baseVel + (omega * wheelbase / 2.0f);

    LeftMotor.setTargetVelocity(leftVel);
    RightMotor.setTargetVelocity(rightVel);
}