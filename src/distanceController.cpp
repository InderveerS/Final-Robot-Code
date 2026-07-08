#include "distanceController.hpp"

DistanceController::DistanceController(MotorController& leftMotor, MotorController& rightMotor, float kp, float ki, float kd, float dt, float alpha) 
    : LeftMotor(leftMotor), RightMotor(rightMotor), distancePID(kp, ki, kd, dt, velMin, velMax, alpha) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
    this->alpha = alpha;
}

void DistanceController::updateDistancePID(float targetDistance) {
    // Get the current distance from the encoders
    float leftDistance = LeftMotor.encoder.getDistance();
    float rightDistance = RightMotor.encoder.getDistance();
    float averageDistance = (leftDistance + rightDistance) / 2.0;

    // Calculate the error
    float error = targetDistance - averageDistance;

    // Update the PID controller
    float output = distancePID.update(error);

    // Set the target velocity for both motors
    LeftMotor.setTargetVelocity(output);
    RightMotor.setTargetVelocity(output);
}