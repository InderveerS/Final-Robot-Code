#include "pid.hpp"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float dt, float outMin, float outMax, float alpha, float integralActivationThreshold) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->dt = dt;
    this->outMin = outMin;
    this->outMax = outMax;
    this->alpha = alpha;
    this->integralActivationThreshold = integralActivationThreshold;
}

float PID::update(float error) {
    float p = kp * error;

    float i = ki * integral;

    float rawD = (error - lastError) / dt;
    float filteredD = alpha * rawD + (1 - alpha) * lastFilteredD;
    lastFilteredD = filteredD;
    float d = kd * filteredD;

    // Update lastError 
    lastError = error;

    // Calculate the PID output
    float output = p + i + d;

    // Update integral with anti-windup
    if(output > outMin && output < outMax) {
        integral += error * dt;
    }

    // Clamp the output
    return clamp(output, outMin, outMax);
}
