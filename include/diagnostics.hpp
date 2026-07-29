#pragma once
#include <Arduino.h>

// Tuning/diagnostic scaffolding. To use: start ONE task below from setup()
// instead of the mission, and optionally call its matching print loop from
// loop(). Tasks marked "self-updates IMU" call robotImu.update() themselves,
// so run them WITHOUT the imuTask sensor task (they own the IMU).

// Debug snapshot: written by the step-response test tasks, read by csvLogLoop().
extern volatile float dbgHeading;
extern volatile float dbgDistance;
extern volatile int dbgSettled;

// Single-primitive step-response tests (self-update IMU; edit target constants
// at the top of diagnostics.cpp; pair with csvLogLoop()).
void turnTestTask(void* pvParameters);
void distanceTestTask(void* pvParameters);

// Concurrent line-following (the pre-blocking-primitive approach; needs no IMU).
void linePIDTask(void* pvParameters);
void motorPIDTask(void* pvParameters);

void servoTestTask(void* pvParameters);

// Old square dead-reckoning test (self-updates IMU).
void navTask(void* pvParameters);

// Motor feedforward calibration: steps duty 0..100, prints "duty, vL, vR".
void dutySweepTask(void* pvParameters);

// UART comms with ESPCAM for SD logging (ESPCAM has SD port)
void espLoggingTask(void* pvParameters);

// loop() helpers (stream over Serial; those reading IMU need imuTask running).
void csvLogLoop();             // time_ms, heading_deg, dist_m, settled
void velocityLogLoop();        // drives a fixed duty, prints measured L,R velocity
void tiltLogLoop();            // prints IMU roll, pitch
void farSensorCalibrateLoop(); // accumulates + prints far-sensor min/max
void lineSensorCalibrateLoop();    // accumulates + prints line-sensor min/max
