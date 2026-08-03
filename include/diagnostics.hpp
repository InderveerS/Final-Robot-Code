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

// Per-turn error accumulation. Both rotate 360 deg total and end at the START
// orientation, so any physical offset from a start mark is accumulated error -
// A in 4 turns, B in 36. Error scaling with turn COUNT rather than total angle
// means something is injected once per turn. Run ONE of these instead of the
// mission (they park at the end so the robot can be photographed), 3 repeats
// each, and measure physically - the IMU reports ~360 either way by design.
void turnCountTestA(void* pvParameters); // 4 x 90 deg,  360 total, net 360
void turnCountTestB(void* pvParameters); // 36 x 10 deg, 360 total, net 360
void turnCountTestC(void *pvParameters); // 36 x 90 deg, 3240 total, net 3240
// D matches C in total rotation, turn count and duration but nets to ZERO, so a
// scale error cancels and only time-based bias drift survives. On the mark ->
// scale (trim IMU_GYRO_SCALE); off the mark -> drift (scale trim won't help).
void turnCountTestD(void* pvParameters); // 36 x +/-90, 3240 total, net 0

// UART comms with ESPCAM for SD logging (ESPCAM has SD port)
void espLoggingTask(void* pvParameters);

// loop() helpers (stream over Serial; those reading IMU need imuTask running).
void csvLogLoop();             // time_ms, heading_deg, dist_m, settled
// Latest parsed detection + the accepted/rejected line counts. Needs visionTask
// running. A climbing bad count means ESP_BAUD is too high for the wire - drop
// BOTH ends (ESP_BAUD here, LINK_BAUD in TestBlobs2.0/include/vision_link.h).
void visionMonitorLoop();
void velocityLogLoop();        // drives a fixed duty, prints measured L,R velocity
void tiltLogLoop();            // prints IMU roll, pitch
void farSensorCalibrateLoop(); // accumulates + prints far-sensor min/max
void lineSensorCalibrateLoop();    // accumulates + prints line-sensor min/max
