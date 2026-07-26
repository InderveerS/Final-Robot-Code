#pragma once
#include "config.hpp"
#include "motorController.hpp"
#include "lineController.hpp"
#include "distanceController.hpp"
#include "turnController.hpp"
#include "imu.hpp"
#include "irArray.hpp"
#include "servo.hpp"
#include "switch.hpp"

// Global hardware + controller instances (defined in robot.cpp).
extern MotorController leftMotor;
extern MotorController rightMotor;
extern Imu robotImu;
extern IRArray irArray;
extern ServoMotor frontClaw;
extern Switch backRightSwitch;
extern Switch backLeftSwitch;
extern LineController lineController;
extern DistanceController distanceController;
extern TurnController turnController;

// Hardware bring-up: Serial, servo, IMU (halts on failure), gyro-settle delay,
// motor PID reset. Call once from setup().
void robotBegin();

// Always-on sensor task: the ONLY task that touches the IMU/I2C.
void imuTask(void* pvParameters);
