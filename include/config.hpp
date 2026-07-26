#pragma once
#include <stdint.h>
#include "driver/adc.h"
#include "driver/mcpwm.h"
#include "driver/pcnt.h"

// Single source of truth for pins, peripheral assignments, physical constants,
// and every tuning value. Classes keep their named constants but source the
// values from here, so there is one place to change any number.
namespace cfg {

// ---- Timing ----
constexpr float CONTROL_DT = 0.01f;         // s, control-loop period
constexpr uint16_t CONTROL_PERIOD_MS = 10;  // ms, blocking-primitive period

// ---- Physical ----
constexpr float WHEELBASE_M = 0.254f;
constexpr float WHEEL_CIRCUMFERENCE_M = 0.29225f;
constexpr float COUNTS_PER_REV = 1973.0f;

// ---- Left motor / encoder ----
constexpr uint8_t LEFT_MOTOR_PIN1 = 41;
constexpr uint8_t LEFT_MOTOR_PIN2 = 42;
constexpr uint8_t LEFT_ENCODER_PIN1 = 39;
constexpr uint8_t LEFT_ENCODER_PIN2 = 38;
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr mcpwm_unit_t LEFT_MCPWM_UNIT = MCPWM_UNIT_0;
constexpr mcpwm_timer_t LEFT_MCPWM_TIMER = MCPWM_TIMER_0;
constexpr pcnt_unit_t LEFT_PCNT_UNIT = PCNT_UNIT_0;

// ---- Right motor / encoder ----
constexpr uint8_t RIGHT_MOTOR_PIN1 = 46;
constexpr uint8_t RIGHT_MOTOR_PIN2 = 45;
constexpr uint8_t RIGHT_ENCODER_PIN1 = 48;
constexpr uint8_t RIGHT_ENCODER_PIN2 = 47;
constexpr bool RIGHT_MOTOR_INVERTED = true;
constexpr mcpwm_unit_t RIGHT_MCPWM_UNIT = MCPWM_UNIT_0;
constexpr mcpwm_timer_t RIGHT_MCPWM_TIMER = MCPWM_TIMER_1;
constexpr pcnt_unit_t RIGHT_PCNT_UNIT = PCNT_UNIT_1;

// ---- Motor drive + velocity control ----
constexpr uint16_t MOTOR_PWM_FREQ_HZ = 1000;
constexpr uint16_t MOTOR_DEADTIME_US = 500; // shoot-through guard on direction flip
constexpr float MOTOR_OUT_MIN = -100.0f;
constexpr float MOTOR_OUT_MAX = 100.0f;
constexpr float MOTOR_KP = 70.0f;
constexpr float MOTOR_KI = 50.0f;
constexpr float MOTOR_KD = 0.0f;
constexpr float MOTOR_ALPHA = 0.0f;
constexpr float MOTOR_INTEGRAL_THRESH = 0.1f; // feature hook, currently unused
constexpr float MOTOR_MAX_ACCEL = 0.6f;       // feature hook, currently unused
// Feedforward: duty = sign(v)*FF_DEADBAND + FF_SLOPE*v. Fit of the moving
// region; onset is ~37% but the running line extrapolates to ~21 (static >
// kinetic friction). Below FF_VEL_THRESHOLD we command 0 so the robot can stop.
constexpr float FF_DEADBAND = 21.0f;      // duty %
constexpr float FF_SLOPE = 84.0f;         // duty % per m/s
constexpr float FF_VEL_THRESHOLD = 0.02f; // m/s

// ---- Encoder ----
constexpr uint16_t ENCODER_FILTER = 250; // PCNT glitch filter, APB cycles

// ---- IMU (BNO055) ----
constexpr uint8_t IMU_SDA_PIN = 40;
constexpr uint8_t IMU_SCL_PIN = 21;
constexpr uint32_t IMU_I2C_CLOCK_HZ = 400000UL; // drop to 100000 if flaky
constexpr float IMU_GYRO_SIGN = 1.0f;   // flip if CCW rotation decreases heading
constexpr float IMU_EULER_SIGN = -1.0f; // Euler heading is CW-positive
constexpr float IMU_GYRO_SCALE = 0.974f; // sensitivity trim, (N*360)/reported
constexpr int IMU_GYRO_BIAS_SAMPLES = 100; // x CONTROL_PERIOD_MS of rest averaging
constexpr float IMU_FUSION_HANDOFF_DPS = 40.0f; // above this, integrate raw gyro
constexpr uint16_t IMU_EULER_RESUME_MS = 500;   // gyro-path hold after a fast turn

// ---- IR array ----
constexpr adc1_channel_t IR_FAR_LEFT_CH = ADC1_CHANNEL_0;
constexpr adc1_channel_t IR_LEFT_CH = ADC1_CHANNEL_3;
constexpr adc1_channel_t IR_CENTER_CH = ADC1_CHANNEL_2;
constexpr adc1_channel_t IR_RIGHT_CH = ADC1_CHANNEL_1;
constexpr adc1_channel_t IR_FAR_RIGHT_CH = ADC1_CHANNEL_4;
constexpr uint16_t LINE_PRESENT_THRESHOLD = 250;
constexpr int LINE_MAX_ERROR = 21;         // lost-line sentinel error
constexpr float IR_SENSOR_WIDTH_MM = 10.0f; // spacing between sensors

// ---- Servo (front claw) ----
constexpr uint8_t FRONT_CLAW_PIN = 8;
constexpr uint8_t FRONT_CLAW_MIN_ANGLE = 7;
constexpr uint8_t FRONT_CLAW_MAX_ANGLE = 110;

// ---- Switches ----
constexpr uint8_t BACK_RIGHT_SWITCH_PIN = 7;
constexpr uint8_t BACK_LEFT_SWITCH_PIN = 6;

// ---- Line controller ----
constexpr float LINE_KP = 0.075f;
constexpr float LINE_KI = 0.0f;
constexpr float LINE_KD = 0.008f;
constexpr float LINE_ALPHA = 0.4f;
constexpr float LINE_TARGET = 0.0f;
constexpr float LINE_MAX_CORRECTION = 5.263394f; // omega clamp, rad/s
constexpr float LINE_BASE_VEL = 0.42f;
constexpr float LINE_VEL_CHANGE_CONST = 1.2f; // base-vel reduction vs |omega|

// ---- Distance controller ----
constexpr float DIST_KP = 4.0f;
constexpr float DIST_KI = 0.0f;
constexpr float DIST_KD = 0.0f;
constexpr float DIST_HKP = 10.0f; // heading hold
constexpr float DIST_HKI = 3.0f;
constexpr float DIST_HKD = 0.0f;
constexpr float DIST_ALPHA = 0.25f;
constexpr float DIST_VEL_MAX = 0.5f;
constexpr float DIST_VEL_MIN = -0.5f;
constexpr float DIST_MIN_VEL = 0.10f;           // breakaway floor
constexpr float DIST_MAX_HEADING_OMEGA = 85.0f; // deg/s
constexpr float DIST_SETTLE_TOLERANCE = 0.01f;  // m
constexpr int DIST_SETTLE_CYCLES = 10;

// ---- Turn controller ----
constexpr float TURN_KP = 5.5f;
constexpr float TURN_KI = 0.0f;
constexpr float TURN_KD = 0.3f;
constexpr float TURN_ALPHA = 0.25f;
constexpr float TURN_MAX_OMEGA = 170.0f;       // deg/s
constexpr float TURN_MIN_OMEGA = 70.0f;        // deg/s breakaway floor
constexpr float TURN_SETTLE_TOLERANCE = 0.5f;  // deg
constexpr int TURN_SETTLE_CYCLES = 10;

} // namespace cfg
