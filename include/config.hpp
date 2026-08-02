#pragma once
#include <Arduino.h> // HardwareSerial, Serial1
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
// Telemetry row period. 40 ms = 25 Hz: enough to resolve every primitive, and
// half the link load of 50 Hz. The receiver blocks for >100 ms on SD flushes,
// so the slack here is what keeps rows from piling up behind those stalls.
constexpr uint16_t TELEMETRY_PERIOD_MS = 40;
// Re-send the CSV header every N rows (0 disables). Cheap insurance: if the
// first copy is lost the column names still turn up later in the file. At 25 Hz
// this is one extra line every ~20 s; filter with `grep -v '^t,'` if it bothers
// an analysis script.
constexpr uint32_t TELEMETRY_HEADER_EVERY = 500;

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
constexpr float MOTOR_KP = 90.0f;
constexpr float MOTOR_KI = 70.0f;
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

// Static-friction kick. FF_DEADBAND is the intercept of the MOVING-region fit,
// so it systematically under-commands from rest - breaking away takes far more
// duty than sustaining motion. Measured on this robot: a straight move starts
// at ~37% duty, but a POINT TURN needs ~58% because the tyres scrub instead of
// roll (LOG161: a turn sat dead at 46-55% and broke free at 58-59%). While a
// wheel is told to move but reads stopped, this deadband replaces FF_DEADBAND
// so the command clears breakaway at once, instead of waiting ~1 s for the
// velocity integral to crawl there. 37 puts a turn at ~66% and a DIST_MIN_VEL
// move at ~58%: both comfortably past breakaway with margin for battery droop.
// Lower it toward 31 if motion onset feels jerky.
constexpr float FF_STATIC_DEADBAND = 37.0f; // duty %
// Below this measured speed a driven wheel counts as stopped. One encoder count
// per 10 ms cycle is 0.0148 m/s, so this is ~2 counts: above quantisation noise,
// far below any speed the primitives command (DIST_MIN_VEL 0.12, turn floor
// 0.166). The exit threshold is higher so the kick can't chatter on and off
// around the boundary - a 16-point duty step at audio rate would judder.
constexpr float FF_STALL_VEL = 0.03f;      // m/s, engage below this
constexpr float FF_STALL_EXIT_VEL = 0.06f; // m/s, release above this

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
// 40, NOT 80. Raising it to 80 - to keep more rotation on the drift-free fused
// path - measurably backfired: LOG177 regressed heading-vs-euler divergence
// from +1.50 to -3.40 deg and the last turn was visibly ~5 deg out. Euler
// accuracy between 40 and 80 deg/s has never actually been measured; the only
// data point is 0.82% under-count at 140 deg/s (LOG175), so moving ~800 deg of
// rotation onto the fused path in that band was a guess. Do not raise this
// again without first measuring Euler against the gyro across 40-100 deg/s.
constexpr float IMU_FUSION_HANDOFF_DPS = 20.0f; // above this, integrate raw gyro
// 50, was 500. The hold existed to avoid double-counting the fusion's catch-up
// slew after a fast turn - but that slew does not happen on this chip, measured
// two ways. (a) During a hold, euler and heading track to within 0.2 deg over
// 450 ms while the robot decelerates, so the fused output is following the real
// motion, not lagging it. (b) In LOG175 the heading-euler divergence froze at
// exactly +1.80 deg for 20 s after a 20-rotation spin - had the fusion been
// recovering its under-counted angle, that would have decayed toward zero. It
// under-counts permanently and never repays.
//
// Meanwhile the hold was 57% of all gyro-path time (14.9 s of 26.3 s in LOG178),
// spent integrating raw gyro at low rates where euler is perfectly good - which
// is where roughly 1.2 deg of the run's 2.1 deg heading error came from.
//
// Cannot be 0: the test is `now - mLastFastMicros < RESUME_MS`, so zero is false
// even on the sample that sets mLastFastMicros and the gyro path would never
// engage at all. 50 ms is 5 control cycles - enough to ride out rate jitter.
constexpr uint16_t IMU_EULER_RESUME_MS = 100;    // gyro-path hold after a fast turn

// There is deliberately NO continuous bias tracking. An EMA gated on "Euler
// path AND |rate| < 2 deg/s" was tried and removed: driving straight satisfies
// that gate continuously, so it absorbed vibration and heading-hold micro-
// corrections as bias and swung over a 0.22 deg/s range in LOG177 - seven times
// the 0.03 deg/s of real error it was built to remove, and it only settled once
// the robot stopped for good. A tracker needs a WHEELS-STOPPED gate, which
// means feeding motor state into Imu; a rate threshold is not a substitute.
// Imu::captureBias() after the boot settle covers the static case, and that
// part did work - boot bias went from -0.042 to -0.008.

// ---- IR array ----
constexpr adc1_channel_t IR_FAR_LEFT_CH = ADC1_CHANNEL_4;
constexpr adc1_channel_t IR_LEFT_CH = ADC1_CHANNEL_3;
constexpr adc1_channel_t IR_CENTER_CH = ADC1_CHANNEL_2;
constexpr adc1_channel_t IR_RIGHT_CH = ADC1_CHANNEL_1;
constexpr adc1_channel_t IR_FAR_RIGHT_CH = ADC1_CHANNEL_0;
constexpr uint16_t LINE_PRESENT_THRESHOLD = 600;
constexpr int LINE_MAX_ERROR = 21;         // lost-line error
constexpr float IR_SENSOR_WIDTH_MM = 10.0f; // spacing between sensors

// ---- Servo (front claw) ----
constexpr uint8_t FRONT_CLAW_PIN = 15;
constexpr uint8_t FRONT_CLAW_MIN_ANGLE = 10;
constexpr uint8_t FRONT_CLAW_MAX_ANGLE = 135;

// ---- Servo (rear claw) ----
constexpr uint8_t REAR_CLAW_PIN = 8; 
constexpr uint8_t REAR_CLAW_MIN_ANGLE = 5;
constexpr uint8_t REAR_CLAW_MAX_ANGLE = 180;

// ---- ESP-CAM ----
constexpr HardwareSerial* ESP_SERIAL = &Serial1; // pointer (a constexpr ref can't bind Serial1); deref at use
constexpr uint8_t ESP_RX_PIN = 12; // 12 = 47
constexpr uint8_t ESP_TX_PIN = 11; // 11 = 21
constexpr uint32_t ESP_BAUD = 230400; // must match the ESP-CAM's UART baud

// ---- Switches ----
constexpr uint8_t BACK_RIGHT_SWITCH_PIN = 7;
constexpr uint8_t BACK_LEFT_SWITCH_PIN = 6;

// ---- Line controller ----
constexpr float LINE_KP = 0.1f;
constexpr float LINE_KI = 0.0f;
constexpr float LINE_KD = 0.008f;
constexpr float LINE_ALPHA = 0.2f;
constexpr float LINE_TARGET = 0.0f;
constexpr float LINE_MAX_CORRECTION = 5.263394f; // omega clamp, rad/s
constexpr float LINE_BASE_VEL = 0.5f;
constexpr float LINE_VEL_CHANGE_CONST = 1.3f; // base-vel reduction vs |omega|

// ---- Distance controller ----
constexpr float DIST_KP = 4.0f;
constexpr float DIST_KI = 0.0f;
constexpr float DIST_KD = 0.05f;
constexpr float DIST_HKP = 10.0f; // heading hold
constexpr float DIST_HKI = 3.0f;
constexpr float DIST_HKD = 0.0f;
constexpr float DIST_ALPHA = 0.2f;
constexpr float DIST_VEL_MAX = 0.5f;
constexpr float DIST_VEL_MIN = -0.5f;
constexpr float DIST_MIN_VEL = 0.12f;           // breakaway floor
constexpr float DIST_MAX_HEADING_OMEGA = 85.0f; // deg/s
constexpr float DIST_SETTLE_TOLERANCE = 0.005f;  // m
constexpr int DIST_SETTLE_CYCLES = 14;

// ---- Turn controller ----
constexpr float TURN_KP = 5.0f;
constexpr float TURN_KI = 0.0f;
constexpr float TURN_KD = 0.8f;
constexpr float TURN_ALPHA = 0.25f;
constexpr float TURN_MAX_OMEGA = 160.0f;       // deg/s
constexpr float TURN_MIN_OMEGA = 60.0f;        // deg/s breakaway floor
constexpr float TURN_SETTLE_TOLERANCE = 0.5f;  // deg
constexpr int TURN_SETTLE_CYCLES = 12;

// ---- User Input Module ----
constexpr uint8_t USER_INPUT_RAW = 9;
constexpr uint8_t TELETUBBY_LED = 10;
// constexpr uint8_t TELETUBBY_SPEAKER = 13;

} // namespace cfg
