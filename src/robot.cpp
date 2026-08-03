#include "robot.hpp"

MotorController leftMotor(cfg::LEFT_MOTOR_PIN1, cfg::LEFT_MOTOR_PIN2, cfg::LEFT_MOTOR_INVERTED,
    cfg::LEFT_MCPWM_UNIT, cfg::LEFT_MCPWM_TIMER, cfg::LEFT_ENCODER_PIN1, cfg::LEFT_ENCODER_PIN2,
    cfg::LEFT_PCNT_UNIT, cfg::MOTOR_KP, cfg::MOTOR_KI, cfg::MOTOR_KD, cfg::CONTROL_DT, cfg::MOTOR_ALPHA);

MotorController rightMotor(cfg::RIGHT_MOTOR_PIN1, cfg::RIGHT_MOTOR_PIN2, cfg::RIGHT_MOTOR_INVERTED,
    cfg::RIGHT_MCPWM_UNIT, cfg::RIGHT_MCPWM_TIMER, cfg::RIGHT_ENCODER_PIN1, cfg::RIGHT_ENCODER_PIN2,
    cfg::RIGHT_PCNT_UNIT, cfg::MOTOR_KP, cfg::MOTOR_KI, cfg::MOTOR_KD, cfg::CONTROL_DT, cfg::MOTOR_ALPHA);

Imu robotImu;
IRArray irArray;

ServoMotor frontClaw(cfg::FRONT_CLAW_PIN, cfg::FRONT_CLAW_MIN_ANGLE, cfg::FRONT_CLAW_MAX_ANGLE);
ServoMotor rearClaw(cfg::REAR_CLAW_PIN, cfg::REAR_CLAW_MIN_ANGLE, cfg::REAR_CLAW_MAX_ANGLE);

Switch backRightSwitch(cfg::BACK_RIGHT_SWITCH_PIN);
Switch backLeftSwitch(cfg::BACK_LEFT_SWITCH_PIN);

LineController lineController(leftMotor, rightMotor, irArray,
    cfg::LINE_KP, cfg::LINE_KI, cfg::LINE_KD, cfg::CONTROL_DT, cfg::LINE_ALPHA);

DistanceController distanceController(leftMotor, rightMotor, robotImu,
    cfg::DIST_KP, cfg::DIST_KI, cfg::DIST_KD, cfg::DIST_HKP, cfg::DIST_HKI, cfg::DIST_HKD,
    cfg::CONTROL_DT, cfg::DIST_ALPHA);

TurnController turnController(leftMotor, rightMotor, robotImu,
    cfg::TURN_KP, cfg::TURN_KI, cfg::TURN_KD, cfg::CONTROL_DT, cfg::TURN_ALPHA);

Communicator robotCommunicator(*cfg::ESP_SERIAL, cfg::ESP_RX_PIN, cfg::ESP_TX_PIN, cfg::ESP_BAUD);

void robotBegin() {
    Serial.begin(115200); // Not used in final robot, remove before comp
    frontClaw.begin();
    rearClaw.begin();
    robotCommunicator.begin();

    if (!robotImu.begin()) {
        Serial.println("No BNO055 detected");
        while (1) { delay(1000); } // do not drive without a heading reference
    }

    delay(5000); // let the BNO055 gyro self-calibrate: keep the robot still

    // Measure our own gyro bias only now, on a settled chip. The 5 s above is
    // what lets the BNO055's internal gyro calibration converge, and the rate
    // register it feeds is what we average - so sampling before this delay (as
    // begin() used to) measures an uncalibrated sensor. Robot must still be
    // stationary here.
    robotImu.captureBias();

    // Bias readout, visible without a laptop: the claw parks halfway open if
    // the boot capture looks wrong. mGyroBias is held constant for the whole
    // run, so a bad capture here is a heading error that grows all mission.
    //
    // Close FIRST so mid-travel is unambiguous. ServoMotor starts at
    // mLastAngle = 90, which is also mid-travel, and on a diagnostic run (no
    // missionTask) nothing else ever commands the claw - so without this a
    // never-commanded servo is indistinguishable from a fired warning.
    frontClaw.close();
    Serial.printf("[IMU] boot gyro bias %.4f deg/s (warn above %.3f)\n",
                  robotImu.getGyroBias(), cfg::GYRO_BIAS_WARN_DPS);
    if (fabsf(robotImu.getGyroBias()) > cfg::GYRO_BIAS_WARN_DPS) {
        frontClaw.write(cfg::FRONT_CLAW_FAULT_ANGLE);
        delay(2000); // the mission closes it again right after; a diag run holds it
    }

    leftMotor.resetPID();
    rightMotor.resetPID();
}

void imuTask(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(cfg::CONTROL_PERIOD_MS);
    for (;;) {
        robotImu.update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
