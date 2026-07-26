#include "diagnostics.hpp"
#include "robot.hpp"

// Targets for the single-primitive step-response tests.
static const float TEST_TURN_DEG = -15.0f; // + is CCW
static const float TEST_DIST_M = 0.15f;    // + is forward

// Old square dead-reckoning test.
static const float SQUARE_SIDE_M = 0.5f;
static const float SQUARE_TURN_DEG = 90.0f;

volatile float dbgHeading = 0.0f;
volatile float dbgDistance = 0.0f;
volatile int dbgSettled = 0;

void turnTestTask(void* pvParameters) {
    robotImu.update(); // first read sets the heading zero reference
    turnController.turnBy(TEST_TURN_DEG);

    for (;;) {
        robotImu.update();
        turnController.update();

        dbgHeading = robotImu.getHeading();
        dbgSettled = turnController.isSettled() ? 1 : 0;
        vTaskDelay(pdMS_TO_TICKS(cfg::CONTROL_PERIOD_MS));
    }
}

void distanceTestTask(void* pvParameters) {
    robotImu.update();
    distanceController.startMove(TEST_DIST_M);

    for (;;) {
        robotImu.update();
        distanceController.update();

        dbgHeading = robotImu.getHeading();
        dbgDistance = distanceController.getDistance();
        dbgSettled = distanceController.isSettled() ? 1 : 0;
        vTaskDelay(pdMS_TO_TICKS(cfg::CONTROL_PERIOD_MS));
    }
}

void linePIDTask(void* pvParameters) {
    for (;;) {
        lineController.updateLinePID();
        vTaskDelay(pdMS_TO_TICKS(cfg::CONTROL_PERIOD_MS));
    }
}

void motorPIDTask(void* pvParameters) {
    for (;;) {
        lineController.updateMotorPID();
        vTaskDelay(pdMS_TO_TICKS(cfg::CONTROL_PERIOD_MS));
    }
}

void navTask(void* pvParameters) {
    enum NavState { NAV_DRIVE, NAV_TURN, NAV_DONE };
    NavState state = NAV_DRIVE;
    int legCount = 0;

    robotImu.update();
    distanceController.startMove(SQUARE_SIDE_M);

    for (;;) {
        robotImu.update();

        switch (state) {
            case NAV_DRIVE:
                distanceController.update();
                if (distanceController.isSettled()) {
                    turnController.turnBy(SQUARE_TURN_DEG);
                    state = NAV_TURN;
                }
                break;

            case NAV_TURN:
                turnController.update();
                if (turnController.isSettled()) {
                    legCount++;
                    if (legCount >= 4) {
                        state = NAV_DONE;
                    } else {
                        distanceController.startMove(SQUARE_SIDE_M);
                        state = NAV_DRIVE;
                    }
                }
                break;

            case NAV_DONE:
                leftMotor.setTargetVelocity(0.0f);
                rightMotor.setTargetVelocity(0.0f);
                break;
        }

        dbgHeading = robotImu.getHeading();
        dbgDistance = distanceController.getDistance();
        vTaskDelay(pdMS_TO_TICKS(cfg::CONTROL_PERIOD_MS));
    }
}

void dutySweepTask(void* pvParameters) {
    const int DUTY_STEP = 5;
    const int SETTLE_MS = 500;  // exceed the mechanical time constant
    const int MEASURE_MS = 150; // steady-state averaging window

    for (int duty = 0; duty <= 100; duty += DUTY_STEP) {
        leftMotor.motor.setPWMPercent(duty);
        rightMotor.motor.setPWMPercent(duty);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

        // Discard reads to move each encoder's window start past the transient.
        leftMotor.encoder.getVelocity();
        rightMotor.encoder.getVelocity();
        vTaskDelay(pdMS_TO_TICKS(MEASURE_MS));

        float vL = leftMotor.encoder.getVelocity();
        float vR = rightMotor.encoder.getVelocity();
        Serial.print(duty);
        Serial.print(", ");
        Serial.print(vL);
        Serial.print(", ");
        Serial.println(vR);
    }

    leftMotor.motor.setPWMPercent(0);
    rightMotor.motor.setPWMPercent(0);
    for (;;) vTaskDelay(pdMS_TO_TICKS(100));
}

void csvLogLoop() {
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(dbgHeading, 1);
    Serial.print(", ");
    Serial.print(dbgDistance, 3);
    Serial.print(", ");
    Serial.println((int)dbgSettled);
    delay(20);
}

void velocityLogLoop() {
    leftMotor.motor.setPWMPercent(50);
    rightMotor.motor.setPWMPercent(50);
    Serial.print(leftMotor.encoder.getVelocity());
    Serial.print(", ");
    Serial.println(rightMotor.encoder.getVelocity());
    delay(10);
}

void tiltLogLoop() {
    Serial.print("roll: ");
    Serial.print(robotImu.getRoll());
    Serial.print("  pitch: ");
    Serial.println(robotImu.getPitch());
    delay(20);
}

void farSensorCalibrateLoop() {
    irArray.calibrateFarLeft();
    irArray.calibrateFarRight();
    Serial.print("MinFR: ");
    Serial.print(irArray.getMinFR());
    Serial.print(", MaxFR: ");
    Serial.print(irArray.getMaxFR());
    Serial.print(", MinFL: ");
    Serial.print(irArray.getMinFL());
    Serial.print(", MaxFL: ");
    Serial.println(irArray.getMaxFL());
    delay(50);
}

void lineSensorCalibrateLoop() {
    irArray.calibrateMiddle();
    Serial.print("MinL: ");
    Serial.print(irArray.getMinL());
    Serial.print(", MaxL: ");
    Serial.print(irArray.getMaxL());
    Serial.print(", MinC: ");
    Serial.print(irArray.getMinC());
    Serial.print(", MaxC: ");
    Serial.print(irArray.getMaxC());
    Serial.print(", MinR: ");
    Serial.print(irArray.getMinR());
    Serial.print(", MaxR: ");
    Serial.println(irArray.getMaxR());
    delay(50);
}