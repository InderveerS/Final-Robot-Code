#include <Arduino.h>
#include "motorController.hpp"
#include "lineController.hpp"
#include "distanceController.hpp"
#include "turnController.hpp"
#include "imu.hpp"

#define LEFT_MOTOR_PIN1 41
#define LEFT_MOTOR_PIN2 42
#define LEFT_ENCODER_PIN1 39
#define LEFT_ENCODER_PIN2 38

#define RIGHT_MOTOR_PIN1 46
#define RIGHT_MOTOR_PIN2 45
#define RIGHT_ENCODER_PIN1 48
#define RIGHT_ENCODER_PIN2 47

const float mdt = 0.01f; // 10 ms loop time
const float lineDt = 0.01f; // 10 ms loop time
const float navDt = 0.01f; // 10 ms loop time

// square test parameters
const float SQUARE_SIDE_M = 0.5f;
const float SQUARE_TURN_DEG = 90.0f; // + is CCW


// MotorController(uint8_t mPin1, uint8_t mPin2, bool mInverted, mcpwm_unit_t mcpwmUnit, mcpwm_timer_t mcpwmTimer,
// uint8_t mEncPin1, uint8_t mEncPin2, pcnt_unit_t mUnit, float kp, float ki, float kd, float dt, float alpha);
MotorController leftMotor(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, false, MCPWM_UNIT_0, MCPWM_TIMER_0,
    LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2, PCNT_UNIT_0, 70.0f, 50.0f, 0.0f, mdt, 0.0f);
MotorController rightMotor(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, true, MCPWM_UNIT_0, MCPWM_TIMER_1,
    RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2, PCNT_UNIT_1, 70.0f, 50.0f, 0.0f, mdt, 0.0f);

IRArray irArray;
Imu robotImu;

LineController lineController(leftMotor, rightMotor, irArray, 0.075f, 0.0f, 0.008f, lineDt, 0.4f);

// DistanceController(left, right, imu, kp, ki, kd, hKp, hKi, hKd, dt, alpha)
// distance gains first, then heading-hold gains
DistanceController distanceController(leftMotor, rightMotor, robotImu, 4.0f, 0.0f, 0.0f, 10.0f, 3.0f, 0.0f, navDt, 0.25f);
TurnController turnController(leftMotor, rightMotor, robotImu, 5.5f, 0.0f, 0.3f, navDt, 0.25f);

// tasks
TaskHandle_t linePIDHandle = NULL;
TaskHandle_t motorPIDHandle = NULL;
TaskHandle_t navHandle = NULL;

// debug snapshot written by nav task, printed by loop() - loop must never
// touch the IMU/Wire directly
volatile float dbgHeading = 0.0f;
volatile float dbgDistance = 0.0f;
volatile int dbgState = 0;

enum NavState { NAV_DRIVE, NAV_TURN, NAV_DONE };

void navTask(void * pvParameters) {
    NavState state = NAV_DRIVE;
    int legCount = 0;

    robotImu.update(); // first read sets the heading zero reference
    distanceController.startMove(SQUARE_SIDE_M);

    for(;;) {
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
        dbgState = state;

        vTaskDelay(pdMS_TO_TICKS(navDt * 1000)); // 100 Hz
    }
}

void linePIDTask(void * pvParameters) {
    for(;;) {
        lineController.updateLinePID();
        vTaskDelay(pdMS_TO_TICKS(lineDt * 1000)); // 100 Hz
    }
}

void motorPIDTask(void * pvParameters) {
    for(;;) {
        lineController.updateMotorPID();
        vTaskDelay(pdMS_TO_TICKS(mdt * 1000)); // 100 Hz
    }
}

// ---- Isolated tuning tests (enable exactly one in setup) ------------------
// One move per boot: edit the constant below, reflash, watch the CSV from
// loop(). The controller keeps updating after settling, so hunting/creep at
// the target shows up in the log.

const float TEST_TURN_DEG = -180.0f; // turnTestTask: + is CCW
const float TEST_DIST_M = 1.5f;    // distanceTestTask: + is forward

volatile int dbgSettled = 0;

void turnTestTask(void * pvParameters) {
    robotImu.update(); // first read sets the heading zero reference
    turnController.turnBy(TEST_TURN_DEG);

    for(;;) {
        robotImu.update();
        turnController.update();

        dbgHeading = robotImu.getHeading();
        dbgSettled = turnController.isSettled() ? 1 : 0;
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

void distanceTestTask(void * pvParameters) {
    robotImu.update(); // first read sets the heading zero reference
    distanceController.startMove(TEST_DIST_M);

    for(;;) {
        robotImu.update();
        distanceController.update();

        dbgHeading = robotImu.getHeading();
        dbgDistance = distanceController.getDistance();
        dbgSettled = distanceController.isSettled() ? 1 : 0;
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

void setup() {

    delay(5000); // also gives the BNO055 gyro time to self-calibrate: keep the robot still

    Serial.begin(115200);

    if (!robotImu.begin()) {
        Serial.println("No BNO055 detected -- check wiring (SDA 40, SCL 21) and address 0x28.");
        while (1) { delay(1000); } // do not drive without a heading reference
    }

    leftMotor.resetPID();
    rightMotor.resetPID();

    // Isolated tuning tests - enable exactly ONE of these (they command the
    // same motors):
    // xTaskCreatePinnedToCore(turnTestTask, "Turn Test", 4096, NULL, 2, &navHandle, 1);
    // xTaskCreatePinnedToCore(distanceTestTask, "Distance Test", 4096, NULL, 2, &navHandle, 1);

    xTaskCreatePinnedToCore(navTask, "Nav Task", 4096, NULL, 2, &navHandle, 1);
    // xTaskCreatePinnedToCore(linePIDTask, "Line PID Task", 4096, NULL, 1, &linePIDHandle, 1);
    // xTaskCreatePinnedToCore(motorPIDTask, "Motor PID Task", 4096, NULL, 2, &motorPIDHandle, 1);
}

// // Duty sweep, forward only. Each step: apply duty, wait out the acceleration
// // transient, then average velocity over a clean steady-state window.
// // Output format: duty, leftVel, rightVel
// const int DUTY_STEP = 5;
// const int SETTLE_MS = 500;   // must exceed the motor's mechanical time constant
// const int MEASURE_MS = 150;  // window the logged velocity is averaged over

int n = 0;
// int duty = 0;
// bool sweepDone = false;

void loop() {
    // Serial.print("state: ");
    // Serial.print(dbgState == 0 ? "DRIVE" : (dbgState == 1 ? "TURN" : "DONE"));
    // Serial.print("  heading(deg): ");
    // Serial.print(dbgHeading, 1); // already degrees
    // Serial.print("  dist(m): ");
    // Serial.println(dbgDistance, 3);
    // delay(100);

    // CSV stream for the tuning test tasks: time_ms, heading_deg, dist_m, settled
    Serial.print(millis());
    Serial.print(", ");
    Serial.print(dbgHeading, 1); // already degrees
    Serial.print(", ");
    Serial.print(dbgDistance, 3);
    Serial.print(", ");
    Serial.println((int)dbgSettled);
    delay(20); // 50 Hz - plenty of resolution for step-response plots

    // leftMotor.setTargetVelocity(0.3f);
    // rightMotor.setTargetVelocity(0.3f);

    // if(n%2 == 0) {
    //     Serial.print(leftMotor.getLastMeasuredVelocity());
    //     Serial.print(", ");
    //     Serial.println(rightMotor.getLastMeasuredVelocity());
    // }

    // if (sweepDone) {
    //     vTaskDelay(pdMS_TO_TICKS(100));
    //     return;
    // }

    // leftMotor.motor.setPWMPercent(duty);
    // rightMotor.motor.setPWMPercent(duty);

    // vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

    // // Return values discarded on purpose: these calls exist only to move each
    // // encoder's window start to now, so the reads below cover the settled
    // // window instead of averaging in the transient above.
    // leftMotor.encoder.getVelocity();
    // rightMotor.encoder.getVelocity();

    // vTaskDelay(pdMS_TO_TICKS(MEASURE_MS));

    // // Both read before printing so the two windows line up and the serial
    // // write doesn't stretch the right motor's window.
    // float vL = leftMotor.encoder.getVelocity();
    // float vR = rightMotor.encoder.getVelocity();

    // Serial.print(duty);
    // Serial.print(", ");
    // Serial.print(vL);
    // Serial.print(", ");
    // Serial.println(vR);

    // duty += DUTY_STEP;
    // if (duty > 100) {
    //     leftMotor.motor.setPWMPercent(0);
    //     rightMotor.motor.setPWMPercent(0);
    //     sweepDone = true;
    // }
}
