#include "diagnostics.hpp"
#include "robot.hpp"
#include "telemetry.hpp"
#include "mission.hpp"

// Targets for the single-primitive step-response tests.
static const float TEST_TURN_DEG = -45; // + is CCW
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

void servoTestTask(void* pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(2000));

    for (;;) {
        rearClaw.close();
        vTaskDelay(pdMS_TO_TICKS(3000)); 
        
        rearClaw.open();
        vTaskDelay(pdMS_TO_TICKS(3000));
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

// logging task
void espLoggingTask(void* pvParameters) {
    // CSV header. Column notes:
    //   euler/rate/gyro - IMU internals. heading is the accumulated estimate;
    //     euler is the chip's absolute fused heading and gyro flags which path
    //     built this sample. Compare heading's delta to euler's across a turn
    //     to see whether the ESTIMATE drifted rather than the controller.
    //   FL/FR/L/C/R     - normalized IR (0-1000), read fresh here. FL/FR are
    //     what lineBL/lineBR compare against 400; L+C+R is what lineF compares
    //     against LINE_PRESENT_THRESHOLD, and the three separately show which
    //     sensor actually holds the line. Without these an event fires in the
    //     log with no evidence of why. (The old lineTot column is just L+C+R.)
    //   turnTarget      - set by turn() ONLY. turnUntil() is rate-based and
    //     leaves it alone, so during a *_UNTIL activity it is STALE from the
    //     previous turn(). Do not read it as that section's goal.
    //   seq             - row counter, never reset. This is what tells a row the
    //     ROBOT never sent apart from one the link dropped: contiguous seq with
    //     a jump in t means this task missed its slot (it is the lowest-priority
    //     task on core 0, behind the 100 Hz imuTask); a gap in seq means the row
    //     went out and was lost downstream. Without it the two are identical in
    //     the log.
    vTaskDelay(pdMS_TO_TICKS(200)); // let the ESP-CAM boot and open its SD file

    static const char* HEADER = "t,seq,heading,euler,rate,gyro,roll,dist,distTarget,linePos,FL,FR,L,C,R,turnTarget,Lt,Lm,Lout,Lpwm,Rt,Rm,Rout,Rpwm,step,activity,goal,event";
    robotCommunicator.send(HEADER);
    // Give the ESP-CAM time to open its SD file. Keep this SHORT: missionTask
    // starts driving 200 ms after boot, so anything longer silently drops the
    // opening moves of the run from the log.
    vTaskDelay(pdMS_TO_TICKS(100));

    uint32_t seq = 0;
    for (;;) {
        if (cfg::TELEMETRY_HEADER_EVERY && seq && seq % cfg::TELEMETRY_HEADER_EVERY == 0) {
            robotCommunicator.send(HEADER);
        }

        // Read the ADC outside the varargs call (argument evaluation order is
        // unspecified, and it keeps the sensor access obvious). These are the
        // only ADC reads outside the control tasks; adc1_get_raw locks, and
        // readFarLeft/Right are side-effect free, unlike readLine().
        unsigned fl = (unsigned)irArray.readFarLeft();
        unsigned fr = (unsigned)irArray.readFarRight();
        uint16_t l, c, r;
        irArray.readMiddle(l, c, r); // same three ADC reads getTotal() did

        robotCommunicator.sendf(
            "%lu,%lu,%.1f,%.1f,%.1f,%d,%.1f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%.1f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.3f,%d,%d,%s,%.2f,%s",
            millis(), seq,
            robotImu.getHeading(), robotImu.getRawEuler(), robotImu.getRate(),
            (int)robotImu.isUsingGyro(), robotImu.getRoll(),
            distanceController.getDistance(), distanceController.getTargetDistance(),
            lineController.getLinePosition(),
            fl, fr, (unsigned)l, (unsigned)c, (unsigned)r,
            turnController.getTargetHeading(),
            leftMotor.getTargetVelocity(), leftMotor.getLastMeasuredVelocity(),
            leftMotor.getLastOutput(), leftMotor.getLastPwm(),
            rightMotor.getTargetVelocity(), rightMotor.getLastMeasuredVelocity(),
            rightMotor.getLastOutput(), rightMotor.getLastPwm(),
            telemetry::step, (const char*)telemetry::activity, telemetry::goal,
            eventName((telemetry::EventFn)telemetry::eventPtr));
        seq++;
        vTaskDelay(pdMS_TO_TICKS(cfg::TELEMETRY_PERIOD_MS));
    }
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

