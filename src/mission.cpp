#include "mission.hpp"
#include "robot.hpp"
#include "telemetry.hpp"

// Event functions
bool lineIsDetectedF() { return (irArray.getTotal() >= cfg::LINE_PRESENT_THRESHOLD); }
bool lineIsDetectedBR() { return ((float)irArray.readFarRight() >= 400.0f); }
bool lineIsDetectedBL() { return ((float)irArray.readFarLeft() >= 400.0f); }
bool backRightSwitchPressed() { return backRightSwitch.isPressed(); }
bool backLeftSwitchPressed() { return backLeftSwitch.isPressed(); }
bool onFlatGround() { return robotImu.getRoll() > -3.0f; }
bool atEnd() {return irArray.getTotal() > 2500; }
bool eitherBackLine() { return lineIsDetectedBL() || lineIsDetectedBR(); }
bool useless() { return false; } // placeholder "no event"

// Maps an event predicate to a short name for telemetry (function-pointer compare).
const char* eventName(telemetry::EventFn p) {
    if (p == lineIsDetectedF)        return "lineF";
    if (p == lineIsDetectedBR)       return "lineBR";
    if (p == lineIsDetectedBL)       return "lineBL";
    if (p == backRightSwitchPressed) return "swBR";
    if (p == backLeftSwitchPressed)  return "swBL";
    if (p == onFlatGround)           return "flat";
    if (p == useless)                return "none";
    return p ? "?" : "-";
}

// delay that also marks the robot as paused for telemetry.
static void missionDelay(uint32_t ms) {
    telemetry::pause();
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void missionTask(void* pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(200)); // let the IMU task accumulate a few samples first

    telemetry::nextStep();
    frontClaw.close();
    lineController.followUntil(useless, 0.5f);
    turnController.turn(-20.0f);
    // look at telletubby 1
    missionDelay(500);

    telemetry::nextStep();
    turnController.turnUntil(140.0f, lineIsDetectedF, 35.0f);
    lineController.followUntil(useless, 0.55f);
    turnController.turn(15.0f);
    // look at telletubby 2
    missionDelay(500);

    telemetry::nextStep();
    turnController.turnUntil(-140.0f, lineIsDetectedF, 30.0f);
    lineController.followUntil(useless, 0.45f);
    turnController.turn(-5.0f);
    // look at telletubby 3
    missionDelay(500);

    telemetry::nextStep();
    turnController.turnUntil(140.0f, lineIsDetectedF, 35.0f);
    lineController.followUntil(useless, 0.5f);
    turnController.turn(70.0f);
    // look at telletubby 4
    missionDelay(500);

    telemetry::nextStep();
    distanceController.move(0.1f);
    turnController.turn(150.0f);
    distanceController.moveUntil(0.44f, useless, 0.3f, 1, false);
    telemetry::nextStep();
    distanceController.moveUntil(0.44f, lineIsDetectedF, 0.5f, 1, false);
    lineController.followUntil(useless, 0.7f, 1, cfg::LINE_BASE_VEL, false);
    lineController.followUntil(onFlatGround, 1.5f);
    // look at telletubby 5
    missionDelay(500);

    telemetry::nextStep();
    lineController.followUntil(useless, 2.4f);
    lineController.followUntil(atEnd, 4.0f, 2, 0.2f);

    turnController.turn(0.0f);
    // distanceController.move(-0.1f);
    turnController.turn(180.0f);

    distanceController.moveUntil(-0.15f, eitherBackLine, 0.5f, 2);
    distanceController.move(0.1f);
    turnController.turn(270.0f);
    distanceController.move(-0.2f);
    turnController.turn(180.0f);



    // lineController.followUntil(lineIsDetectedBL, 0.8f);
    // missionDelay(100);
    // turnController.turn(-100.0f);
    // distanceController.move(0.1f);
    // // look at telletubby 6
    // missionDelay(500);

    // telemetry::nextStep();
    // distanceController.move(-0.23f);
    // turnController.turnUntil(140.0f, useless, -25.0f, 1, false);
    // turnController.turnUntil(130.0f, lineIsDetectedF, 25.0f, 2, true);
    // lineController.followUntil(useless, 0.5f, 1, false);
    // lineController.followUntil(atEnd, 3.0f);



    // distanceController.move(0.2f);
    // turnController.turn(125.0f);
    // distanceController.moveUntil(0.4f, lineIsDetectedF, 0.3f, false);

    // lineController.followUntil(useless, 0.9f, false);
    // lineController.followUntil(lineIsDetectedBL, 0.8f);
    // missionDelay(100);
    // turnController.turn(-100.0f);
    // distanceController.move(0.1f);
    // // look at telletubby 6
    // missionDelay(500);

    // telemetry::nextStep();
    // distanceController.move(-0.23f);
    // turnController.turn(0.0f);
    // distanceController.moveUntil(0.25f, lineIsDetectedBL, 0.5f, false);

    // // start solar move
    // telemetry::nextStep();
    // distanceController.move(0.45f);
    // missionDelay(100);
    // turnController.turn(-85.0f);
    // missionDelay(200);
    // frontClaw.open();
    // missionDelay(800);
    // distanceController.move(0.25f);
    // missionDelay(300);

    // // solar second try
    // telemetry::nextStep();
    // distanceController.move(-0.25f);
    // missionDelay(100);
    // turnController.turn(-80.0f);
    // missionDelay(100);
    // distanceController.move(0.25f);
    // missionDelay(100);
    // turnController.turn(-85.0f);
    // missionDelay(100);
    // distanceController.move(-0.25f);

    // // solar third try
    // telemetry::nextStep();
    // missionDelay(100);
    // turnController.turn(-80.0f);
    // missionDelay(100);
    // distanceController.move(0.25f);
    // missionDelay(100);
    // turnController.turn(-85.0f);
    // missionDelay(100);
    // distanceController.move(-0.25f);

    // // solar fourth try
    // telemetry::nextStep();
    // missionDelay(100);
    // turnController.turn(-80.0f);
    // missionDelay(100);
    // distanceController.move(0.25f);
    // missionDelay(100);
    // turnController.turn(-85.0f);
    // missionDelay(100);
    // distanceController.move(-0.25f);

    // telemetry::nextStep();
    // missionDelay(100);
    // frontClaw.close();

    // turnController.turnUntil(20.0f, lineIsDetectedF, 30.0f, dtMs);
    // lineController.followUntil(lineIsDetectedBL, 0.5f, dtMs, true, 10000);
    // turnController.turn(180.0f, dtMs, 5000);
    // distanceController.moveUntil(-0.5f, (lineIsDetectedBR || lineIsDetectedBL), 0.5f, dtMs, false, 10000);

    // A FreeRTOS task must never return: park with the motors stopped.
    telemetry::setActivity("DONE", 0.0f, false, nullptr);
    for (;;) {
        leftMotor.setTargetVelocity(0.0f);
        rightMotor.setTargetVelocity(0.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
