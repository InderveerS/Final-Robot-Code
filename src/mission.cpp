#include "mission.hpp"
#include "robot.hpp"

// Event predicates for the "...Until" primitives. All kept as reusable mission
// vocabulary even when a given run doesn't wire them all in.
bool lineIsDetectedF() { return (irArray.getTotal() >= cfg::LINE_PRESENT_THRESHOLD); }
bool lineIsDetectedBR() { return ((float)irArray.readFarRight() >= 400.0f); }
bool lineIsDetectedBL() { return ((float)irArray.readFarLeft() >= 400.0f); }
bool backRightSwitchPressed() { return backRightSwitch.isPressed(); }
bool backLeftSwitchPressed() { return backLeftSwitch.isPressed(); }
bool onFlatGround() { return robotImu.getRoll() > -3.0f; }
bool useless() { return false; } // placeholder "no event"

static const uint16_t dtMs = cfg::CONTROL_PERIOD_MS;

void missionTask(void* pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(200)); // let the IMU task accumulate a few samples first

    frontClaw.close();
    lineController.followUntil(useless, 0.5f, dtMs, true, 10000);
    turnController.turn(-20.0f, dtMs, 5000);
    // look at telletubby 1
    delay(500);

    turnController.turnUntil(140.0f, lineIsDetectedF, 35.0f, dtMs);
    lineController.followUntil(useless, 0.55f, dtMs, true, 10000);
    turnController.turn(15.0f, dtMs, 5000);
    // look at telletubby 2
    delay(500);

    turnController.turnUntil(-140.0f, lineIsDetectedF, -30.0f, dtMs);
    lineController.followUntil(useless, 0.45f, dtMs, true, 10000);
    turnController.turn(-5.0f, dtMs, 5000);
    // look at telletubby 3
    delay(500);

    turnController.turnUntil(140.0f, lineIsDetectedF, 35.0f, dtMs);
    lineController.followUntil(useless, 0.5f, dtMs, true, 10000);
    turnController.turn(70.0f, dtMs, 5000);
    // look at telletubby 4
    delay(500);

    turnController.turnUntil(140.0f, lineIsDetectedF, 110.0f, dtMs);
    lineController.followUntil(useless, 0.7f, dtMs, false, 10000);
    lineController.followUntil(onFlatGround, 1.5f, dtMs, true, 10000);
    // look at telletubby 5
    delay(500);

    distanceController.move(0.2f, dtMs, 5000);
    turnController.turn(125.0f, dtMs, 5000);
    distanceController.moveUntil(0.4f, lineIsDetectedF, 0.3f, dtMs, false, 10000);

    lineController.followUntil(useless, 0.9f, dtMs, false, 10000);
    lineController.followUntil(lineIsDetectedBL, 0.8f, dtMs, true, 10000);
    delay(100);
    turnController.turn(-100.0f, dtMs, 5000);
    distanceController.move(0.1f, dtMs, 5000);
    // look at telletubby 6
    delay(500);

    distanceController.move(-0.23f, dtMs, 5000);
    turnController.turn(0.0f, dtMs, 5000);
    distanceController.moveUntil(0.25f, lineIsDetectedBL, 0.5f, dtMs, false);

    // start solar move
    distanceController.move(0.45f, dtMs, 5000);
    delay(100);
    turnController.turn(-85.0f, dtMs, 5000);
    delay(200);
    frontClaw.open();
    delay(800);
    distanceController.move(0.25f, dtMs, 5000);
    delay(300);

    // solar second try
    distanceController.move(-0.25f, dtMs, 5000);
    delay(100);
    turnController.turn(-80.0f, dtMs, 5000);
    delay(100);
    distanceController.move(0.25f, dtMs, 5000);
    delay(100);
    turnController.turn(-85.0f, dtMs, 5000);
    delay(100);
    distanceController.move(-0.25f, dtMs, 5000);

    // solar third try
    delay(100);
    turnController.turn(-80.0f, dtMs, 5000);
    delay(100);
    distanceController.move(0.25f, dtMs, 5000);
    delay(100);
    turnController.turn(-85.0f, dtMs, 5000);
    delay(100);
    distanceController.move(-0.25f, dtMs, 5000);

    // solar fourth try
    delay(100);
    turnController.turn(-80.0f, dtMs, 5000);
    delay(100);
    distanceController.move(0.25f, dtMs, 5000);
    delay(100);
    turnController.turn(-85.0f, dtMs, 5000);
    delay(100);
    distanceController.move(-0.25f, dtMs, 5000);

    delay(100);
    frontClaw.close();

    // turnController.turnUntil(20.0f, lineIsDetectedF, 30.0f, dtMs);
    // lineController.followUntil(lineIsDetectedBL, 0.5f, dtMs, true, 10000);
    // turnController.turn(180.0f, dtMs, 5000);
    // distanceController.moveUntil(-0.5f, (lineIsDetectedBR || lineIsDetectedBL), 0.5f, dtMs, false, 10000);

    // A FreeRTOS task must never return: park with the motors stopped.
    for (;;) {
        leftMotor.setTargetVelocity(0.0f);
        rightMotor.setTargetVelocity(0.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
