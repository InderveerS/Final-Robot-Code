#include "mission.hpp"
#include "robot.hpp"
#include "telemetry.hpp"
#include "vision.hpp"

// 6 candidate spots, 2 teletubbies. Owned solely by missionTask.
static TubbyHunt tubbies;

// Event functions
bool lineIsDetectedF() { return (irArray.getTotal() >= cfg::LINE_PRESENT_THRESHOLD); }
bool lineIsDetectedBR() { return ((float)irArray.readFarRight() >= 400.0f); }
bool lineIsDetectedBL() { return ((float)irArray.readFarLeft() >= 400.0f); }
bool backRightSwitchPressed() { return backRightSwitch.isPressed(); }
bool backLeftSwitchPressed() { return backLeftSwitch.isPressed(); }
bool eitherSwitchPressed() { return backRightSwitch.isPressed() || backLeftSwitch.isPressed(); };
bool onFlatGround() { return robotImu.getRoll() > -3.0f; }
bool atEnd() {return irArray.getTotal() > 2500; }
bool eitherBackLine() { return lineIsDetectedBL() || lineIsDetectedBR(); }
bool notAtEnd() { return !atEnd(); }
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

void calibrateOnEnd(void) {
    turnController.turnUntil(100.0f, notAtEnd, 35.0f, 2);
    float end1 = robotImu.getHeading();
    turnController.turnUntil(-100.0f, useless, 0.0f, 1, false);
    turnController.turnUntil(-100.0f, notAtEnd, -35.0f, 2);
    float end2 = robotImu.getHeading();

    float newZero = (end1 + end2)/2.0f;
    robotImu.setHeading(newZero);
}

void missionTask(void* pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(200)); // let the IMU task accumulate a few samples first

    // Start counting I2C read failures only now. Everything before this point
    // is the boot settle with the robot stationary and level, where Euler
    // genuinely reads (0,0,0) and would count as pure fault.
    robotImu.resetFaultCounts();

    // turnController.turnUntil( 140.0f, nullptr, 3600.0f, 1, true, 40000);  // 10 rotations CCW
    // // turnController.turnUntil(-140.0f, nullptr, 3600.0f, 1, true, 40000);  // 10 rotations back CW
    // turnController.turn(0.0f);

    telemetry::nextStep();
    tubbies.begin();
    frontClaw.close();
    rearClaw.open();

    // Spots 1-3 are closed, pose-neutral excursions: aim, look, recover to the
    // line. Skipping one leaves the robot exactly where scanning it would have,
    // so nothing downstream can tell the difference.
    //
    // `scan` is hoisted so the follow and the block cannot disagree, and feeds
    // stopAtEnd - stop for the aim turn, coast straight through once both
    // teletubbies are found. stopAtEnd is parameter 5, so confirmCycles and
    // baseVel MUST be spelled out; a bare bool there silently becomes one of
    // them.
    bool scan = tubbies.hunting();
    lineController.followUntil(useless, 0.5f, 1, cfg::LINE_BASE_VEL, scan);
    if (scan) { // spot 1
        turnController.turn(-20.0f, 3.0f);
        tubbies.look(1);
        turnController.turnUntil(140.0f, lineIsDetectedF, 35.0f);
    }

    telemetry::nextStep();
    scan = tubbies.hunting();
    lineController.followUntil(useless, 0.55f, 1, cfg::LINE_BASE_VEL, scan);
    if (scan) { // spot 2
        turnController.turn(15.0f, 3.0f);
        tubbies.look(2);
        turnController.turnUntil(-130.0f, lineIsDetectedF, 30.0f);
    }

    telemetry::nextStep();
    scan = tubbies.hunting();
    lineController.followUntil(useless, 0.45f, 1, cfg::LINE_BASE_VEL, scan);
    if (scan) { // spot 3
        turnController.turn(-5.0f, 2.0f);
        tubbies.look(3);
        turnController.turnUntil(130.0f, lineIsDetectedF, 35.0f);
    }

    // Spot 4 is different: turn(70) is NAVIGATION, not an aim. The move(0.39)
    // below runs at that heading onto the ramp approach, so skipping it drives
    // 39 cm the wrong way. Only the dwell is conditional, and the follow always
    // stops because the turn is unconditional.
    telemetry::nextStep();
    lineController.followUntil(useless, 0.5f);
    turnController.turn(70.0f, 3.0f);
    if (tubbies.hunting()) tubbies.look(4);

    telemetry::nextStep();
    distanceController.move(0.39f);
    turnController.turn(165.0f);
    // >>> DIAG P1: pre-ramp photo. turn() stops the motors, so the robot is
    // genuinely stationary here - a coasting primitive would keep driving
    // through a missionDelay. Everything from here to P2 is "the ramp section",
    // approach included. Remove for competition.
    // missionDelay(5000);
    distanceController.moveUntil(0.34f, useless, 0.3f, 1, false);
    telemetry::nextStep();
    distanceController.moveUntil(0.44f, lineIsDetectedF, 0.5f, 1, false);
    lineController.followUntil(useless, 0.7f, 1, cfg::LINE_BASE_VEL, false);

    // Spot 5 has no aim turn - the look happens wherever the ramp exit leaves
    // us. Once both are found the robot must not even slow down here, so the
    // ramp-exit follow coasts instead of stopping.
    scan = tubbies.hunting();
    lineController.followUntil(onFlatGround, 1.5f, 1, cfg::LINE_BASE_VEL, scan);
    // >>> DIAG P2: post-ramp photo. (P2 - P1) measured from the photos is the
    // TRUE heading change across the ramp; compare against the log's heading
    // change over the same interval. Remove for competition. NOTE: the follow
    // above only stops when scan is true, so force scan (or pass true) before
    // photographing here - otherwise the robot coasts through the pause.
    // missionDelay(5000);
    if (scan) tubbies.look(5);

    telemetry::nextStep();

    lineController.followUntil(useless, 0.7f, 1, cfg::LINE_BASE_VEL, false);

    // Spot 6 is a real if/else, not a guard: the scan path turns off the line,
    // looks, and drives back onto it, while the skip path just keeps following
    // from the back corner to the end. The two rejoin the line at different
    // places, so the fast-follow that follows is not the same length. The
    // follow after THAT is event-terminated with a 4 m cap, so short only
    // costs time.
    scan = tubbies.hunting();
    lineController.followUntil(lineIsDetectedBL, 0.8f, 2, cfg::LINE_BASE_VEL, scan);
    float fastFollowM = cfg::TUBBY6_SKIP_FOLLOW_M;
    if (scan) {
        missionDelay(100);
        //float curr = robotImu.getHeading();
        turnController.turn(-95.0f, 3.0f);
        // distanceController.move(0.1f); // scan path only
        tubbies.look(6);

        telemetry::nextStep();
        // distanceController.move(-0.23f);
        // turnController.turnUntil(140.0f, useless, -45.0f, 1, false);
        turnController.turn(5.0f, 3.0f);
        distanceController.moveUntil(0.3f, lineIsDetectedF, 0.6, 2, false);
        // turnController.turn();
        fastFollowM = 1.1f;
    }
    lineController.followUntil(useless, fastFollowM);
    lineController.followUntil(atEnd, 4.0f, 2, 0.2f);

    // >>> DIAG: mitigation disabled for the diagnostic runs - we need the raw
    // drift behaviour, not a corrected version of it. RESTORE for competition.
    // robotImu.setHeading(-8.2f);
    // turnController.turn(0.0f);
    // robotImu.setHeading(0.0f);
    missionDelay(300);
    distanceController.move(-0.05f);// xTaskCreatePinnedToCore(turnCountTestD, "TurnD", 4096, NULL, 2, NULL, 1); 

    turnController.turn(-90.0f);
    missionDelay(300);
    distanceController.move(-0.115f);
    turnController.turn(180.0f);
    missionDelay(300);
    // >>> DIAG P3: park at the failure point so it can be photographed against
    // a field wall. Everything below is unreachable until this block is removed.
    // Motors are re-zeroed each cycle so nothing creeps while you photograph.
    // telemetry::setActivity("PARK", 0.0f, false, nullptr);
    // for (;;) {
    //     leftMotor.setTargetVelocity(0.0f);
    //     rightMotor.setTargetVelocity(0.0f);
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }
    distanceController.moveUntil(-0.4f, lineIsDetectedBR, 0.12f, 2, false);
    distanceController.moveUntil(-0.2f, eitherSwitchPressed, 0.3f, 2);
    rearClaw.close();
    missionDelay(300);

    //find habitat boss
    // distanceController.move(0.1f);
    // turnController.turn(190.0f);
    // distanceController.moveUntil(0.44f, lineIsDetectedF, 2.0f, 1, false);
    // lineController.followUntil(lineIsDetectedBR, 2.0f, 1, 0.15f);
    // turnController.turn(270.0f);
    // distanceController.move(-0.175f);
    // rearClaw.open();

    // try hardcode
    distanceController.moveUntil(0.15f, eitherBackLine, 0.3f);
    distanceController.move(0.2f);
    turnController.turn(-90.0f);
    distanceController.move(-0.355f);
    turnController.turn(0.0f);
    distanceController.move(-0.13f);
    missionDelay(200);
    rearClaw.open();

    // go to second habitat
    // distanceController.move(0.155f);
    // turnController.turn(90.0f);
    // turnController.turnUntil(-150.0f, lineIsDetectedF, -40.0f, 2);

    distanceController.move(0.15f);
    turnController.turn(-90.0f);    
    distanceController.moveUntil(0.4f, useless, 0.2f, 1, false);
    distanceController.moveUntil(0.2f, lineIsDetectedBL, 0.36f, 2, false);
    distanceController.move(0.06f);
    // distanceController.moveUntil(0.4f, useless, 0.2f, 1, false);
    // distanceController.moveUntil(0.2f, eitherBackLine, 0.3f, 2, false);
    // distanceController.move(0.1f);

    // turnController.turn(0.0f);
    // distanceController.moveUntil(0.2f, lineIsDetectedF, 0.4f, 2, false);

    // lineController.followUntil(atEnd, 0.8f, 2, 0.2f);
    // turnController.turn(0.0f);
    // distanceController.move(-0.05f);
    // turnController.turn(-90.0f);
    // distanceController.move(0.115f);
    turnController.turn(180.0f);
    distanceController.moveUntil(-0.4f, eitherBackLine, 0.11f, 2, false);
    distanceController.moveUntil(-0.2f, eitherSwitchPressed, 0.3f, 2);
    rearClaw.close();
    delay(200);

    // find boss second time
    distanceController.moveUntil(0.3f, eitherBackLine, 0.35f, 2);
    turnController.turn(120.0f);
    distanceController.move(0.15f);
    turnController.turnUntil(150.0f, lineIsDetectedF, 195.0f, 2);
    lineController.followUntil(useless, 0.2f, 1, 0.44f, false);
    lineController.followUntil(lineIsDetectedBR, 2.0f, 1, 0.15f);
    // distanceController.move(0.02f);
    turnController.turn(270.0f);
    distanceController.move(-0.21f);
    rearClaw.open();

    distanceController.move(0.1f);

    // go to third habitat
    //turnController.turn(-40.0f); 
    if(turnController.turnUntil(130.0f, lineIsDetectedF, -45.0f, 2) != StopReason::Event) {
        distanceController.moveUntil(0.4f, lineIsDetectedF, 0.2f, 1, false);
    }
    lineController.followUntil(useless, 0.1f, 1, 0.44f, false);
    lineController.followUntil(atEnd, 2.0f, 1, 0.2f);
    // robotImu.setHeading(-12.0f); // second setHeading
    // turnController.turn(0.0f);
    distanceController.move(-0.05f);
    turnController.turn(90.0f);
    distanceController.move(-0.375f);
    turnController.turn(180.0f, 0.25f);
    distanceController.moveUntil(-0.4f, eitherBackLine, 0.12f, 2, false);
    distanceController.moveUntil(-0.2f, eitherSwitchPressed, 0.3f, 2);
    rearClaw.close();
    delay(200);

    // find boss 3rd time
    distanceController.move(0.1f);
    turnController.turn(120.0f);
    distanceController.move(0.42f);
    // turnController.turn(180.0f);
    // distanceController.moveUntil(0.4f, lineIsDetectedF, 0.2f, 1, false);
    turnController.turnUntil(150.0f, lineIsDetectedF, 130.0f, 2);
    lineController.followUntil(useless, 0.2f, 1, 0.42, false);
    lineController.followUntil(lineIsDetectedBR, 2.0f, 1, 0.15f);
    turnController.turn(180.0f);
    distanceController.move(0.29f);
    turnController.turn(270.0f);
    distanceController.move(-0.375f);
    turnController.turn(180.0f);
    distanceController.move(-0.14f, 2500);
    rearClaw.open();
    // distanceController.move(-0.02f, 1000);
    distanceController.move(0.32f);
    
    turnController.turn(-90.0f, 0.3f);
    frontClaw.open();
    distanceController.move(0.63f, 3500);
    delay(200);
    turnController.turn(0.0f, 3.0f); 
    // frontClaw.close();
    // distanceController.move(-0.2f);

    // END CODE HERE

    // The I2C-fault claw indicator that used to sit here is gone: it measured
    // ZERO across a full run, so the bus is not dropping reads, and leaving it
    // in risked releasing the final grab for a signal that never fires.
    // robotImu.getEulerFaults()/getGyroFaults() are still there if it is ever
    // worth re-checking. The live readout is now the BOOT bias check in
    // robotBegin(), which fires before the claw has hold of anything.

    // A FreeRTOS task must never return: park with the motors stopped.
    telemetry::setActivity("DONE", 0.0f, false, nullptr);
    for (;;) {
        leftMotor.setTargetVelocity(0.0f);
        rightMotor.setTargetVelocity(0.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
