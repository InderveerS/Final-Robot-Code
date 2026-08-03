#include "diagnostics.hpp"
#include "robot.hpp"
#include "telemetry.hpp"
#include "mission.hpp"
#include "vision.hpp"

void visionMonitorLoop() {
    VisionSample s;
    const unsigned long good = (unsigned long)vision.goodCount();
    const unsigned long bad = (unsigned long)vision.badCount();

    if (vision.snapshot(s)) {
        Serial.printf("seq %5u  mask %02X  best %3u  conf %3u  | good %lu bad %lu\n",
                      s.seq, s.mask, s.best, s.conf, good, bad);
    } else {
        Serial.printf("no valid line yet | good %lu bad %lu\n", good, bad);
    }
    delay(200);
}

// Targets for the single-primitive step-response tests.
static const float TEST_TURN_DEG = -45; // + is CCW
static const float TEST_DIST_M = 0.15f;    // + is forward

// Old square dead-reckoning test.
static const float SQUARE_SIDE_M = 0.5f;
static const float SQUARE_TURN_DEG = 90.0f;

volatile float dbgHeading = 0.0f;
volatile float dbgDistance = 0.0f;
volatile int dbgSettled = 0;

// ---------------------------------------------------------------------------
// Per-turn error accumulation test (A vs B).
//
// Both routines rotate 360 deg in total and finish at the STARTING orientation,
// so any physical offset from a start mark is accumulated error. A does it in
// 4 turns, B in 36. If B ends several degrees out while A does not, the error
// scales with the NUMBER of turns rather than the angle turned - which points at
// something injected once per turn. Two candidates fit: the 2-40 deg/s band that
// every accel and decel passes through and which has never been characterised
// (2.5x under-count is measured at 1-2 deg/s, 0.056% accuracy at 140 deg/s,
// nothing in between), and the euler/gyro handoff that every turn crosses twice.
//
// Why this is a clean test: the turn targets are ABSOLUTE and cumulative
// (h0 + step*i), so each turn corrects to its own absolute target and settle
// residual cannot accumulate - the final IMU heading is h0+360 within tolerance
// either way. Only genuine estimate error survives to the end. The IMU will
// report ~360 in both cases; that is the point. Only the photo can see this.
//
// Protocol: mark the chassis against a field wall, run, photograph the same way
// as the mission checkpoints, 3 repeats each. Start with the robot stationary.
// ---------------------------------------------------------------------------
static constexpr float TURN_TEST_TOLERANCE = cfg::TURN_SETTLE_TOLERANCE;
static constexpr uint16_t TURN_TEST_SETTLE_MS = 500; // let the chassis stop between turns

// Shared body so the variants cannot drift apart in semantics. Never returns -
// parks with the motors held at zero so the robot can be photographed.
//
// alternate=false: cumulative targets (h0 + step*i), so net rotation = the total.
// alternate=true : targets ping-pong between h0 and h0+step, so the SAME total
//   rotation and duration produce a NET of zero. That is what separates a scale
//   error (proportional to net rotation, so it cancels here) from bias drift
//   (proportional to elapsed time, so it does not).
static void runTurnCountTest(const char* name, float stepDeg, int steps,
                             bool alternate = false) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // let imuTask accumulate samples first

    const float h0 = robotImu.getHeading();
    const float netDeg = alternate ? 0.0f : stepDeg * (float)steps;
    Serial.printf("\n%s: start heading %.2f | %d steps of %.1f deg | total %.0f, net %.0f\n",
                  name, h0, steps, stepDeg, stepDeg * steps, netDeg);

    // Per-step diagnostics. A 45 deg miss over 36 turns is either ~1.2 deg every
    // turn or one turn losing the lot - those need completely different fixes,
    // and the summary at the end cannot tell them apart. resid is what the turn
    // gave up on; settled=0 means it hit the timeout instead of arriving, which
    // is a controller failure that looks exactly like an IMU fault from outside.
    robotImu.resetFaultCounts();
    const uint32_t t0 = millis();
    int timeouts = 0;
    uint32_t lastGyroF = 0, lastEulerF = 0;
    float worstSpring = 0.0f, springSum = 0.0f;
    Serial.println("step,target,heading,resid,spring,settled,gyro000,euler000,maxDtMs");

    for (int i = 1; i <= steps; i++) {
        // Absolute target. turnTo() takes the shortest path, so cumulative gives
        // a +stepDeg move each time, and alternating gives +step, -step, +step...
        const float target = alternate ? (h0 + stepDeg * (float)(i & 1))
                                       : (h0 + stepDeg * (float)i);
        const bool settled = turnController.turn(target, TURN_TEST_TOLERANCE);
        if (!settled) timeouts++;

        // Heading the instant the controller let go, before the dwell. The
        // difference against h below is rotation that happened with the motors
        // already stopped - tyre wind-up unwinding after a point turn. It is
        // real motion the IMU measures correctly, and nothing corrects it
        // because the turn is over; the NEXT absolute target takes it out. So it
        // shows up only in the FINAL position, with a random sign.
        const float hSettle = robotImu.getHeading();
        vTaskDelay(pdMS_TO_TICKS(TURN_TEST_SETTLE_MS));

        // Printed with the motors stopped, between turns, so it cannot stall one.
        const float h = robotImu.getHeading();
        const float spring = h - hSettle;
        if (fabsf(spring) > fabsf(worstSpring)) worstSpring = spring;
        springSum += fabsf(spring);
        const uint32_t gf = robotImu.getGyroFaults();
        const uint32_t ef = robotImu.getEulerFaults();
        Serial.printf("%d,%.2f,%.2f,%.2f,%.2f,%d,%lu,%lu,%.1f\n",
                      i, target, h, target - h, spring, settled ? 1 : 0,
                      (unsigned long)(gf - lastGyroF), (unsigned long)(ef - lastEulerF),
                      robotImu.getMaxDtUs() / 1000.0f);
        lastGyroF = gf;
        lastEulerF = ef;
    }

    const uint32_t elapsedMs = millis() - t0;
    const float h1 = robotImu.getHeading();
    Serial.printf("%s: DONE in %.1f s. IMU heading %.2f, IMU thinks it moved %.2f (net commanded %.1f)\n",
                  name, elapsedMs / 1000.0f, h1, h1 - h0, netDeg);
    Serial.printf("  timeouts %d/%d | gyro(0,0,0) %lu | euler(0,0,0) %lu | updates %lu | maxDt %.1f ms\n",
                  timeouts, steps,
                  (unsigned long)robotImu.getGyroFaults(),
                  (unsigned long)robotImu.getEulerFaults(),
                  (unsigned long)robotImu.getUpdates(),
                  robotImu.getMaxDtUs() / 1000.0f);
    Serial.printf("  GLITCHES rejected: rate %lu (worst %.1f deg/s) | euler %lu (worst %.1f deg)\n",
                  (unsigned long)robotImu.getRateGlitches(), robotImu.getWorstRate(),
                  (unsigned long)robotImu.getEulerGlitches(), robotImu.getWorstEulerStep());
    // Post-turn springback. Only the LAST turn's is uncorrected, so mean|spring|
    // is roughly the noise floor this test can ever reach.
    Serial.printf("  springback: mean |%.2f| deg, worst %.2f deg over %d turns\n",
                  springSum / (float)steps, worstSpring, steps);
    Serial.println("Photograph the chassis against the wall now - the physical");
    Serial.println("offset from the start mark is the accumulated error.\n");

    telemetry::setActivity("PARK", 0.0f, false, nullptr);
    for (;;) {
        leftMotor.setTargetVelocity(0.0f);
        rightMotor.setTargetVelocity(0.0f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// FEW turns: 4 x 90 deg. Baseline - same total rotation, minimal turn count.
void turnCountTestA(void* pvParameters) {
    runTurnCountTest("TEST A (4 x 90)", 90.0f, 4);
}

// MANY turns: 36 x 10 deg. Same 360 deg total, 9x the turns. Note each 10 deg
// step is below minOmega/TURN_KP (75/5 = 15 deg), so it executes entirely at the
// TURN_MIN_OMEGA floor - which is also what every small mission turn does.
void turnCountTestB(void* pvParameters) {
    runTurnCountTest("TEST B (36 x 10)", 10.0f, 36);
}

void turnCountTestC(void *pvParameters) {
    runTurnCountTest("TEST C (36 X 90)", 90.0f, 36);
}

// SCALE vs DRIFT discriminator. 36 turns alternating +90/-90: identical total
// rotation (3240 deg), turn count and duration to Test C, but NET rotation zero.
//
//   ends ON the mark   -> the error is proportional to NET rotation, i.e. a
//                         scale error. Trim IMU_GYRO_SCALE by C's ratio
//                         (0.974 * 3232/3240 = 0.9716) and re-run C to confirm.
//   ends OFF the mark  -> the error accumulates with TIME, i.e. gyro bias drift.
//                         Scale trimming would not touch it, and it would grow
//                         with the mission's duration rather than its rotation.
//
// Run it right after C, same start mark, same photo method.
void turnCountTestD(void* pvParameters) {
    runTurnCountTest("TEST D (36 x +/-90, net 0)", 90.0f, 36, true);
}

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
    //   bias            - gyro rest bias (raw sensor deg/s) from captureBias().
    //     Constant for the whole run by design, so it is a per-run record of
    //     what the boot capture measured, not a live signal. Compare it across
    //     runs: if it lands in the same place each boot, the capture is
    //     trustworthy and heading error is coming from somewhere else.
    //   FL/FR/L/C/R     - normalized IR (0-1000), read fresh here. FL/FR are
    //     what lineBL/lineBR compare against 400; L+C+R is what lineF compares
    //     against LINE_PRESENT_THRESHOLD, and the three separately show which
    //     sensor actually holds the line. Without these an event fires in the
    //     log with no evidence of why. (The old lineTot column is just L+C+R.)
    //   turnTarget      - set by turn() ONLY. turnUntil() is rate-based and
    //     leaves it alone, so during a *_UNTIL activity it is STALE from the
    //     previous turn(). Do not read it as that section's goal.
    //   encYaw          - heading in degrees derived from the ENCODERS alone:
    //     (Rticks - Lticks) * metersPerCount / WHEELBASE, cumulative from boot.
    //     Completely independent of the IMU, which is the whole point - every
    //     other heading number in this file is IMU-derived and therefore
    //     self-consistent even when the estimate has drifted. Plot
    //     (heading - encYaw) to localise where the two disagree. Trust it during
    //     straight/slow driving; it OVER-reads during point turns (tyre scrub)
    //     and is unreliable on the ramp (wheel slip), so only sustained
    //     divergence on straight sections is evidence.
    //   pitch/rollPP/pitchPP - attitude and its peak-to-peak since the previous
    //     row. update() runs at 100 Hz but this task samples ~22 Hz, so
    //     instantaneous roll/pitch alias chassis rocking away completely. Coning
    //     error (spurious yaw from simultaneous rotation about two axes) scales
    //     with the SQUARE of that amplitude, so the PP columns are the ones that
    //     matter for the ramp question.
    //   seq             - row counter, never reset. This is what tells a row the
    //     ROBOT never sent apart from one the link dropped: contiguous seq with
    //     a jump in t means this task missed its slot (it is the lowest-priority
    //     task on core 0, behind the 100 Hz imuTask); a gap in seq means the row
    //     went out and was lost downstream. Without it the two are identical in
    //     the log.
    vTaskDelay(pdMS_TO_TICKS(200)); // let the ESP-CAM boot and open its SD file

    static const char* HEADER = "t,seq,heading,euler,encYaw,rate,gyro,bias,roll,pitch,rollPP,pitchPP,dist,distTarget,linePos,FL,FR,L,C,R,turnTarget,Lt,Lm,Lout,Lpwm,Rt,Rm,Rout,Rpwm,step,activity,goal,event";
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

        // Encoder-only heading. getCount() is absolute since boot and NOT reset
        // by the primitives (they call startDistance(), which moves the distance
        // reference and leaves the tick counter alone), so this is continuous
        // across the whole run. Sign matches heading: CCW is right wheel forward.
        static constexpr float M_PER_COUNT = cfg::WHEEL_CIRCUMFERENCE_M / cfg::COUNTS_PER_REV;
        float encYaw = (float)(rightMotor.encoder.getCount() - leftMotor.encoder.getCount())
                       * M_PER_COUNT / cfg::WHEELBASE_M * RAD_TO_DEG;

        float rollPP, pitchPP;
        robotImu.takeAttitudePeaks(rollPP, pitchPP); // consumes the window - one reader only

        robotCommunicator.sendf(
            "%lu,%lu,%.1f,%.1f,%.2f,%.1f,%d,%.3f,%.1f,%.1f,%.2f,%.2f,%.3f,%.3f,%.3f,%u,%u,%u,%u,%u,%.1f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.3f,%d,%d,%s,%.2f,%s",
            millis(), seq,
            robotImu.getHeading(), robotImu.getRawEuler(), encYaw, robotImu.getRate(),
            (int)robotImu.isUsingGyro(), robotImu.getGyroBias(),
            robotImu.getRoll(), robotImu.getPitch(), rollPP, pitchPP,
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

