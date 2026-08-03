#include <Arduino.h>
#include "robot.hpp"
#include "mission.hpp"
#include "diagnostics.hpp"
#include "vision.hpp"

void setup() {
    robotBegin();

    // Sensor task: the ONLY task that touches the IMU/I2C. Core 0, high priority
    // so heading stays sampled on schedule; everything else reads cached values.
    xTaskCreatePinnedToCore(imuTask, "IMU Task", 4096, NULL, 4, NULL, 0);

    // Mission task: owns the motors for the whole run. Core 1.
    // xTaskCreatePinnedToCore(missionTask, "Mission Task", 4096, NULL, 3, NULL, 1);

    // To run a diagnostic instead: comment out the mission task above and start
    // ONE task from diagnostics.hpp here. The self-updating tests (turn/distance/
    // nav) must run WITHOUT the imuTask above. Pair a step-response test with
    // csvLogLoop() in loop(); the duty sweep and log helpers print themselves.

    // Serial1 carries EITHER the CSV out to the sd_logging board OR detections
    // in from the vision CAM - never both. Flip ESP_LINK_MODE in linkMode.hpp.
#if ESP_LINK_MODE == ESP_LINK_VISION
    xTaskCreatePinnedToCore(visionTask, "Vision", 3072, NULL, 1, NULL, 0);
#else
    xTaskCreatePinnedToCore(espLoggingTask, "ESP Log", 4096, NULL, 1, NULL, 0);
#endif

    //xTaskCreatePinnedToCore(turnTestTask, "Turn Test", 4096, NULL, 1, NULL, 1);

    // xTaskCreatePinnedToCore(servoTestTask, "Servo Test", 4096, NULL, 1, NULL, 1);

    // xTaskCreatePinnedToCore(turnCountTestC, "TurnC", 4096, NULL, 2, NULL, 1);

    // xTaskCreatePinnedToCore(turnCountTestB, "TurnB", 4096, NULL, 2, NULL, 1);

    xTaskCreatePinnedToCore(turnCountTestD, "TurnD", 4096, NULL, 2, NULL, 1);
}


void loop() {

    // Serial.print(robotImu.getRate());

    // Serial.print(", ");

    // Serial.println(robotImu.getHeading());

    // delay(50);

    // farSensorCalibrateLoop();
    // delay(30);

    // Serial.print(millis());
    // Serial.print(", ");
    // Serial.print(robotImu.getHeading());
    // Serial.print(", ");
    // Serial.print(robotImu.getRawEuler());
    // Serial.print(", ");
    // Serial.print(robotImu.getRate());
    // Serial.print(", ");
    // Serial.println(robotImu.isUsingGyro());
    // delay(100);

    // Serial.print(backLeftSwitch.isPressed());
    // Serial.print(", ");
    // Serial.println(backRightSwitch.isPressed());
    // delay(100);

    // Serial.print(l);
    // Serial.print(", ");
    // Serial.print(c);
    // Serial.print(", ");
    // Serial.println(r);  

    // delay(20);
}
