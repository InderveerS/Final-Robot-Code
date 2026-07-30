#include <Arduino.h>
#include "robot.hpp"
#include "mission.hpp"
#include "diagnostics.hpp"

void setup() {
    robotBegin();

    // Sensor task: the ONLY task that touches the IMU/I2C. Core 0, high priority
    // so heading stays sampled on schedule; everything else reads cached values.
    xTaskCreatePinnedToCore(imuTask, "IMU Task", 4096, NULL, 3, NULL, 0);

    // Mission task: owns the motors for the whole run. Core 1.
    xTaskCreatePinnedToCore(missionTask, "Mission Task", 4096, NULL, 2, NULL, 1);

    // To run a diagnostic instead: comment out the mission task above and start
    // ONE task from diagnostics.hpp here. The self-updating tests (turn/distance/
    // nav) must run WITHOUT the imuTask above. Pair a step-response test with
    // csvLogLoop() in loop(); the duty sweep and log helpers print themselves.

    xTaskCreatePinnedToCore(espLoggingTask, "ESP Log", 4096, NULL, 1, NULL, 0);

    //xTaskCreatePinnedToCore(turnTestTask, "Turn Test", 4096, NULL, 1, NULL, 1);

    //xTaskCreatePinnedToCore(servoTestTask, "Servo Test", 4096, NULL, 1, NULL, 1);
}


void loop() {

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
