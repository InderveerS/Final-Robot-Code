#include <Arduino.h>
#include "motorController.hpp"
#include "lineController.hpp"

#define LEFT_MOTOR_PIN1 5
#define LEFT_MOTOR_PIN2 4
#define LEFT_ENCODER_PIN1 18
#define LEFT_ENCODER_PIN2 19

#define RIGHT_MOTOR_PIN1 17
#define RIGHT_MOTOR_PIN2 16
#define RIGHT_ENCODER_PIN1 21
#define RIGHT_ENCODER_PIN2 22

// actually fix the pins and PID constants later
// MotorController(uint8_t mPin1, uint8_t mPin2, bool mInverted, uint8_t mEncPin1, uint8_t mEncPin2, pcnt_unit_t mUnit, float kp, float ki, float kd, float dt, float alpha);

MotorController leftMotor(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, false, LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2, PCNT_UNIT_0, 1.0f, 0.0f, 0.0f, 0.01f, 0.1f);
MotorController rightMotor(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, true, RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2, PCNT_UNIT_1, 1.0f, 0.0f, 0.0f, 0.01f, 0.1f);

IRArray irArray;

LineController lineController(leftMotor, rightMotor, irArray, 1.0f, 0.0f, 0.0f, 0.01f, 0.1f);

// tasks
TaskHandle_t linePIDHandle = NULL;
TaskHandle_t motorPIDHandle = NULL;

void linePIDTask(void * pvParameters) {
    for(;;) {
        lineController.updateLinePID();
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
    }
}

void motorPIDTask(void * pvParameters) {
    for(;;) {
        lineController.updateMotorPID();
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz
    }
}

void setup() {
    // start the tasks - priority higher for motor
    xTaskCreatePinnedToCore(linePIDTask, "Line PID Task", 2048, NULL, 1, &linePIDHandle, 1);
    xTaskCreatePinnedToCore(motorPIDTask, "Motor PID Task", 2048, NULL, 2, &motorPIDHandle, 1);
}

void loop() {

}