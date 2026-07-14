#include <Arduino.h>
#include "motorController.hpp"
#include "lineController.hpp"
#include "distanceController.hpp"

#define LEFT_MOTOR_PIN1 41
#define LEFT_MOTOR_PIN2 42
#define LEFT_ENCODER_PIN1 39
#define LEFT_ENCODER_PIN2 38

#define RIGHT_MOTOR_PIN1 46
#define RIGHT_MOTOR_PIN2 45
#define RIGHT_ENCODER_PIN1 48
#define RIGHT_ENCODER_PIN2 47

const float mdt = 0.01f; // 10 ms loop time
const float lineDt = 0.01f; // 50 ms loop time

TickType_t xLastWakeTime;
const TickType_t xPeriod = pdMS_TO_TICKS(10);


// MotorController(uint8_t mPin1, uint8_t mPin2, bool mInverted, mcpwm_unit_t mcpwmUnit, mcpwm_timer_t mcpwmTimer,
// uint8_t mEncPin1, uint8_t mEncPin2, pcnt_unit_t mUnit, float kp, float ki, float kd, float dt, float alpha);
MotorController leftMotor(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, false, MCPWM_UNIT_0, MCPWM_TIMER_0, 
    LEFT_ENCODER_PIN1, LEFT_ENCODER_PIN2, PCNT_UNIT_0, 70.0f, 50.0f, 0.0f, mdt, 0.0f);
MotorController rightMotor(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, true, MCPWM_UNIT_0, MCPWM_TIMER_1, 
    RIGHT_ENCODER_PIN1, RIGHT_ENCODER_PIN2, PCNT_UNIT_1, 70.0f, 50.0f, 0.0f, mdt, 0.0f);

IRArray irArray;

LineController lineController(leftMotor, rightMotor, irArray, 0.07f, 0.0f, 0.007f, lineDt, 0.6f);

// DistanceController distanceController(leftMotor, rightMotor, 50.0f, 0.0f, 0.0f, mdt, 0.6f);

// task
// TaskHandle_t distancePIDHandle = NULL;

// void distancePIDTask(void * pvParameters) {
//     for(;;) {
//         distanceController.updateDistancePID(0.5f); // target distance in meters
//         vTaskDelay(pdMS_TO_TICKS(mdt * 1000)); // 100 Hz
//     }
// }

// // tasks
TaskHandle_t linePIDHandle = NULL;
TaskHandle_t motorPIDHandle = NULL;


void linePIDTask(void * pvParameters) {
    for(;;) {
        lineController.updateLinePID();
        vTaskDelay(pdMS_TO_TICKS(lineDt * 1000)); // 20 Hz
    }
}

void motorPIDTask(void * pvParameters) {
    for(;;) {
        lineController.updateMotorPID();
        vTaskDelay(pdMS_TO_TICKS(mdt * 1000)); // 100 Hz
    }
}


void setup() {


    delay(5000);
    leftMotor.resetPID();
    rightMotor.resetPID();

    // Serial.begin(115200);

    // start the tasks - priority higher for motor
    xTaskCreatePinnedToCore(linePIDTask, "Line PID Task", 2048, NULL, 1, &linePIDHandle, 1);
    xTaskCreatePinnedToCore(motorPIDTask, "Motor PID Task", 2048, NULL, 2, &motorPIDHandle, 1);
    
    // xTaskCreatePinnedToCore(distancePIDTask, "Distance PID Task", 2048, NULL, 1, &distancePIDHandle, 1);

    // distanceController.startDistanceCalculation();
}

void loop() {
    // irArray.calibrateMiddle();

    // Serial.print("MinL: "); Serial.print(irArray.getMinL());
    // Serial.print(", MaxL: "); Serial.print(irArray.getMaxL());
    // Serial.print(", MinC: "); Serial.print(irArray.getMinC());
    // Serial.print(", MaxC: "); Serial.print(irArray.getMaxC());
    // Serial.print(", MinR: "); Serial.print(irArray.getMinR());
    // Serial.print(", MaxR: "); Serial.print(irArray.getMaxR());

    // Serial.println();

    // delay(100); // for readability of output

    // Serial.print(adc1_get_raw(LEFT_CH));
    // Serial.print(" ");
    // Serial.print(adc1_get_raw(CENTER_CH));
    // Serial.print(" ");
    // Serial.print(adc1_get_raw(RIGHT_CH));
    // Serial.print(" ");
    // Serial.print(irArray.readLine());
    // Serial.print(" ");
    // Serial.print(irArray.getTotal());
    // Serial.print(" ");
    // Serial.print(adc1_get_raw(FAR_LEFT_CH));
    // Serial.print(" ");
    // Serial.print(adc1_get_raw(FAR_RIGHT_CH));
    // Serial.println();
    // delay(100);    
}