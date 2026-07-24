#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>

// Thin wrapper over ESP32Servo (madhephaestus) for basic hobby-servo control.
// ESP32Servo drives servos on the LEDC peripheral; the drive motors use MCPWM,
// so the two never contend for hardware.
class ServoMotor {
    public:
        // minUs/maxUs are the pulse widths (microseconds) mapped to 0 and 180
        // degrees. Defaults span a wide range; narrow them per servo if it
        // buzzes or slams into an endstop at the extremes.
        ServoMotor(uint8_t pin, uint8_t minAngle = 0, uint8_t maxAngle = 180, int minUs = 500, int maxUs = 2500);

        void begin();                    // attach + configure; call once in setup
        void write(int angle);           // 0..180 degrees
        void writeMicroseconds(int us);  // raw pulse width
        int read() const { return mLastAngle; } // last angle commanded
        void open();
        void close();
        void detach();                   // release the pin (servo goes limp)

    private:
        uint8_t mPin;
        int mMinUs;
        int mMaxUs;
        uint8_t mMinAngle;
        uint8_t mMaxAngle;
        int mLastAngle = 90;
        Servo servo;

        static bool timersAllocated;
};
