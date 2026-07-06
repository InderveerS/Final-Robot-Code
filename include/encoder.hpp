#pragma once
#include <Arduino.h>
#include <stdint.h>

class Encoder {
    public:
        Encoder(uint8_t pinA, uint8_t pinB);
        void begin();
        int32_t getCount();
        float getVelocity(float dt);
        float startDistance();
        float getDistance(); // resets distance to 0
        void resetCount();

    private:
        uint8_t mPinA;
        uint8_t mPinB;
        volatile int16_t mCount = 0.0;
        float mDistance = 0.0;
};