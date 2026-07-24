#pragma once
#include <Arduino.h>

class Switch {
    public:
        Switch(uint8_t pin) : mPin(pin){
            pinMode(mPin, INPUT);
        }
        
        bool isPressed() const {
            return digitalRead(mPin) == HIGH;
        }

    private:
        uint8_t mPin;
};