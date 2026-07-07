#pragma once
#include <stdint.h>
#include <Arduino.h>
#include "driver/mcpwm.h" 

class Motor {
    public:
        Motor(uint8_t mPin1, uint8_t mPin2, bool mInverted);

        // sets duty cycle as positive or negative percentage of full speed (for direction)
        void setPWMPercent(int8_t percent);

        void stop();
        
    private:
        uint8_t mPin1;
        uint8_t mPin2;
        bool mInverted;

        int8_t last_dir = 0;

        const uint16_t freq = 8000; //PWM channel frequency Hz
        // const uint8_t resolution = 8; //PWM channel resolution bits

};
